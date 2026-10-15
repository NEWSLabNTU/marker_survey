"""A minimal WebSocket (RFC 6455) on top of http.server: enough for /api/live.

The server sends text (JSON) and binary (JPEG) messages; the page sends small
JSON text messages. No extensions, no compression; a message from the page
may be fragmented, the server never fragments. Standard library only.
"""

import base64
import hashlib
import struct

GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'
TEXT, BINARY, CLOSE, PING, PONG, CONT = 0x1, 0x2, 0x8, 0x9, 0xA, 0x0
MAX_IN = 1 << 20          # the page sends small messages only


class Closed(Exception):
    pass


def accept_key(key: str) -> str:
    return base64.b64encode(hashlib.sha1((key.strip() + GUID).encode()).digest()).decode()


def is_upgrade(headers) -> bool:
    return ('websocket' in (headers.get('Upgrade') or '').lower()
            and bool(headers.get('Sec-WebSocket-Key')))


def handshake(handler) -> None:
    """Answer the upgrade request of a BaseHTTPRequestHandler."""
    handler.send_response(101, 'Switching Protocols')
    handler.send_header('Upgrade', 'websocket')
    handler.send_header('Connection', 'Upgrade')
    handler.send_header('Sec-WebSocket-Accept', accept_key(handler.headers['Sec-WebSocket-Key']))
    handler.end_headers()
    handler.wfile.flush()


def encode(opcode: int, payload: bytes) -> bytes:
    n = len(payload)
    if n < 126:
        head = struct.pack('!BB', 0x80 | opcode, n)
    elif n < 1 << 16:
        head = struct.pack('!BBH', 0x80 | opcode, 126, n)
    else:
        head = struct.pack('!BBQ', 0x80 | opcode, 127, n)
    return head + payload


def _read(rfile, n: int) -> bytes:
    data = b''
    while len(data) < n:
        chunk = rfile.read(n - len(data))
        if not chunk:
            raise Closed()
        data += chunk
    return data


def read_frame(rfile):
    """(fin, opcode, payload) of one frame; raises Closed at the end of the stream."""
    b0, b1 = _read(rfile, 2)
    fin, opcode = bool(b0 & 0x80), b0 & 0x0F
    masked, n = bool(b1 & 0x80), b1 & 0x7F
    if n == 126:
        n = struct.unpack('!H', _read(rfile, 2))[0]
    elif n == 127:
        n = struct.unpack('!Q', _read(rfile, 8))[0]
    if n > MAX_IN:
        raise Closed()
    mask = _read(rfile, 4) if masked else None
    payload = _read(rfile, n)
    if mask:
        payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    return fin, opcode, payload


def read_message(rfile, send):
    """(opcode, payload) of the next whole data message; answers pings with `send`."""
    parts, first = [], None
    while True:
        fin, op, payload = read_frame(rfile)
        if op == CLOSE:
            raise Closed()
        if op == PING:
            send(encode(PONG, payload))
            continue
        if op == PONG:
            continue
        if op != CONT:
            first, parts = op, []
        parts.append(payload)
        if fin:
            return first, b''.join(parts)
