"""Spin a node until Ctrl-C or SIGTERM, and return within a bounded time. ROS.

rclpy.spin waits with no timeout, and on SIGTERM relies on rclpy's own signal
handling to shut the context down and wake the wait. Once, in CI on Jazzy, a
survey node did not exit on SIGTERM after its work was done (with sim time
and the recording over, no timer of it fired any more); it could not be
reproduced. Here SIGTERM sets a flag in Python and the executor waits at most
`period` seconds at a time, so the loop sees the flag (or a shut-down context)
within that, whatever wakes or does not wake the wait.
"""
import signal

from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor


def spin(node, period: float = 0.5) -> str:
    """Run the node's callbacks until SIGINT (KeyboardInterrupt), SIGTERM or the
    context's shutdown; returns which ('interrupt', 'terminate', 'shutdown')."""
    stop = []

    def on_term(signum, frame):
        stop.append(signum)

    old = signal.signal(signal.SIGTERM, on_term)
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    why = 'shutdown'
    try:
        while not stop and node.context.ok():
            executor.spin_once(timeout_sec=period)
        if stop:
            why = 'terminate'
    except KeyboardInterrupt:
        why = 'interrupt'
    except ExternalShutdownException:
        pass
    finally:
        signal.signal(signal.SIGTERM, old)
        executor.remove_node(node)
        executor.shutdown(timeout_sec=period)
    return why
