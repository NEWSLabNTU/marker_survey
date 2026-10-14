#!/usr/bin/env python3
"""Keep the repository neutral: no banned words in tracked files or commit messages,
no e-mail addresses other than the allowed placeholders.

    python3 ci/check_words.py [--commits]

Words are compared as SHA-256 hashes of lower-case tokens (ci/banned_words.sha256),
so the list names nobody. Tokens are runs of letters and digits, also split at
case changes (FooBar -> foo, bar) and between letters and digits (abc12 -> abc).
To add a word: python3 ci/check_words.py --hash WORD, and append the line.
"""
import argparse
import hashlib
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ALLOWED_EMAILS = {'noreply@example.com', 'noreply@anthropic.com'}
EMAIL = re.compile(r'[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}')
SKIP_SUFFIXES = ('.png', '.jpg', '.pdf', '.npz', '.db3', '.zip', '.gz')


def digest(word: str) -> str:
    return hashlib.sha256(word.lower().encode()).hexdigest()


def banned():
    with open(os.path.join(HERE, 'banned_words.sha256')) as f:
        return {ln.strip() for ln in f if ln.strip() and not ln.startswith('#')}


def tokens(text: str):
    for run in re.findall(r'[A-Za-z0-9]+', text):
        yield run
        parts = re.findall(r'[A-Z]+(?=[A-Z][a-z])|[A-Z]?[a-z]+|[A-Z]+|[0-9]+', run)
        if len(parts) > 1:
            yield from parts
        letters = re.findall(r'[A-Za-z]+', run)
        if len(letters) > 1 or (letters and letters[0] != run):
            yield from letters


def check_text(text: str, where: str, bad: set, emails: bool):
    problems = []
    for n, line in enumerate(text.splitlines(), start=1):
        for t in set(tokens(line)):
            if digest(t) in bad:
                problems.append(f'{where}:{n}: banned word {t!r}')
        if emails:
            for e in EMAIL.findall(line):
                if e.lower() not in ALLOWED_EMAILS:
                    problems.append(f'{where}:{n}: e-mail address {e!r}')
    return problems


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--commits', action='store_true', help='also check every commit message')
    ap.add_argument('--hash', metavar='WORD', help='print the hash line for a new word')
    args = ap.parse_args(argv)
    if args.hash:
        print(digest(args.hash))
        return 0
    bad = banned()
    root = subprocess.check_output(['git', 'rev-parse', '--show-toplevel'], text=True).strip()
    files = subprocess.check_output(['git', 'ls-files'], cwd=root, text=True).split('\n')
    problems = []
    for rel in files:
        if not rel or rel.endswith(SKIP_SUFFIXES) or rel == 'ci/banned_words.sha256':
            continue
        path = os.path.join(root, rel)
        if not os.path.isfile(path):
            continue
        try:
            with open(path, encoding='utf-8') as f:
                text = f.read()
        except UnicodeDecodeError:
            continue
        problems += check_text(rel, f'{rel} (file name)', bad, False)
        problems += check_text(text, rel, bad, rel != 'ci/check_words.py')
    if args.commits:
        log = subprocess.check_output(['git', 'log', '--format=%H%n%B%x00'], cwd=root, text=True)
        for entry in log.split('\x00'):
            entry = entry.strip()
            if not entry:
                continue
            sha, _, body = entry.partition('\n')
            problems += check_text(body, f'commit {sha[:10]}', bad, False)
    for p in problems:
        print(p)
    print(f'{len(problems)} problem(s)' if problems else 'neutral wording: OK')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
