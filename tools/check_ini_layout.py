"""Check that every constant in the ini fits inside the page it claims.

The ini is the contract between the firmware's config structs and TunerStudio,
and nothing in the build enforces it: a field moved between pages, or a page
whose size changed, shows up as TunerStudio quietly reading the wrong bytes.
This is the cheap half of that check - offsets against declared page sizes,
plus names declared in two places at once - and it needs no board.

    python tools/check_ini_layout.py [path/to/speeduino.ini]

Exits non-zero if anything overruns. The other half, that the declared page
sizes match what the firmware actually serves, needs hardware:

    python tools/check_ini_layout.py --port COM25
"""
import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

TYPE_WIDTH = {'U08': 1, 'S08': 1, 'U16': 2, 'S16': 2, 'U32': 4, 'S32': 4, 'F32': 4}

ENTRY = re.compile(
    r'([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(scalar|bits|array|string)\s*,'
    r'\s*([A-Z0-9]+)\s*,\s*(\d+)\s*(?:,\s*(.*))?$')


def page_sizes(ini):
    return [int(x) for x in re.search(r'pageSize\s*=\s*([0-9,\s]+)', ini).group(1).split(',')]


def entry_size(kind, type_name, rest):
    width = TYPE_WIDTH.get(type_name, 1)
    if kind == 'array':
        dims = re.search(r'\[\s*(\d+)\s*(?:x\s*(\d+))?\s*\]', rest)
        if not dims:
            return width
        count = int(dims.group(1)) * (int(dims.group(2)) if dims.group(2) else 1)
        return width * count
    if kind == 'string':
        length = re.search(r',\s*(\d+)', rest)
        return int(length.group(1)) if length else width
    return width


def check_offsets(ini):
    """Every constant must fit in its page, and mean one place only."""
    sizes = page_sizes(ini)
    problems = []
    located = {}
    page = None
    in_constants = False

    for lineno, line in enumerate(ini.split('\n'), 1):
        stripped = line.strip()
        if stripped.startswith('['):
            in_constants = stripped.startswith('[Constants]')
            page = None
            continue
        if not in_constants or stripped.startswith(';'):
            continue
        page_decl = re.match(r'page\s*=\s*(\d+)\s*$', stripped)
        if page_decl:
            page = int(page_decl.group(1))
            continue
        if page is None:
            continue
        match = ENTRY.match(stripped)
        if not match:
            continue

        name, kind, type_name, offset = match.group(1), match.group(2), match.group(3), int(match.group(4))
        size = entry_size(kind, type_name, match.group(5) or '')
        limit = sizes[page - 1]
        if offset + size > limit:
            problems.append('line %d: %s is at %d+%d on page %d, which is only %d bytes'
                            % (lineno, name, offset, size, page, limit))
        if name in located and located[name] != (page, offset):
            problems.append('line %d: %s is also declared at page %d offset %d'
                            % (lineno, name, located[name][0], located[name][1]))
        located[name] = (page, offset)

    return len(located), len(sizes), problems


def check_against_board(ini, port):
    """Ask a running board for each page and compare with the declared size."""
    sys.path.insert(0, HERE)
    from tsclient import Ecu, EcuError

    sizes = page_sizes(ini)
    problems = []
    with Ecu(port) as ecu:
        print('board: %s' % ecu.code_version())
        for number, declared in enumerate(sizes, start=1):
            try:
                served = len(ecu.read_page(number, 0, declared))
            except EcuError as exc:
                problems.append('page %d: %s' % (number, exc))
                continue
            if served != declared:
                problems.append('page %d: ini says %d bytes, board served %d' % (number, declared, served))
    return problems


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('ini', nargs='?', default=os.path.join(REPO, 'reference', 'speeduino.ini'))
    parser.add_argument('--port', help='also check the declared page sizes against a board')
    args = parser.parse_args()

    with open(args.ini, encoding='latin-1') as handle:
        ini = handle.read()

    count, pages, problems = check_offsets(ini)
    print('%s: %d constants across %d pages' % (os.path.relpath(args.ini, REPO), count, pages))

    if args.port:
        problems += check_against_board(ini, args.port)

    for problem in problems:
        print('  ! %s' % problem)
    print('problems: %d' % len(problems))
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
