"""Check that every constant in the ini fits inside the page it claims.

The ini is the contract between the firmware's config structs and TunerStudio,
and nothing in the build enforces it: a field moved between pages, or a page
whose size changed, shows up as TunerStudio quietly reading the wrong bytes.
This is the cheap half of that check - offsets against declared page sizes,
plus names declared in two places at once - and it needs no board.

    python tools/check_ini_layout.py [path/to/speeduino.ini]

It also checks the live data block, where nothing else does: a channel declared
past ochBlockSize is never sent, and two scalars sharing bytes means one gauge
shows another's value - both silent, both wrong on screen rather than missing.

Exits non-zero if anything overruns. The rest needs hardware: that the declared
page sizes and ochBlockSize match what the firmware serves, that the firmware's
own page CRC - the number TunerStudio uses to decide whether a burn took -
agrees with the bytes it just handed over, and that the ini does not promise
TunerStudio bigger chunks than the board will accept.

    python tools/check_ini_layout.py --port COM25
"""
import argparse
import os
import re
import sys
import zlib

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


def check_output_channels(ini):
    """Nothing guards the live data block the way page sizes guard the pages.

    A channel declared past ochBlockSize is simply never sent, and two scalars
    sharing bytes means one gauge shows the other's value. Both are silent: the
    number on the screen is wrong, not missing. Bits sharing a byte are how
    flags are meant to work, and the same name at the same offset twice is a
    CELSIUS/#else pair, so neither counts.
    """
    match = re.search(r'ochBlockSize\s*=\s*(\d+)', ini)
    if not match:
        return ['ochBlockSize not found']
    size = int(match.group(1))

    start = ini.index('[OutputChannels]')
    end = ini.index('[Datalog]', start)

    problems = []
    entries = []
    for line in ini[start:end].split('\n'):
        stripped = line.strip()
        if not stripped or stripped.startswith(';'):
            continue
        match = ENTRY.match(stripped)
        if not match:
            continue
        name, kind, type_name, offset = match.group(1), match.group(2), match.group(3), int(match.group(4))
        span = entry_size(kind, type_name, match.group(5) or '')
        if offset + span > size:
            problems.append('output channel %s at %d+%d is past ochBlockSize %d'
                            % (name, offset, span, size))
        if kind != 'bits':
            entries.append((offset, span, name))

    entries.sort()
    for (offset, span, name), (next_offset, next_span, next_name) in zip(entries, entries[1:]):
        if offset + span > next_offset and name != next_name:
            problems.append('output channels %s (%d+%d) and %s (%d+%d) overlap'
                            % (name, offset, span, next_name, next_offset, next_span))

    print('[OutputChannels]: %d channels in %d bytes' % (len(entries), size))
    return problems


def declared_names(ini):
    """Every name the ini defines: typed entries, expression channels, defines."""
    names = set()
    for pattern in (r'^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(?:scalar|bits|array|string)\s*,',
                    r'^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*\{',
                    r'^\s*#define\s+([A-Za-z_][A-Za-z0-9_]*)',
                    r'^\s*settingOption\s*=\s*([A-Za-z_][A-Za-z0-9_]*)'):
        names.update(match.group(1) for match in re.finditer(pattern, ini, re.M))
    return names


def ini_section(ini, name, following):
    """The text of one [Section], up to the next one."""
    begin = ini.index('[%s]' % name)
    return ini[begin:ini.index('[%s]' % following, begin)]


DATALOG_REF = re.compile(r'^\s*entry\s*=\s*([A-Za-z_][A-Za-z0-9_]*)', re.M)
GAUGE_REF = re.compile(r'^\s*[A-Za-z_][A-Za-z0-9_]*\s*=\s*([A-Za-z_][A-Za-z0-9_]*)\s*,\s*"', re.M)
FIELD_REF = re.compile(r'^\s*field\s*=\s*"[^"]*"\s*,\s*([A-Za-z_][A-Za-z0-9_]*)', re.M)


def check_references(ini):
    """A field, gauge or log entry naming something that does not exist.

    TunerStudio does not stop on one: the field is missing from the dialog, or
    the gauge reads nothing, and you find out by noticing an absence. Moving
    entries between pages - which this branch did with thirty pin fields and
    thirty-two aux ones - is exactly what leaves these behind.

    The gauge and datalog patterns are scoped to their own sections: the same
    "name, quoted label" shape means something else entirely in [Menu] and in
    the dialog definitions.
    """
    names = declared_names(ini)
    checks = (
        ('datalog entry', DATALOG_REF, ini_section(ini, 'Datalog', 'LoggerDefinition')),
        ('gauge', GAUGE_REF, ini_section(ini, 'GaugeConfigurations', 'FrontPage')),
        ('dialog field', FIELD_REF, ini),
    )

    problems = []
    total = 0
    for label, pattern, text in checks:
        refs = pattern.findall(text)
        total += len(refs)
        for name in sorted(set(refs) - names):
            problems.append('%s names %s, which the ini never declares' % (label, name))

    print('cross references: %d, %s'
          % (total, 'all resolved' if not problems else '%d dangling' % len(problems)))
    return problems


def active_setting(ini, name):
    """The last uncommented value of a setting, ignoring COMMS_COMPAT branches.

    The ini declares several of these inside #if branches. Only the compat one
    is conditional on something we are not, so take the value that a normal
    STM32 build sees.
    """
    value = None
    in_compat = False
    for line in ini.split('\n'):
        stripped = line.strip()
        if stripped.startswith('#if COMMS_COMPAT'):
            in_compat = True
            continue
        if stripped.startswith('#endif'):
            in_compat = False
            continue
        if in_compat or stripped.startswith(';'):
            continue
        match = re.match(re.escape(name) + r'\s*=\s*(\d+)', stripped)
        if match:
            value = int(match.group(1))
    return value


def check_blocking_factors(ini, ecu):
    """The ini must not promise TunerStudio bigger chunks than the board takes.

    The firmware reports its own limits through the 'f' command, and nothing
    checks the two against each other: an ini asking for more than the serial
    buffer holds fails as a corrupted transfer, not as a clear error.
    """
    reply = ecu.command(b'f')
    board_block = (reply[2] << 8) | reply[3]
    board_table = (reply[4] << 8) | reply[5]
    print('board blocking factors: %d / %d (page / table)' % (board_block, board_table))

    problems = []
    for name, reported in (('blockingFactor', board_block), ('tableBlockingFactor', board_table)):
        declared = active_setting(ini, name)
        if declared is None:
            problems.append('%s: not found in the ini' % name)
        elif declared > reported:
            problems.append('%s: ini asks for %d, board accepts %d' % (name, declared, reported))
    return problems


def check_against_board(ini, port):
    """Ask a running board for each page: right size, and a CRC that agrees.

    The CRC comparison is the one that matters. TunerStudio decides whether a
    burn took by asking the board for a page CRC, so if the firmware's own walk
    over a page ever disagrees with the bytes it serves for that page, TS is
    told the tune did not stick when it did - or worse, that it did when it
    did not.
    """
    sys.path.insert(0, HERE)
    from tsclient import Ecu, EcuError

    sizes = page_sizes(ini)
    problems = []
    with Ecu(port) as ecu:
        print('board: %s' % ecu.code_version())
        problems += check_blocking_factors(ini, ecu)

        #The live data block is the other size the two sides have to agree on.
        declared_block = int(re.search(r'ochBlockSize\s*=\s*(\d+)', ini).group(1))
        served = len(ecu.output_channels(0, declared_block))
        if served != declared_block:
            problems.append('ochBlockSize: ini says %d bytes, board served %d' % (declared_block, served))
        for number, declared in enumerate(sizes, start=1):
            try:
                data = ecu.read_page(number, 0, declared)
            except EcuError as exc:
                problems.append('page %d: %s' % (number, exc))
                continue
            if len(data) != declared:
                problems.append('page %d: ini says %d bytes, board served %d'
                                % (number, declared, len(data)))
                continue
            try:
                reported = ecu.page_crc(number)
            except EcuError as exc:
                problems.append('page %d CRC: %s' % (number, exc))
                continue
            computed = zlib.crc32(data) & 0xFFFFFFFF
            if reported != computed:
                problems.append('page %d: board reports CRC %08X, its own bytes give %08X'
                                % (number, reported, computed))
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
    problems += check_output_channels(ini)
    problems += check_references(ini)

    if args.port:
        problems += check_against_board(ini, args.port)

    for problem in problems:
        print('  ! %s' % problem)
    print('problems: %d' % len(problems))
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
