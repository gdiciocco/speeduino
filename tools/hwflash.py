"""Flash a PlatformIO-built Speeduino image to a board over USB DFU.

There is no debug probe on the bench board, so the only way in is the STM32
system bootloader. The running firmware can be asked to jump there with the
TunerStudio command button 12801 (TS_CMD_STM32_BOOTLOADER); once it does the
USB CDC port disappears and a 0483:DF11 DFU device shows up in its place.

    python tools/hwflash.py --env caponord_black_f407zg --port COM25

Flags:
    --list-only     enter DFU, print the descriptors, leave the board there
    --leave         just tell a board already in DFU to run its application
    --no-build      flash whatever is already in .pio/build/<env>
"""
import argparse
import glob
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

PIO = os.path.expandvars(r'%USERPROFILE%\.platformio\penv\Scripts\pio.exe')
DFU_VIDPID = '0483:df11'


def dfu_util():
    hits = glob.glob(os.path.expandvars(r'%USERPROFILE%\.platformio\packages\tool-dfuutil*\bin\dfu-util.exe'))
    if not hits:
        raise SystemExit('dfu-util not found under .platformio/packages')
    return hits[0]


def dfu_list():
    out = subprocess.run([dfu_util(), '-l'], capture_output=True, text=True).stdout
    return [l.strip() for l in out.splitlines() if 'Found DFU' in l]


def wait_for_dfu(timeout=15.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        found = dfu_list()
        if found:
            return found
        time.sleep(0.5)
    return []


def wait_for_serial(port, timeout=25.0):
    import serial.tools.list_ports
    deadline = time.time() + timeout
    while time.time() < deadline:
        if port.upper() in [p.device.upper() for p in serial.tools.list_ports.comports()]:
            return True
        time.sleep(0.5)
    return False


def enter_dfu(port):
    from tsclient import Ecu
    print('[dfu] asking %s to jump to the bootloader' % port)
    try:
        with Ecu(port) as ecu:
            print('[dfu] running firmware: %s' % ecu.code_version())
            ecu.jump_to_bootloader()
    except Exception as exc:                                  # noqa: BLE001
        print('[dfu] could not talk to %s (%s) - is it already in DFU?' % (port, exc))
    found = wait_for_dfu()
    if not found:
        raise SystemExit('[dfu] no DFU device appeared. Power-cycle the board and retry.')
    for line in found:
        print('[dfu] %s' % line)
    return found


def build(env):
    print('[build] pio run -e %s' % env)
    r = subprocess.run([PIO, 'run', '-e', env], cwd=REPO)
    if r.returncode:
        raise SystemExit('[build] failed')


def firmware_path(env):
    for name in ('firmware.bin', 'firmware.hex'):
        p = os.path.join(REPO, '.pio', 'build', env, name)
        if os.path.exists(p):
            return p
    raise SystemExit('[flash] no firmware image in .pio/build/%s' % env)


def flash(env):
    image = firmware_path(env)
    size = os.path.getsize(image)
    print('[flash] %s (%d bytes)' % (os.path.relpath(image, REPO), size))
    cmd = [dfu_util(), '-d', DFU_VIDPID, '-a', '0', '-s', '0x08000000:leave', '-D', image]
    r = subprocess.run(cmd, cwd=REPO)
    if r.returncode:
        raise SystemExit('[flash] dfu-util failed')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--env', default='caponord_black_f407zg')
    ap.add_argument('--port', default='COM25')
    ap.add_argument('--list-only', action='store_true')
    ap.add_argument('--leave', action='store_true')
    ap.add_argument('--no-build', action='store_true')
    args = ap.parse_args()

    if args.leave:
        subprocess.run([dfu_util(), '-d', DFU_VIDPID, '-a', '0', '-s', '0x08000000:leave'], cwd=REPO)
        return

    if args.list_only:
        enter_dfu(args.port)
        subprocess.run([dfu_util(), '-d', DFU_VIDPID, '-l'], cwd=REPO)
        return

    if not args.no_build:
        build(args.env)
    enter_dfu(args.port)
    flash(args.env)

    print('[flash] waiting for %s to come back' % args.port)
    if wait_for_serial(args.port):
        print('[flash] %s is back' % args.port)
    else:
        print('[flash] %s did not reappear - check the board' % args.port)


if __name__ == '__main__':
    main()
