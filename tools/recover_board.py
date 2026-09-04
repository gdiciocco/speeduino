"""Wait for a board in DFU mode, flash the ECU firmware, verify it answers.

For when a board has ended up with an image that has no way back - an old test
binary, a half-written flash - and has to be put into the system bootloader by
hand (hold BOOT0, tap RESET). Run this first, then do that: it picks the board
up the moment it enumerates and restores it without further steps.

    python tools/recover_board.py --env caponord_black_f407zg --port COM25
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

DFU_VIDPID = '0483:df11'
PS = ('Get-PnpDevice -PresentOnly | Where-Object { $_.InstanceId -match "VID_0483&PID_DF11" } | '
      'ForEach-Object { $_.InstanceId }')


def dfu_util():
    hits = glob.glob(os.path.expandvars(
        r'%USERPROFILE%\.platformio\packages\tool-dfuutil*\bin\dfu-util.exe'))
    if not hits:
        sys.exit('dfu-util not found')
    return hits[0]


def in_dfu():
    out = subprocess.run(['powershell', '-NoProfile', '-Command', PS],
                         capture_output=True, text=True).stdout.strip()
    return bool(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--env', default='caponord_black_f407zg')
    ap.add_argument('--port', default='COM25')
    ap.add_argument('--timeout', type=float, default=600.0)
    args = ap.parse_args()

    image = os.path.join(REPO, '.pio', 'build', args.env, 'firmware.bin')
    if not os.path.exists(image):
        print('[recover] building %s' % args.env)
        if subprocess.run([os.path.expandvars(r'%USERPROFILE%\.platformio\penv\Scripts\pio.exe'),
                           'run', '-e', args.env], cwd=REPO).returncode:
            sys.exit('[recover] build failed')

    print('[recover] hold BOOT0 and tap RESET on the board - waiting up to %.0f s' % args.timeout)
    t0 = time.time()
    while time.time() - t0 < args.timeout:
        if in_dfu():
            break
        time.sleep(1.0)
    else:
        sys.exit('[recover] no DFU device appeared')

    print('[recover] DFU device found after %.0f s' % (time.time() - t0))
    time.sleep(1.0)
    cmd = [dfu_util(), '-d', DFU_VIDPID, '-a', '0', '-s', '0x08000000:leave', '-D', image]
    if subprocess.run(cmd, cwd=REPO).returncode:
        sys.exit('[recover] dfu-util failed')

    print('[recover] flashed %s, waiting for it to answer on %s' % (args.env, args.port))
    time.sleep(6.0)
    try:
        from tsclient import Ecu
        with Ecu(args.port) as ecu:
            print('[recover] board is running: %s' % ecu.code_version())
    except Exception as exc:                                    # noqa: BLE001
        print('[recover] flashed, but %s did not answer: %s' % (args.port, exc))
        sys.exit(1)
    print('[recover] done')


if __name__ == '__main__':
    main()
