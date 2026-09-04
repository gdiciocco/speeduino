"""PlatformIO custom uploader: put a running Speeduino into DFU, then flash it.

The bench board has no debug probe, so the only way in is the STM32 system
bootloader. The running firmware jumps there on TunerStudio command button
12801; this script sends that, waits for 0483:DF11 to enumerate, and hands the
image to dfu-util with ":leave" so the board runs the new build immediately.

It is tolerant of a board that is already sitting in DFU (a previous run that
was interrupted), so re-running is always safe.

    upload_protocol = custom
    upload_command = python $PROJECT_DIR/tools/dfu_upload.py --port COM25 $SOURCE
"""
import argparse
import glob
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

DFU_VIDPID = '0483:df11'
PS_PRESENT = ('Get-PnpDevice -PresentOnly | Where-Object { $_.InstanceId -match "%s" } | '
              'ForEach-Object { $_.InstanceId }')


def dfu_util():
    hits = glob.glob(os.path.expandvars(
        r'%USERPROFILE%\.platformio\packages\tool-dfuutil*\bin\dfu-util.exe'))
    if not hits:
        sys.exit('dfu-util not found under .platformio/packages')
    return hits[0]


def _pnp(pattern):
    out = subprocess.run(['powershell', '-NoProfile', '-Command', PS_PRESENT % pattern],
                         capture_output=True, text=True).stdout.strip()
    return [l.strip() for l in out.splitlines() if l.strip()]


def in_dfu():
    return bool(_pnp('VID_0483&PID_DF11'))


def wait(predicate, timeout, what):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.4)
    print('[dfu] timed out waiting for %s' % what)
    return False


def _jump_via_firmware(port):
    """The ECU firmware answers TunerStudio command button 12801."""
    from tsclient import Ecu
    with Ecu(port, settle=1.5) as ecu:
        print('[dfu] %s is running %s' % (port, ecu.code_version()))
        ecu.jump_to_bootloader()


def _jump_via_test_harness(port):
    """A test image is not the ECU firmware and does not speak the TS protocol.

    test/test_harness_device.h watches the test port for this escape sequence
    precisely so that flashing a test build is not a one-way door.
    """
    import serial
    # Opening the port is what releases the harness's "wait for the monitor"
    # loop, so the board then runs its whole suite and - because UNITY_END()
    # calls Serial.end() - drops off the bus and comes back. Stream the escape
    # for long enough to cover the run and the re-enumeration afterwards.
    deadline = time.time() + 40.0
    sent = 0
    while time.time() < deadline and not in_dfu():
        try:
            with serial.Serial(port, 115200, timeout=0.2) as ser:
                while time.time() < deadline and not in_dfu():
                    ser.write(b'@BOOTLOADER')
                    ser.flush()
                    sent += 1
                    ser.read(256)          # drain whatever the suite prints
                    time.sleep(0.2)
        except Exception:                                       # noqa: BLE001
            # The port vanishes each time the harness tears the CDC down.
            time.sleep(0.4)
    print('[dfu] streamed the test-harness escape sequence to %s (%d writes)' % (port, sent))


def enter_dfu(port, attempts=2):
    if in_dfu():
        print('[dfu] board is already in DFU')
        return True

    for attempt in range(1, attempts + 1):
        for name, jump in (('firmware', _jump_via_firmware),
                           ('test harness', _jump_via_test_harness)):
            try:
                jump(port)
            except Exception as exc:                            # noqa: BLE001
                print('[dfu] attempt %d, %s path: %s' % (attempt, name, exc))
                continue
            if wait(in_dfu, 10, 'the DFU device'):
                print('[dfu] board is in DFU (via the %s path)' % name)
                return True
    return False


def image_for(path):
    """PlatformIO may hand us the .elf; dfu-util wants the raw binary."""
    if path.lower().endswith('.bin'):
        return path
    candidate = os.path.splitext(path)[0] + '.bin'
    if os.path.exists(candidate):
        return candidate
    sibling = os.path.join(os.path.dirname(path), 'firmware.bin')
    if os.path.exists(sibling):
        return sibling
    sys.exit('[dfu] no .bin next to %s' % path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default=os.environ.get('SPEEDUINO_HW_PORT', 'COM25'))
    ap.add_argument('image')
    args = ap.parse_args()

    image = image_for(args.image)
    print('[dfu] image: %s (%d bytes)' % (image, os.path.getsize(image)))

    if not enter_dfu(args.port):
        sys.exit('[dfu] board never entered DFU - power-cycle it, or hold BOOT0 and reset')

    cmd = [dfu_util(), '-d', DFU_VIDPID, '-a', '0', '-s', '0x08000000:leave', '-D', image]
    if subprocess.run(cmd).returncode:
        sys.exit('[dfu] dfu-util failed')

    # PlatformIO opens the test port straight after upload; give the CDC time
    # to enumerate or the first read races the device.
    import serial.tools.list_ports

    def port_back():
        return args.port.upper() in [p.device.upper() for p in serial.tools.list_ports.comports()]

    if wait(port_back, 25, '%s to come back' % args.port):
        print('[dfu] %s is back' % args.port)
        time.sleep(1.5)


if __name__ == '__main__':
    main()
