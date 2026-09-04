"""Speeduino serial protocol client (new "002" framing).

Enough of the TunerStudio protocol to drive a board over USB CDC from a test
script: read and write config pages, burn them, read output channels, fetch
page CRCs and press command buttons.

Wire format, from speeduino/comms.cpp:

    host -> ecu :  <length:uint16 BE> <payload...> <crc32:uint32 BE>
    ecu  -> host:  <length:uint16 BE> <payload...> <crc32:uint32 BE>

The CRC is a plain CRC-32 (Ethernet/zlib) over the payload only. Note that the
*payload* fields are little-endian - Arduino's word(hi, lo) is called with the
high byte second - except in the 'E' command, which is the other way round.

Usage:
    from tsclient import Ecu
    with Ecu('COM25') as ecu:
        print(ecu.code_version())
        page = ecu.read_page(2, 0, 128)
"""
import struct
import time
import zlib

import serial

SERIAL_RC_OK = 0x00
SERIAL_RC_REALTIME = 0x01
SERIAL_RC_PAGE = 0x02
SERIAL_RC_BURN_OK = 0x04
SERIAL_RC_TIMEOUT = 0x80
SERIAL_RC_CRC_ERR = 0x82
SERIAL_RC_UKWN_ERR = 0x83
SERIAL_RC_RANGE_ERR = 0x84
SERIAL_RC_BUSY_ERR = 0x85

RC_NAMES = {
    0x00: 'OK', 0x01: 'REALTIME', 0x02: 'PAGE', 0x03: 'CONFIG', 0x04: 'BURN_OK',
    0x80: 'TIMEOUT', 0x81: 'RANGE', 0x82: 'CRC_ERR', 0x83: 'UKWN_ERR',
    0x84: 'RANGE_ERR', 0x85: 'BUSY_ERR',
}

SEND_OUTPUT_CHANNELS = 0x30
TS_CMD_STM32_REBOOT = 12800
TS_CMD_STM32_BOOTLOADER = 12801


class EcuError(RuntimeError):
    pass


class Ecu(object):
    def __init__(self, port, baud=115200, timeout=2.0, can_id=0, settle=2.0):
        self.port = port
        self.can_id = can_id
        self._ser = serial.Serial(port, baud, timeout=timeout)
        # A USB CDC endpoint needs a moment after open, and Windows sends a
        # 0xF0 DTR byte the firmware explicitly discards.
        time.sleep(settle)
        self._ser.reset_input_buffer()

    # ---------------------------------------------------------------- framing
    def _send(self, payload):
        frame = struct.pack('>H', len(payload)) + payload + struct.pack('>I', zlib.crc32(payload))
        self._ser.reset_input_buffer()
        self._ser.write(frame)
        self._ser.flush()

    def _read_exactly(self, n, what):
        buf = b''
        while len(buf) < n:
            chunk = self._ser.read(n - len(buf))
            if not chunk:
                raise EcuError('timed out reading %s (%d of %d bytes)' % (what, len(buf), n))
            buf += chunk
        return buf

    def _recv(self):
        length = struct.unpack('>H', self._read_exactly(2, 'length'))[0]
        payload = self._read_exactly(length, 'payload')
        crc = struct.unpack('>I', self._read_exactly(4, 'crc'))[0]
        if crc != zlib.crc32(payload):
            raise EcuError('CRC mismatch: got %08X, computed %08X' % (crc, zlib.crc32(payload)))
        return payload

    def command(self, payload, expect_rc=SERIAL_RC_OK):
        self._send(payload)
        reply = self._recv()
        if expect_rc is not None:
            if not reply:
                raise EcuError('empty reply to %r' % payload[:1])
            if reply[0] != expect_rc:
                raise EcuError('%r returned %s (0x%02X), expected %s' % (
                    payload[:1], RC_NAMES.get(reply[0], '?'), reply[0],
                    RC_NAMES.get(expect_rc, expect_rc)))
        return reply

    # ---------------------------------------------------------------- identity
    def test_comms(self):
        """'C' - what TunerStudio uses to find a board on a port."""
        return self.command(b'C')

    def code_version(self):
        """'Q' - the firmware's own version string."""
        return self.command(b'Q')[1:].decode('ascii', 'replace')

    def product_string(self):
        """'S'."""
        return self.command(b'S')[1:].decode('ascii', 'replace')

    def serial_version(self):
        """'F' is answered by the legacy handler, outside the framing."""
        self._ser.reset_input_buffer()
        self._ser.write(b'F')
        self._ser.flush()
        time.sleep(0.3)
        return self._ser.read(8).decode('ascii', 'replace')

    # ------------------------------------------------------------------- pages
    def read_page(self, page, offset, length):
        payload = struct.pack('<BBBHH', ord('p'), self.can_id, page, offset, length)
        reply = self.command(payload, SERIAL_RC_OK)
        data = reply[1:]
        if len(data) != length:
            raise EcuError('page %d: asked for %d bytes, got %d' % (page, length, len(data)))
        return data

    def write_page(self, page, offset, data):
        payload = struct.pack('<BBBHH', ord('M'), self.can_id, page, offset, len(data)) + bytes(data)
        self.command(payload, SERIAL_RC_OK)

    def burn_page(self, page):
        payload = struct.pack('<BBB', ord('b'), self.can_id, page)
        self.command(payload, SERIAL_RC_BURN_OK)

    def page_crc(self, page):
        payload = struct.pack('<BBB', ord('d'), self.can_id, page)
        reply = self.command(payload, SERIAL_RC_OK)
        return struct.unpack('>I', reply[1:5])[0]

    # ---------------------------------------------------------- output channels
    def output_channels(self, offset=0, length=128):
        payload = struct.pack('<BBBHH', ord('r'), self.can_id, SEND_OUTPUT_CHANNELS, offset, length)
        reply = self.command(payload, SERIAL_RC_OK)
        return reply[1:]

    # -------------------------------------------------------------- command btn
    def press_button(self, cmd, expect_reply=True):
        """'E' - note this one packs its argument big-endian."""
        payload = bytes([ord('E'), (cmd >> 8) & 0xFF, cmd & 0xFF])
        if not expect_reply:
            self._send(payload)
            return None
        return self.command(payload, SERIAL_RC_OK)

    def jump_to_bootloader(self):
        """The board vanishes off the USB bus, so there is no reply to wait for."""
        self.press_button(TS_CMD_STM32_BOOTLOADER, expect_reply=False)

    def reboot(self):
        self.press_button(TS_CMD_STM32_REBOOT, expect_reply=False)

    # ------------------------------------------------------------------- ctxmgr
    def close(self):
        try:
            self._ser.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False
