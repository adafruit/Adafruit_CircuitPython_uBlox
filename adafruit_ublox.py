# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2026 Limor Fried/Ladyada for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_ublox`
================================================================================

CircuitPython Library for interfacing with u-blox GPS Modules


* Author(s): Limor Fried/Ladyada

Implementation Notes
--------------------

**Hardware:**

* `u-blox MAX-M10S <https://www.u-blox.com/en/product/max-m10-series>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

import time
import struct

try:
    from typing import Optional, Callable
    from busio import I2C
except ImportError:
    pass

from micropython import const
from adafruit_bus_device import i2c_device

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_uBlox.git"

# I2C DDC registers
_REG_DATA_STREAM = const(0xFF)
_REG_BYTES_AVAIL_MSB = const(0xFD)

# UBX sync characters
_UBX_SYNC_1 = const(0xB5)
_UBX_SYNC_2 = const(0x62)

# UBX message classes
UBX_CLASS_NAV = const(0x01)
UBX_CLASS_RXM = const(0x02)
UBX_CLASS_INF = const(0x04)
UBX_CLASS_ACK = const(0x05)
UBX_CLASS_CFG = const(0x06)
UBX_CLASS_UPD = const(0x09)
UBX_CLASS_MON = const(0x0A)
UBX_CLASS_AID = const(0x0B)
UBX_CLASS_TIM = const(0x0D)
UBX_CLASS_ESF = const(0x10)
UBX_CLASS_MGA = const(0x13)
UBX_CLASS_LOG = const(0x21)
UBX_CLASS_SEC = const(0x27)
UBX_CLASS_HNR = const(0x28)
UBX_CLASS_NMEA = const(0xF0)

# UBX CFG message IDs
UBX_CFG_PRT = const(0x00)
UBX_CFG_MSG = const(0x01)
UBX_CFG_RST = const(0x04)
UBX_CFG_RATE = const(0x08)
UBX_CFG_CFG = const(0x09)
UBX_CFG_NAVX5 = const(0x23)
UBX_CFG_GNSS = const(0x3E)
UBX_CFG_PMS = const(0x86)

# UBX ACK message IDs
_UBX_ACK_ACK = const(0x01)
_UBX_ACK_NAK = const(0x00)

# Port IDs
UBX_PORT_DDC = const(0)
UBX_PORT_UART1 = const(1)
UBX_PORT_UART2 = const(2)
UBX_PORT_USB = const(3)
UBX_PORT_SPI = const(4)

# Protocol masks
UBX_PROTOCOL_UBX = const(0x0001)
UBX_PROTOCOL_NMEA = const(0x0002)
UBX_PROTOCOL_RTCM = const(0x0004)
UBX_PROTOCOL_RTCM3 = const(0x0020)

# Send status
UBX_SEND_OK = const(0)
UBX_SEND_NAK = const(1)
UBX_SEND_FAIL = const(2)
UBX_SEND_TIMEOUT = const(3)

# Parser states (internal)
_WAIT_SYNC_1 = const(0)
_WAIT_SYNC_2 = const(1)
_GET_CLASS = const(2)
_GET_ID = const(3)
_GET_LEN_1 = const(4)
_GET_LEN_2 = const(5)
_GET_PAYLOAD = const(6)
_GET_CK_A = const(7)
_GET_CK_B = const(8)

_MAX_PAYLOAD = const(64)
_MAX_BUFFER = const(128)

# NMEA message IDs (within class 0xF0)
NMEA_GGA = const(0x00)
NMEA_GLL = const(0x01)
NMEA_GSA = const(0x02)
NMEA_GSV = const(0x03)
NMEA_RMC = const(0x04)
NMEA_VTG = const(0x05)


class UBloxDDC:
    """I2C (DDC) transport for u-blox GPS modules.

    :param ~busio.I2C i2c_bus: The I2C bus connected to the module
    :param int address: I2C address, default 0x42
    :param float timeout: Timeout in seconds for readline, default 5.0
    """

    def __init__(
        self,
        i2c_bus: "I2C",
        *,
        address: int = 0x42,
        timeout: float = 5.0,
    ) -> None:
        self._i2c = i2c_device.I2CDevice(i2c_bus, address)
        self._timeout = timeout
        self._linebuf = []

    @property
    def in_waiting(self) -> int:
        """Number of bytes available to read from the module."""
        buf = bytearray(2)
        with self._i2c as i2c:
            i2c.write_then_readinto(bytes([_REG_BYTES_AVAIL_MSB]), buf)
        return (buf[0] << 8) | buf[1]

    def read(self, num_bytes: int = 1) -> Optional[bytearray]:
        """Read up to num_bytes from the DDC data stream.

        :param int num_bytes: Maximum number of bytes to read
        :return: Data read, or None if no data available
        :rtype: bytearray or None
        """
        avail = self.in_waiting
        if avail == 0:
            return None
        to_read = min(num_bytes, avail)
        buf = bytearray(to_read)
        with self._i2c as i2c:
            i2c.write_then_readinto(bytes([_REG_DATA_STREAM]), buf)
        return buf

    def readline(self) -> Optional[bytearray]:
        """Read a newline-terminated line from the DDC data stream.

        Buffers bytes internally until a newline (0x0A) is found or timeout.

        :return: Newline-terminated bytearray, or None if timeout
        :rtype: bytearray or None
        """
        timeout = time.monotonic() + self._timeout
        while timeout > time.monotonic():
            if self._linebuf and self._linebuf[-1] == 0x0A:
                break
            char = self.read(1)
            if not char:
                continue
            self._linebuf.append(char[0])
        if self._linebuf and self._linebuf[-1] == 0x0A:
            ret = bytearray(self._linebuf)
            self._linebuf = []
            return ret
        return None

    def write(self, buffer: bytes) -> None:
        """Write raw bytes to the module over I2C.

        :param bytes buffer: Data to write
        """
        with self._i2c as i2c:
            i2c.write(bytes(buffer))


class UBloxUBX:
    """UBX binary protocol engine for u-blox GPS modules.

    :param UBloxDDC ddc: DDC transport instance
    :param bool debug: Enable debug output, default False
    """

    def __init__(self, ddc: UBloxDDC, *, debug: bool = False) -> None:
        self._ddc = ddc
        self._debug = debug

        # Parser state
        self._state = _WAIT_SYNC_1
        self._msg_class = 0
        self._msg_id = 0
        self._payload_len = 0
        self._payload_counter = 0
        self._payload_buf = bytearray(_MAX_PAYLOAD)
        self._ck_a = 0
        self._ck_b = 0

        # Last received message (for ACK matching)
        self._last_msg_class = 0
        self._last_msg_id = 0
        self._last_payload = bytearray(8)
        self._last_payload_len = 0

        # Callback
        self._callback = None

    @property
    def callback(self) -> Optional["Callable"]:
        """Get/set callback for received UBX messages.

        Callback signature: ``callback(msg_class, msg_id, payload)``
        where payload is a bytearray.
        """
        return self._callback

    @callback.setter
    def callback(self, func: Optional["Callable"]) -> None:
        self._callback = func

    @staticmethod
    def _checksum(data: bytes) -> tuple:
        """Calculate UBX Fletcher checksum.

        :param bytes data: Data to checksum (class through end of payload)
        :return: Tuple of (ck_a, ck_b)
        :rtype: tuple
        """
        ck_a = 0
        ck_b = 0
        for byte in data:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        return ck_a, ck_b

    def send(self, msg_class: int, msg_id: int, payload: bytes = b"") -> bool:
        """Send a UBX message.

        Builds the complete frame with sync chars, header, payload, and checksum.

        :param int msg_class: UBX message class
        :param int msg_id: UBX message ID
        :param bytes payload: Message payload, default empty
        :return: True if message was sent
        :rtype: bool
        """
        length = len(payload)
        # Build message: sync1 + sync2 + class + id + len_lo + len_hi + payload + ck_a + ck_b
        msg = bytearray(length + 8)
        msg[0] = _UBX_SYNC_1
        msg[1] = _UBX_SYNC_2
        msg[2] = msg_class
        msg[3] = msg_id
        msg[4] = length & 0xFF
        msg[5] = (length >> 8) & 0xFF
        if payload:
            msg[6 : 6 + length] = payload

        # Checksum over class + id + length + payload
        ck_a, ck_b = self._checksum(msg[2 : 6 + length])
        msg[6 + length] = ck_a
        msg[7 + length] = ck_b

        if self._debug:
            print(
                "UBX TX: class=0x{:02X} id=0x{:02X} len={}".format(
                    msg_class, msg_id, length
                )
            )

        self._ddc.write(msg)
        return True

    def send_with_ack(
        self,
        msg_class: int,
        msg_id: int,
        payload: bytes = b"",
        timeout: float = 0.5,
    ) -> int:
        """Send a UBX message and wait for ACK/NAK.

        :param int msg_class: UBX message class
        :param int msg_id: UBX message ID
        :param bytes payload: Message payload
        :param float timeout: Timeout in seconds, default 0.5
        :return: UBX_SEND_OK, UBX_SEND_NAK, UBX_SEND_FAIL, or UBX_SEND_TIMEOUT
        :rtype: int
        """
        if not self.send(msg_class, msg_id, payload):
            return UBX_SEND_FAIL

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.check_messages():
                if self._last_msg_class == UBX_CLASS_ACK:
                    if (
                        self._last_payload_len >= 2
                        and self._last_payload[0] == msg_class
                        and self._last_payload[1] == msg_id
                    ):
                        if self._last_msg_id == _UBX_ACK_ACK:
                            if self._debug:
                                print(
                                    "UBX ACK: OK for 0x{:02X} 0x{:02X}".format(
                                        msg_class, msg_id
                                    )
                                )
                            return UBX_SEND_OK
                        if self._last_msg_id == _UBX_ACK_NAK:
                            if self._debug:
                                print(
                                    "UBX ACK: NAK for 0x{:02X} 0x{:02X}".format(
                                        msg_class, msg_id
                                    )
                                )
                            return UBX_SEND_NAK

        if self._debug:
            print(
                "UBX ACK: TIMEOUT for 0x{:02X} 0x{:02X}".format(msg_class, msg_id)
            )
        return UBX_SEND_TIMEOUT

    def check_messages(self) -> bool:
        """Process available bytes through the UBX parser.

        Reads bytes from DDC and feeds them through the state machine.
        Triggers the callback when a complete valid message is received.

        :return: True if a complete message was received
        :rtype: bool
        """
        message_received = False
        avail = self._ddc.in_waiting
        if avail == 0:
            return False

        data = self._ddc.read(min(avail, _MAX_BUFFER))
        if data is None:
            return False

        for byte in data:
            if self._parse_byte(byte):
                message_received = True

        return message_received

    def _parse_byte(self, byte: int) -> bool:
        """Feed one byte into the UBX parser state machine.

        :param int byte: Byte to parse
        :return: True if a complete valid message was received
        :rtype: bool
        """
        if self._state == _WAIT_SYNC_1:
            if byte == _UBX_SYNC_1:
                self._state = _WAIT_SYNC_2

        elif self._state == _WAIT_SYNC_2:
            if byte == _UBX_SYNC_2:
                self._state = _GET_CLASS
            else:
                self._state = _WAIT_SYNC_1

        elif self._state == _GET_CLASS:
            self._msg_class = byte
            self._ck_a = byte
            self._ck_b = byte
            self._state = _GET_ID

        elif self._state == _GET_ID:
            self._msg_id = byte
            self._ck_a = (self._ck_a + byte) & 0xFF
            self._ck_b = (self._ck_b + self._ck_a) & 0xFF
            self._state = _GET_LEN_1

        elif self._state == _GET_LEN_1:
            self._payload_len = byte
            self._ck_a = (self._ck_a + byte) & 0xFF
            self._ck_b = (self._ck_b + self._ck_a) & 0xFF
            self._state = _GET_LEN_2

        elif self._state == _GET_LEN_2:
            self._payload_len |= byte << 8
            self._ck_a = (self._ck_a + byte) & 0xFF
            self._ck_b = (self._ck_b + self._ck_a) & 0xFF
            if self._payload_len > _MAX_PAYLOAD:
                self._state = _WAIT_SYNC_1
            else:
                self._payload_counter = 0
                if self._payload_len == 0:
                    self._state = _GET_CK_A
                else:
                    self._state = _GET_PAYLOAD

        elif self._state == _GET_PAYLOAD:
            self._payload_buf[self._payload_counter] = byte
            self._ck_a = (self._ck_a + byte) & 0xFF
            self._ck_b = (self._ck_b + self._ck_a) & 0xFF
            self._payload_counter += 1
            if self._payload_counter == self._payload_len:
                self._state = _GET_CK_A

        elif self._state == _GET_CK_A:
            if byte == self._ck_a:
                self._state = _GET_CK_B
            else:
                self._state = _WAIT_SYNC_1

        elif self._state == _GET_CK_B:
            self._state = _WAIT_SYNC_1
            if byte == self._ck_b:
                # Valid message received
                self._last_msg_class = self._msg_class
                self._last_msg_id = self._msg_id
                self._last_payload_len = self._payload_len

                # Store small payloads for ACK matching
                if self._payload_len <= len(self._last_payload):
                    self._last_payload[: self._payload_len] = self._payload_buf[
                        : self._payload_len
                    ]

                if self._debug:
                    print(
                        "UBX RX: class=0x{:02X} id=0x{:02X} len={}".format(
                            self._msg_class, self._msg_id, self._payload_len
                        )
                    )

                if self._callback is not None:
                    self._callback(
                        self._msg_class,
                        self._msg_id,
                        bytes(self._payload_buf[: self._payload_len]),
                    )
                return True

        return False

    # --- High-level configuration helpers ---

    def set_ubx_only(
        self, port_id: int = UBX_PORT_DDC, timeout: float = 0.5
    ) -> int:
        """Configure a port to use UBX protocol only (disable NMEA).

        :param int port_id: Port to configure, default UBX_PORT_DDC
        :param float timeout: ACK timeout in seconds
        :return: UBX_SEND_OK, UBX_SEND_NAK, UBX_SEND_FAIL, or UBX_SEND_TIMEOUT
        :rtype: int
        """
        # Build 20-byte CFG-PRT payload
        payload = bytearray(20)
        payload[0] = port_id

        if port_id == UBX_PORT_DDC:
            # Mode field: I2C address in bits 7:1
            struct.pack_into("<I", payload, 4, 0x42 << 1)
        elif port_id in (UBX_PORT_UART1, UBX_PORT_UART2):
            # 8N1 mode
            struct.pack_into("<I", payload, 4, 0x000)

        # inProtoMask and outProtoMask = UBX only
        struct.pack_into("<H", payload, 12, UBX_PROTOCOL_UBX)
        struct.pack_into("<H", payload, 14, UBX_PROTOCOL_UBX)

        return self.send_with_ack(UBX_CLASS_CFG, UBX_CFG_PRT, payload, timeout)

    def set_nmea_output(
        self,
        *,
        gga: bool = True,
        gll: bool = False,
        gsa: bool = False,
        gsv: bool = False,
        rmc: bool = True,
        vtg: bool = False,
        timeout: float = 0.5,
    ) -> int:
        """Enable/disable individual NMEA sentences on the DDC port.

        :param bool gga: Enable GGA sentence
        :param bool gll: Enable GLL sentence
        :param bool gsa: Enable GSA sentence
        :param bool gsv: Enable GSV sentence
        :param bool rmc: Enable RMC sentence
        :param bool vtg: Enable VTG sentence
        :param float timeout: ACK timeout in seconds per message
        :return: UBX_SEND_OK if all succeeded, or first failure status
        :rtype: int
        """
        sentences = (
            (NMEA_GGA, gga),
            (NMEA_GLL, gll),
            (NMEA_GSA, gsa),
            (NMEA_GSV, gsv),
            (NMEA_RMC, rmc),
            (NMEA_VTG, vtg),
        )
        for nmea_id, enabled in sentences:
            # CFG-MSG payload: class, id, rate_DDC, rate_UART1, rate_UART2, rate_USB, rate_SPI, reserved
            rate = 0x01 if enabled else 0x00
            payload = bytes(
                [UBX_CLASS_NMEA, nmea_id, rate, 0x00, 0x00, 0x00, 0x00, 0x00]
            )
            status = self.send_with_ack(
                UBX_CLASS_CFG, UBX_CFG_MSG, payload, timeout
            )
            if status != UBX_SEND_OK:
                return status
        return UBX_SEND_OK

    def set_update_rate(self, rate_hz: int = 1, timeout: float = 0.5) -> int:
        """Set navigation measurement/update rate.

        :param int rate_hz: Update rate in Hz (1, 2, 5, or 10)
        :param float timeout: ACK timeout in seconds
        :return: UBX_SEND_OK, UBX_SEND_NAK, UBX_SEND_FAIL, or UBX_SEND_TIMEOUT
        :rtype: int
        """
        rates = {
            1: 1000,
            2: 500,
            5: 200,
            10: 100,
        }
        meas_rate = rates.get(rate_hz)
        if meas_rate is None:
            raise ValueError("rate_hz must be 1, 2, 5, or 10")

        # CFG-RATE payload: measRate(2), navRate(2), timeRef(2)
        payload = struct.pack("<HHH", meas_rate, 1, 1)
        return self.send_with_ack(UBX_CLASS_CFG, UBX_CFG_RATE, payload, timeout)


class GPS_UBloxI2C:
    """GPS NMEA parsing for u-blox modules over I2C (DDC).

    Subclasses ``adafruit_gps.GPS`` to provide NMEA parsing, using a
    :class:`UBloxDDC` instance for I2C transport.

    :param UBloxDDC ddc: DDC transport instance
    :param bool debug: Enable GPS debug output, default False
    """

    def __new__(cls, ddc: UBloxDDC, *, debug: bool = False):
        # Import GPS here to avoid hard dependency at module level
        from adafruit_gps import GPS  # noqa: PLC0415

        # Dynamically create a subclass of GPS
        class _GPS_UBloxI2C(GPS):
            def __init__(self, ddc, *, debug=False):
                super().__init__(None, debug)
                self._ddc = ddc

            def read(self, num_bytes=1):
                """Read from DDC."""
                return self._ddc.read(num_bytes)

            def write(self, bytestr):
                """Write to DDC."""
                return self._ddc.write(bytestr)

            @property
            def in_waiting(self):
                """Bytes available from DDC."""
                return self._ddc.in_waiting

            def readline(self):
                """Read a newline-terminated line from DDC."""
                return self._ddc.readline()

        return _GPS_UBloxI2C(ddc, debug=debug)
