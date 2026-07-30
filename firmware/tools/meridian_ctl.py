#!/usr/bin/env python3
"""meridian_ctl - reference client for the Meridian Console Protocol (MCP).

This is the host side of the binary channel the STS1000 "Meridian" exposes on
its second USB CDC-ACM endpoint: configuration, telemetry, logs and serial
firmware upgrade. The wire format is fixed by ``firmware/ARCHITECTURE.md`` §7
and spelled out byte-by-byte in ``firmware/app/src/core/mcp/mcp_wire.h``; this
file mirrors it and nothing else.

Dependencies: Python 3.8+ and pyserial. Nothing else, deliberately - this tool
has to run on a laptop in a rack room.

    meridian_ctl.py --port /dev/ttyACM1 info
    meridian_ctl.py status timing
    meridian_ctl.py watch --rate 4 --groups timing,gnss
    meridian_ctl.py cfg-list
    meridian_ctl.py cfg-get tim.tau.s
    meridian_ctl.py --password s3cret cfg-set tim.tau.s 300
    meridian_ctl.py cfg-export backup.mcf
    meridian_ctl.py phase-export run1.phr   # then: meridian_phase run1.phr
    meridian_ctl.py phase-export run.phr --repeat 12   # ~45 min, tau to 512 s
                                            # then: meridian_phase run.*.phr
    meridian_ctl.py cfg-import backup.mcf
    meridian_ctl.py log-tail --follow
    meridian_ctl.py --password s3cret fw-upload zephyr.signed.bin
    meridian_ctl.py fw-confirm

Sessions are per-invocation. The tool opens one on connect (from
``--password`` / ``$MERIDIAN_PASSWORD``, or a prompt for a mutating command on a
terminal) and the firmware drops it when the port closes. So a mutating flow
must run in ONE invocation: ``cfg-set`` then a separate ``cfg-commit`` process
would commit nothing, because closing the port after ``cfg-set`` reverts the
staged change. Chain such work in a single process (e.g. an interactive
session, or ``cfg-import`` which stages and commits atomically), not with
``auth && cfg-set && cfg-commit`` across three processes.

Everything above the ``Link`` class is pure: the codec, the payload builders
and the payload decoders take and return bytes, so they can be exercised
without a serial port (``python3 -m doctest meridian_ctl.py`` covers the codec,
and the firmware's own test_mcp.c pins the same vectors).
"""

from __future__ import annotations

import argparse
import binascii
import getpass
import hashlib
import json
import os
import struct
import sys
import time
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

# --------------------------------------------------------------------------
# Protocol constants (mcp_wire.h)
# --------------------------------------------------------------------------

MCP_VER = 1
HDR_LEN = 8
CRC_LEN = 4
MAX_PAYLOAD = 1040
MAX_FRAME = HDR_LEN + MAX_PAYLOAD + CRC_LEN

T_REQ, T_RSP, T_EVT = 0, 1, 2

CMD = {
    "HELLO": 0x01,
    "REBOOT": 0x02,
    "AUTH": 0x08,
    "CFG_LIST": 0x10,
    "CFG_GET": 0x11,
    "CFG_SET": 0x12,
    "CFG_COMMIT": 0x13,
    "CFG_REVERT": 0x14,
    "CFG_EXPORT": 0x15,
    "PHASE_EXPORT": 0x24,
    "CFG_IMPORT": 0x16,
    "FACTORY_RESET": 0x17,
    "STATUS_GET": 0x20,
    "TELEM_SUB": 0x21,
    "TELEM_UNSUB": 0x22,
    "LOG_TAIL": 0x30,
    "LOG_LEVEL": 0x31,
    "FW_INFO": 0x40,
    "FW_BEGIN": 0x41,
    "FW_DATA": 0x42,
    "FW_END": 0x43,
    "FW_CONFIRM": 0x44,
    "FW_REVERT": 0x45,
    "DIAG": 0x50,
}
CMD_NAME = {v: k for k, v in CMD.items()}

ERR = {
    0: "OK",
    1: "ERR_AUTH",
    2: "ERR_ARG",
    3: "ERR_STATE",
    4: "ERR_CRC",
    5: "ERR_OFFSET",
    6: "ERR_VERIFY",
    7: "ERR_NOSPC",
    8: "ERR_NOTSUP",
    9: "ERR_INTERNAL",
    10: "ERR_BUSY",
}

CAP_BITS = [
    (0x01, "dfu"),
    (0x02, "telemetry"),
    (0x04, "logs"),
    (0x08, "cfg"),
    (0x10, "auth"),
    (0x20, "diag"),
]

SLOT_BITS = [(0x01, "valid"), (0x02, "active"), (0x04, "pending"), (0x08, "confirmed")]

GROUPS = ["summary", "timing", "gnss", "power", "net", "ptp", "alarms"]

# cfg_schema.h value types.
T_BOOL, T_U8, T_U16, T_U32, T_U64, T_I32, T_F32, T_STR, T_BLOB = range(1, 10)
TYPE_NAME = {
    T_BOOL: "bool", T_U8: "u8", T_U16: "u16", T_U32: "u32", T_U64: "u64",
    T_I32: "i32", T_F32: "f32", T_STR: "str", T_BLOB: "blob",
}
NUM_WIDTH = {T_BOOL: 1, T_U8: 1, T_U16: 2, T_U32: 4, T_U64: 8, T_I32: 4, T_F32: 4}

CFG_FLAG_BITS = [
    (0x01, "runtime"),
    (0x02, "reboot"),
    (0x04, "secret"),
    (0x08, "cal"),
]

LEVELS = ["emerg", "alert", "crit", "err", "warning", "notice", "info", "debug"]
SUBSYS = ["SYS", "TIMING", "GNSS", "NET", "NTP", "NTS", "PTP", "PWR", "THERM",
          "UI", "SEC", "MCP", "SNMP", "FAULT"]

FW_CHUNK_MAX = 1024
MCUBOOT_MAGIC = 0x96F3B83D
FACTORY_MAGIC = 0x54434146  # "FACT" read little-endian

DFU_STATE = {0: "idle", 1: "active", 2: "suspended", 3: "verified"}


class McpError(Exception):
    """A command answered with a non-OK status."""

    def __init__(self, cmd: int, status: int) -> None:
        super().__init__(
            "%s -> %s" % (CMD_NAME.get(cmd, "0x%02X" % cmd),
                          ERR.get(status, "status %d" % status))
        )
        self.cmd = cmd
        self.status = status


class ProtocolError(Exception):
    """The peer sent something that is not a frame we can use."""


# --------------------------------------------------------------------------
# COBS + CRC framing (pure)
# --------------------------------------------------------------------------


def cobs_encode(data: bytes) -> bytes:
    """COBS-encode a frame body; the caller appends the 0x00 delimiter.

    >>> cobs_encode(b'').hex()
    '01'
    >>> cobs_encode(b'\\x11\\x22\\x00\\x33').hex()
    '0311220233'
    >>> cobs_encode(b'\\x00').hex()
    '0101'
    >>> e = cobs_encode(bytes(range(1, 255)))   # 254 non-zero bytes
    >>> len(e), hex(e[0]), hex(e[-1])           # 0xFF group, no trailing 0x01
    (255, '0xff', '0xfe')
    """
    out = bytearray()
    idx = 0
    n = len(data)
    while True:
        zero = data.find(0, idx)
        if zero < 0:
            zero = n
        block = data[idx:zero]
        while len(block) > 254:
            out.append(0xFF)
            out += block[:254]
            block = block[254:]
        out.append(len(block) + 1)
        out += block
        idx = zero + 1
        if idx > n:
            break
        if idx == n:
            # The body ended on a zero: one more empty group carries it.
            out.append(1)
            break
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """Decode a COBS body (delimiter already stripped).

    >>> cobs_decode(cobs_encode(b'\\x00\\x01\\x00')) == b'\\x00\\x01\\x00'
    True
    """
    out = bytearray()
    i = 0
    n = len(data)
    if n == 0:
        raise ProtocolError("empty COBS body")
    while i < n:
        code = data[i]
        i += 1
        if code == 0:
            raise ProtocolError("delimiter inside a COBS body")
        run = code - 1
        if i + run > n:
            raise ProtocolError("COBS group runs past the frame")
        chunk = data[i:i + run]
        if 0 in chunk:
            raise ProtocolError("zero inside a COBS group")
        out += chunk
        i += run
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


def crc32(data: bytes) -> int:
    """CRC-32/ISO-HDLC, the same catalogue entry core/util/crc.c implements."""
    return binascii.crc32(data) & 0xFFFFFFFF


def build_frame(ftype: int, cmd: int, seq: int, payload: bytes = b"",
                flags: int = 0) -> bytes:
    """Build one decoded (pre-COBS) frame.

    The vector below is HELLO/REQ/seq 1 with an empty payload, the same one
    firmware/tests/host/test_mcp.c pins:

    >>> build_frame(0, 0x01, 1).hex()
    '0100010001000000376b68da'
    >>> build_frame(0, 0x20, 7, b'\\x02').hex()
    '0100200007000100027aee2363'

    The two firmware-update frames below are the shared cross-codec vectors:
    the identical byte strings are asserted against mcp_wire_build() in
    firmware/tests/host/test_mcp_dfu.c (test_wire_vectors_match_the_tool), so
    the two implementations cannot drift apart silently.

    FW_BEGIN/REQ/seq 1, size 0x00020000, sha256 = 0x00..0x1F:

    >>> build_frame(0, 0x41, 1, struct.pack('<I', 0x20000) + bytes(range(32))).hex()
    '010041000100240000000200000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1ff47924c1'

    FW_DATA/REQ/seq 2, offset 0, data AA BB CC DD:

    >>> build_frame(0, 0x42, 2, struct.pack('<I', 0) + bytes([0xAA,0xBB,0xCC,0xDD])).hex()
    '010042000200080000000000aabbccdd48e91869'
    """
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload of %d exceeds %d" % (len(payload), MAX_PAYLOAD))
    head = struct.pack("<BBBBHH", MCP_VER, ftype, cmd, flags, seq & 0xFFFF,
                       len(payload)) + payload
    return head + struct.pack("<I", crc32(head))


def parse_frame(frame: bytes) -> Dict[str, Any]:
    """Validate and split a decoded frame."""
    if len(frame) < HDR_LEN + CRC_LEN or len(frame) > MAX_FRAME:
        raise ProtocolError("frame length %d out of range" % len(frame))
    ver, ftype, cmd, flags, seq, plen = struct.unpack("<BBBBHH", frame[:HDR_LEN])
    if plen != len(frame) - HDR_LEN - CRC_LEN:
        raise ProtocolError("length field disagrees with the frame")
    want = struct.unpack("<I", frame[-CRC_LEN:])[0]
    got = crc32(frame[:-CRC_LEN])
    if want != got:
        raise ProtocolError("CRC mismatch (%08x != %08x)" % (want, got))
    return {
        "ver": ver, "type": ftype, "cmd": cmd, "flags": flags, "seq": seq,
        "payload": frame[HDR_LEN:HDR_LEN + plen],
    }


def wrap(frame: bytes) -> bytes:
    """COBS-encode a frame and append the delimiter, ready for the wire."""
    return cobs_encode(frame) + b"\x00"


class FrameStream:
    """Incremental de-framer: feed received bytes, get whole frames back.

    >>> fs = FrameStream()
    >>> list(fs.feed(wrap(build_frame(1, 1, 3, b'\\x00'))))[0]['seq']
    3
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> Iterable[Dict[str, Any]]:
        for byte in data:
            if byte != 0:
                if len(self._buf) <= MAX_FRAME + MAX_FRAME // 254 + 1:
                    self._buf.append(byte)
                continue
            body = bytes(self._buf)
            self._buf.clear()
            if not body:
                continue  # repeated or leading delimiter: a separator
            try:
                yield parse_frame(cobs_decode(body))
            except ProtocolError:
                continue  # one corrupt frame must not desynchronise the link


# --------------------------------------------------------------------------
# Payload codecs (pure)
# --------------------------------------------------------------------------


def _bits(value: int, table: Sequence[Tuple[int, str]]) -> List[str]:
    return [name for bit, name in table if value & bit]


def decode_hello(p: bytes) -> Dict[str, Any]:
    if len(p) < 74:
        raise ProtocolError("short HELLO payload")
    proto, max_payload, caps = struct.unpack_from("<BHI", p, 1)
    model = p[8:24].split(b"\x00", 1)[0].decode("ascii", "replace")
    board = p[24:32].hex()

    def slot(off: int) -> Dict[str, Any]:
        ver = struct.unpack_from("<4I", p, off)
        size, = struct.unpack_from("<I", p, off + 16)
        flags = p[off + 20]
        return {
            "version": ".".join(str(v) for v in ver),
            "size": size,
            "flags": _bits(flags, SLOT_BITS),
        }

    return {
        "proto": proto, "max_payload": max_payload,
        "capabilities": _bits(caps, CAP_BITS), "model": model, "board_id": board,
        "running": slot(32), "staged": slot(53),
    }


def decode_fw_info(p: bytes) -> Dict[str, Any]:
    nslots = p[1]
    off = 2
    slots = []
    for _ in range(nslots):
        slot, flags = p[off], p[off + 1]
        size, = struct.unpack_from("<I", p, off + 2)
        ver = struct.unpack_from("<4I", p, off + 6)
        slots.append({
            "slot": slot, "flags": _bits(flags, SLOT_BITS), "size": size,
            "version": ".".join(str(v) for v in ver),
        })
        off += 22
    state, total, written, chunk_max, write_block = struct.unpack_from(
        "<BIIII", p, off)
    return {
        "slots": slots,
        "dfu": {
            "state": DFU_STATE.get(state, state), "total": total,
            "written": written, "chunk_max": chunk_max,
            "write_block": write_block,
        },
    }


def decode_cfg_value(vtype: int, raw: bytes) -> Any:
    """Turn an on-the-wire config value into a Python object."""
    if vtype == T_BOOL:
        return bool(raw[0])
    if vtype in (T_U8, T_U16, T_U32, T_U64):
        return int.from_bytes(raw, "little", signed=False)
    if vtype == T_I32:
        return int.from_bytes(raw, "little", signed=True)
    if vtype == T_F32:
        return struct.unpack("<f", raw)[0]
    if vtype == T_STR:
        return raw.decode("utf-8", "replace")
    return raw.hex()


def encode_cfg_value(vtype: int, text: str) -> bytes:
    """Turn a command-line string into an on-the-wire config value."""
    if vtype == T_BOOL:
        low = text.strip().lower()
        if low in ("1", "true", "yes", "on"):
            return b"\x01"
        if low in ("0", "false", "no", "off"):
            return b"\x00"
        raise ValueError("expected a boolean, got %r" % text)
    if vtype in (T_U8, T_U16, T_U32, T_U64):
        return int(text, 0).to_bytes(NUM_WIDTH[vtype], "little", signed=False)
    if vtype == T_I32:
        return int(text, 0).to_bytes(4, "little", signed=True)
    if vtype == T_F32:
        return struct.pack("<f", float(text))
    if vtype == T_STR:
        return text.encode("utf-8")
    return bytes.fromhex(text)


def decode_log_records(p: bytes, off: int, count: int) -> List[Dict[str, Any]]:
    recs = []
    for _ in range(count):
        seq, mono, level, sub, ln = struct.unpack_from("<IQBBB", p, off)
        off += 15
        msg = p[off:off + ln].decode("utf-8", "replace")
        off += ln
        recs.append({
            "seq": seq, "mono_ms": mono,
            "level": LEVELS[level] if level < len(LEVELS) else level,
            "subsys": SUBSYS[sub] if sub < len(SUBSYS) else sub,
            "msg": msg,
        })
    return recs


def group_mask(names: Sequence[str]) -> int:
    mask = 0
    for name in names:
        key = name.strip().lower()
        if key not in GROUPS:
            raise ValueError("unknown status group %r (have: %s)"
                             % (name, ", ".join(GROUPS)))
        mask |= 1 << GROUPS.index(key)
    return mask


# --------------------------------------------------------------------------
# Serial link
# --------------------------------------------------------------------------


class Link:
    """Request/response over a serial port, with retries and event delivery."""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 2.0,
                 retries: int = 3, verbose: bool = False) -> None:
        try:
            import serial  # noqa: PLC0415 - optional at import time
        except ImportError:  # pragma: no cover - environment dependent
            raise SystemExit(
                "meridian_ctl: pyserial is required (pip install pyserial)")
        self._serial_mod = serial
        self.port_name = port
        self.baud = baud
        self.timeout = timeout
        self.retries = retries
        self.verbose = verbose
        self._seq = 0
        self._stream = FrameStream()
        self._events: List[Dict[str, Any]] = []
        self._port = None
        try:
            self.open()
        except Exception as exc:
            raise SystemExit("meridian_ctl: cannot open %s: %s" % (port, exc))

    # -- lifecycle ------------------------------------------------------
    def open(self) -> None:
        self._port = self._serial_mod.Serial(
            self.port_name, self.baud, timeout=0.05, write_timeout=self.timeout)
        self._stream = FrameStream()

    def close(self) -> None:
        if self._port is not None:
            try:
                self._port.close()
            finally:
                self._port = None

    def reopen(self, attempts: int = 30, delay: float = 0.5) -> None:
        """Wait for the device to come back after a disconnect or reboot."""
        self.close()
        last: Optional[Exception] = None
        for _ in range(attempts):
            time.sleep(delay)
            try:
                self.open()
                return
            except Exception as exc:  # pragma: no cover - hardware dependent
                last = exc
        raise SystemExit("meridian_ctl: %s did not come back: %s"
                         % (self.port_name, last))

    def __enter__(self) -> "Link":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    # -- transport ------------------------------------------------------
    def _pump(self, deadline: float) -> Iterable[Dict[str, Any]]:
        while time.monotonic() < deadline:
            chunk = self._port.read(4096)
            if not chunk:
                continue
            for frame in self._stream.feed(chunk):
                yield frame

    def poll_events(self, seconds: float) -> List[Dict[str, Any]]:
        """Collect EVT frames for a while. Any stray RSP is discarded."""
        out: List[Dict[str, Any]] = []
        out.extend(self._events)
        self._events.clear()
        for frame in self._pump(time.monotonic() + seconds):
            if frame["type"] == T_EVT:
                out.append(frame)
        return out

    def request(self, cmd: int, payload: bytes = b"",
                timeout: Optional[float] = None) -> bytes:
        """Send one REQ and return the RSP payload, raising on a non-OK status."""
        frame = self.request_raw(cmd, payload, timeout)
        status = frame[0] if frame else 9
        if status != 0:
            raise McpError(cmd, status)
        return frame

    def request_raw(self, cmd: int, payload: bytes = b"",
                    timeout: Optional[float] = None) -> bytes:
        """As request(), but hand back the payload whatever the status is."""
        tmo = self.timeout if timeout is None else timeout
        last: Optional[Exception] = None

        for attempt in range(self.retries):
            self._seq = (self._seq + 1) & 0xFFFF
            seq = self._seq or 1
            wire = wrap(build_frame(T_REQ, cmd, seq, payload))
            if self.verbose:
                sys.stderr.write("> %s seq=%d len=%d\n"
                                 % (CMD_NAME.get(cmd, hex(cmd)), seq, len(payload)))
            self._port.write(wire)
            self._port.flush()

            try:
                for frame in self._pump(time.monotonic() + tmo):
                    if frame["type"] == T_EVT:
                        self._events.append(frame)
                        continue
                    if frame["type"] != T_RSP:
                        continue
                    if frame["seq"] != seq or frame["cmd"] != cmd:
                        continue  # a stale answer to a retried request
                    if self.verbose:
                        sys.stderr.write("< %s status=%s\n" % (
                            CMD_NAME.get(cmd, hex(cmd)),
                            ERR.get(frame["payload"][0], "?")))
                    return frame["payload"]
            except OSError as exc:  # pragma: no cover - hardware dependent
                last = exc
                self.reopen()
                continue
            last = TimeoutError("no answer to %s (attempt %d/%d)"
                                % (CMD_NAME.get(cmd, hex(cmd)), attempt + 1,
                                   self.retries))
        raise SystemExit("meridian_ctl: %s" % last)


# --------------------------------------------------------------------------
# Client
# --------------------------------------------------------------------------


class Client:
    def __init__(self, link: Link) -> None:
        self.link = link
        self._schema: Optional[List[Dict[str, Any]]] = None

    # -- identity -------------------------------------------------------
    def hello(self) -> Dict[str, Any]:
        return decode_hello(self.link.request(CMD["HELLO"]))

    def auth(self, password: str) -> Dict[str, Any]:
        p = self.link.request(CMD["AUTH"], password.encode("utf-8"))
        return {"authenticated": bool(p[1]),
                "session_seconds": struct.unpack_from("<H", p, 2)[0]}

    def reboot(self, mode: int) -> None:
        self.link.request(CMD["REBOOT"], bytes([mode]))

    # -- status ---------------------------------------------------------
    def status(self, group: int) -> Dict[str, Any]:
        p = self.link.request(CMD["STATUS_GET"], bytes([group]))
        body = p[2:]
        return {
            "group": GROUPS[p[1]] if p[1] < len(GROUPS) else p[1],
            "struct_version": body[0] if body else None,
            "raw": body.hex(),
        }

    def telem_sub(self, mask: int, rate: int) -> None:
        self.link.request(CMD["TELEM_SUB"], bytes([mask, rate]))

    def telem_unsub(self) -> None:
        self.link.request(CMD["TELEM_UNSUB"])

    # -- config ---------------------------------------------------------
    def schema(self, refresh: bool = False) -> List[Dict[str, Any]]:
        """The full key list, fetched once per run via CFG_LIST paging."""
        if self._schema is not None and not refresh:
            return self._schema

        keys: List[Dict[str, Any]] = []
        start = 0
        while True:
            p = self.link.request(CMD["CFG_LIST"],
                                  struct.pack("<HB", start, 32))
            nxt, count = struct.unpack_from("<HB", p, 1)
            off = 4
            for _ in range(count):
                kid, ktype, kflags, nlen = struct.unpack_from("<HBBB", p, off)
                name = p[off + 5:off + 5 + nlen].decode("ascii", "replace")
                keys.append({"id": kid, "type": ktype, "flags": kflags,
                             "name": name})
                off += 5 + nlen
            if nxt == 0:
                break
            start = nxt
        self._schema = keys
        return keys

    def resolve(self, key: str) -> Dict[str, Any]:
        """Accept a key name or a numeric ID and return its schema row."""
        for row in self.schema():
            if row["name"] == key:
                return row
        try:
            wanted = int(key, 0)
        except ValueError:
            raise SystemExit("meridian_ctl: no config key named %r" % key)
        for row in self.schema():
            if row["id"] == wanted:
                return row
        raise SystemExit("meridian_ctl: no config key with id 0x%04X" % wanted)

    def cfg_get(self, key: str, staged: bool = False) -> Dict[str, Any]:
        row = self.resolve(key)
        req = struct.pack("<HB", row["id"], 1 if staged else 0)
        p = self.link.request(CMD["CFG_GET"], req)
        kid, ktype, kflags, vlen = struct.unpack_from("<HBBH", p, 1)
        raw = p[7:7 + vlen]
        return {
            "id": kid, "name": row["name"], "type": TYPE_NAME.get(ktype, ktype),
            "flags": _bits(kflags, CFG_FLAG_BITS),
            "value": decode_cfg_value(ktype, raw),
        }

    def cfg_set(self, key: str, text: str) -> Dict[str, Any]:
        row = self.resolve(key)
        raw = encode_cfg_value(row["type"], text)
        req = struct.pack("<HBH", row["id"], row["type"], len(raw)) + raw
        self.link.request(CMD["CFG_SET"], req)
        return {"id": row["id"], "name": row["name"], "staged": text}

    def cfg_commit(self) -> Dict[str, Any]:
        p = self.link.request(CMD["CFG_COMMIT"])
        applied, reboot_keys, groups, persist_errors = struct.unpack_from(
            "<HHIH", p, 1)
        return {"applied": applied, "reboot_required_keys": reboot_keys,
                "reboot_groups_mask": groups,
                "reboot_required": reboot_keys > 0,
                "persist_errors": persist_errors,
                "persisted": persist_errors == 0}

    def cfg_revert(self) -> Dict[str, Any]:
        p = self.link.request(CMD["CFG_REVERT"])
        return {"dropped": struct.unpack_from("<H", p, 1)[0]}

    def cfg_export(self, secrets: bool = False) -> bytes:
        blob = bytearray()
        while True:
            req = struct.pack("<IB", len(blob), 1 if secrets else 0)
            p = self.link.request(CMD["CFG_EXPORT"], req)
            offset, more = struct.unpack_from("<IB", p, 1)
            if offset != len(blob):
                raise ProtocolError("export offset %d, expected %d"
                                    % (offset, len(blob)))
            blob += p[6:]
            if not more:
                break
        return bytes(blob)

    def phase_export(self) -> bytes:
        """Fetch one phase record (mcp_wire.h PHASE_EXPORT 0x24).

        The offset contract is CFG_EXPORT's: 0 restarts and snapshots, then
        each request names the byte offset the previous chunk ended at. The
        device serves every chunk from the one snapshot, so the record is a
        coherent window on the discipline loop's phase ring rather than a
        splice of several.

        Returns the raw record. This tool deliberately does NOT decode or
        analyse it: the estimators live in core/stats and are exercised
        through tools/meridian_phase.c, so that there is exactly one
        implementation of the arithmetic and it is the one the firmware's own
        test suite pins against closed forms.
        """
        blob = bytearray()
        while True:
            p = self.link.request(CMD["PHASE_EXPORT"],
                                  struct.pack("<I", len(blob)))
            offset, more = struct.unpack_from("<IB", p, 1)
            if offset != len(blob):
                raise ProtocolError("phase export offset %d, expected %d"
                                    % (offset, len(blob)))
            blob += p[6:]
            if not more:
                break
        return bytes(blob)

    def cfg_import(self, blob: bytes, strict: bool = True,
                   chunk: int = 512) -> Dict[str, Any]:
        off = 0
        result: Dict[str, Any] = {}
        while True:
            piece = blob[off:off + chunk]
            last = (off + len(piece)) >= len(blob)
            flags = (0x01 if strict else 0x00) | (0x02 if last else 0x00)
            req = struct.pack("<IB", off, flags) + piece
            p = self.link.request(CMD["CFG_IMPORT"], req)
            nxt, complete, applied, groups, persist_errors = struct.unpack_from(
                "<IBHIH", p, 1)
            result = {"applied": applied, "complete": bool(complete),
                      "reboot_groups_mask": groups,
                      "persist_errors": persist_errors,
                      "persisted": persist_errors == 0}
            if last:
                break
            if nxt <= off:
                raise ProtocolError(
                    "import stalled at offset %d (device reported %d)"
                    % (off, nxt))
            off = nxt
        return result

    def factory_reset(self) -> None:
        self.link.request(CMD["FACTORY_RESET"], struct.pack("<I", FACTORY_MAGIC))

    # -- logs -----------------------------------------------------------
    def log_tail(self, cursor: int, follow: bool, count: int, level: int,
                 sub_mask: int) -> Tuple[List[Dict[str, Any]], int, int]:
        req = struct.pack("<IBBBH", cursor, 1 if follow else 0, count, level,
                          sub_mask)
        p = self.link.request(CMD["LOG_TAIL"], req)
        nxt, gap, n = struct.unpack_from("<IIB", p, 1)
        return decode_log_records(p, 10, n), nxt, gap

    def log_level(self, subsys: int, level: int) -> None:
        self.link.request(CMD["LOG_LEVEL"], bytes([subsys, level]))

    # -- firmware -------------------------------------------------------
    def fw_info(self) -> Dict[str, Any]:
        return decode_fw_info(self.link.request(CMD["FW_INFO"]))

    def fw_begin(self, size: int, digest: bytes) -> Dict[str, Any]:
        p = self.link.request(CMD["FW_BEGIN"], struct.pack("<I", size) + digest,
                              timeout=15.0)
        nxt, chunk_max, write_block, resumed = struct.unpack_from("<IIIB", p, 1)
        return {"next": nxt, "chunk_max": min(chunk_max, FW_CHUNK_MAX),
                "write_block": max(1, write_block), "resumed": bool(resumed)}

    def fw_data(self, off: int, data: bytes) -> Tuple[int, int]:
        p = self.link.request_raw(CMD["FW_DATA"],
                                  struct.pack("<I", off) + data, timeout=10.0)
        return p[0], struct.unpack_from("<I", p, 1)[0]

    def fw_end(self) -> int:
        p = self.link.request(CMD["FW_END"], timeout=60.0)
        return struct.unpack_from("<I", p, 1)[0]

    def fw_confirm(self) -> None:
        self.link.request(CMD["FW_CONFIRM"], timeout=10.0)

    def fw_revert(self) -> None:
        self.link.request(CMD["FW_REVERT"], timeout=10.0)

    def diag(self, sub: int) -> Dict[str, Any]:
        p = self.link.request(CMD["DIAG"], bytes([sub]))
        return {"sub": p[1], "raw": p[2:].hex()}


# --------------------------------------------------------------------------
# Output helpers
# --------------------------------------------------------------------------


class Out:
    def __init__(self, as_json: bool) -> None:
        self.json = as_json

    def obj(self, value: Any) -> None:
        if self.json:
            print(json.dumps(value, sort_keys=True))
        else:
            self._pretty(value, 0)

    def line(self, text: str) -> None:
        if not self.json:
            print(text)

    def _pretty(self, value: Any, indent: int) -> None:
        pad = " " * indent
        if isinstance(value, dict):
            width = max((len(str(k)) for k in value), default=0)
            for key, val in value.items():
                if isinstance(val, (dict, list)):
                    print("%s%s:" % (pad, key))
                    self._pretty(val, indent + 2)
                else:
                    print("%s%-*s  %s" % (pad, width, key, self._scalar(val)))
        elif isinstance(value, list):
            for item in value:
                if isinstance(item, (dict, list)):
                    self._pretty(item, indent + 2)
                    print()
                else:
                    print("%s- %s" % (pad, self._scalar(item)))
        else:
            print("%s%s" % (pad, self._scalar(value)))

    @staticmethod
    def _scalar(value: Any) -> str:
        if isinstance(value, bool):
            return "yes" if value else "no"
        if value is None:
            return "-"
        return str(value)


def human_size(n: int) -> str:
    for unit in ("B", "KiB", "MiB"):
        if n < 1024 or unit == "MiB":
            return "%.1f %s" % (n, unit) if unit != "B" else "%d B" % n
        n /= 1024.0
    return str(n)


def progress(done: int, total: int, started: float) -> None:
    if not sys.stderr.isatty():
        return
    pct = (100.0 * done / total) if total else 100.0
    elapsed = max(time.monotonic() - started, 1e-6)
    rate = done / elapsed
    eta = (total - done) / rate if rate > 0 else 0.0
    sys.stderr.write("\r  %5.1f%%  %8d / %d B  %6.1f kB/s  ETA %3.0f s"
                     % (pct, done, total, rate / 1000.0, eta))
    sys.stderr.flush()


# --------------------------------------------------------------------------
# Commands
# --------------------------------------------------------------------------


def cmd_info(cli: Client, out: Out, args: argparse.Namespace) -> int:
    del args
    out.obj(cli.hello())
    return 0


def cmd_status(cli: Client, out: Out, args: argparse.Namespace) -> int:
    if args.group == "all":
        out.obj([cli.status(i) for i in range(len(GROUPS))])
    else:
        out.obj(cli.status(GROUPS.index(args.group)))
    return 0


def cmd_watch(cli: Client, out: Out, args: argparse.Namespace) -> int:
    mask = group_mask(args.groups.split(","))
    cli.telem_sub(mask, args.rate)
    out.line("subscribed: groups=%s rate=%d Hz (ctrl-c to stop)"
             % (args.groups, args.rate))
    try:
        while True:
            for frame in cli.link.poll_events(0.25):
                if frame["cmd"] != CMD["TELEM_SUB"]:
                    continue
                p = frame["payload"]
                rec = {
                    "group": GROUPS[p[0]] if p[0] < len(GROUPS) else p[0],
                    "struct_version": p[1] if len(p) > 1 else None,
                    "raw": p[1:].hex(),
                }
                out.obj(rec)
    except KeyboardInterrupt:
        out.line("")
    finally:
        try:
            cli.telem_unsub()
        except Exception:  # pragma: no cover - best effort on the way out
            pass
    return 0


def cmd_cfg_list(cli: Client, out: Out, args: argparse.Namespace) -> int:
    rows = cli.schema()
    if args.group:
        want = args.group.lower()
        rows = [r for r in rows if r["name"].split(".")[0] == want]
    if out.json:
        out.obj([{"id": "0x%04X" % r["id"], "name": r["name"],
                  "type": TYPE_NAME.get(r["type"], r["type"]),
                  "flags": _bits(r["flags"], CFG_FLAG_BITS)} for r in rows])
        return 0
    width = max((len(r["name"]) for r in rows), default=8)
    for row in rows:
        print("0x%04X  %-*s  %-5s  %s"
              % (row["id"], width, row["name"],
                 TYPE_NAME.get(row["type"], row["type"]),
                 ",".join(_bits(row["flags"], CFG_FLAG_BITS))))
    return 0


def cmd_cfg_get(cli: Client, out: Out, args: argparse.Namespace) -> int:
    out.obj(cli.cfg_get(args.key, args.staged))
    return 0


def cmd_cfg_set(cli: Client, out: Out, args: argparse.Namespace) -> int:
    out.obj(cli.cfg_set(args.key, args.value))
    out.line("staged - run `cfg-commit` to apply")
    return 0


def _warn_persist(out: Out, res: Dict[str, Any]) -> int:
    """Print a persistence warning and return the process exit code."""
    if res.get("persist_errors", 0) and not out.json:
        print("WARNING: %d key(s) applied to RAM but NOT saved to flash; the "
              "change is live now but will not survive a reboot"
              % res["persist_errors"])
    return 4 if res.get("persist_errors", 0) else 0


def cmd_cfg_commit(cli: Client, out: Out, args: argparse.Namespace) -> int:
    del args
    res = cli.cfg_commit()
    out.obj(res)
    if res["reboot_required"] and not out.json:
        print("a reboot is required for %d of the applied keys"
              % res["reboot_required_keys"])
    return _warn_persist(out, res)


def cmd_cfg_revert(cli: Client, out: Out, args: argparse.Namespace) -> int:
    del args
    out.obj(cli.cfg_revert())
    return 0


def cmd_cfg_export(cli: Client, out: Out, args: argparse.Namespace) -> int:
    blob = cli.cfg_export(args.secrets)
    with open(args.file, "wb") as fh:
        fh.write(blob)
    out.obj({"file": args.file, "bytes": len(blob),
             "secrets": bool(args.secrets)})
    return 0


# Wire constants from core/stats/phase_rec.h. Duplicated here rather than
# parsed, because this tool only ever reports what it fetched — the decoding
# that matters is done by meridian_phase, which links the C codec itself.
PHR_F_IDENT = 0x04
PHR_HDR_LEN = 24


def _phase_describe(blob: bytes, path: str) -> dict:
    """Header fields of one fetched record, for the operator's log line."""
    if len(blob) < PHR_HDR_LEN or blob[:4] != b"PHR1":
        raise SystemExit("meridian_ctl: device did not return a phase record")

    ver, flags = blob[4], blob[5]
    n, tau0_ns, gaps = struct.unpack_from("<HII", blob, 6)
    d = {"file": path, "bytes": len(blob), "samples": n,
         "tau0_ns": tau0_ns, "absorbed_gaps": gaps,
         "fit_for_adev": gaps == 0}

    # The identity block is the last 12 bytes, appended past both series.
    # Reported so an operator polling repeatedly can see the window sliding —
    # if seq0 does not advance between polls the ring is not moving, and if the
    # epoch changes the loop reset and the chain has to start over.
    if ver >= 3 and (flags & PHR_F_IDENT):
        epoch, seq0 = struct.unpack_from("<IQ", blob, len(blob) - 12)
        d["epoch"] = "%08x" % epoch
        d["seq0"] = seq0
        d["seq_end"] = seq0 + n
    else:
        d["epoch"] = None
    return d


def cmd_phase_export(cli: Client, out: Out, args: argparse.Namespace) -> int:
    # Reported, not interpreted. A non-zero absorbed-gap count means the
    # discipline loop absorbed a short PPS gap into the ring, so the samples are
    # not uniformly spaced and the record is not fit for ADEV as a whole.
    # Deciding what to do about that belongs to meridian_phase, which is where
    # the estimators are — and so does deciding whether a set of records is
    # joinable, which is why --repeat only fetches and names the files.
    if args.repeat <= 1:
        blob = cli.phase_export()
        with open(args.file, "wb") as fh:
            fh.write(blob)
        d = _phase_describe(blob, args.file)
        d["analyse_with"] = "meridian_phase %s" % args.file
        out.obj(d)
        return 0

    # Repeat poll. The device ring is 512 samples at 1 Hz, so a capture is
    # 8 min 32 s and one record's tau axis stops at 128 s. Polling faster than
    # the ring empties makes successive records OVERLAP, and the overlap is what
    # lets meridian_phase prove the concatenation is contiguous rather than
    # assuming it. The default interval is deliberately well under the ring's
    # 512 s so a slow link or a missed poll still leaves samples in common.
    base, dot, ext = args.file.rpartition(".")
    if not dot:
        base, ext = args.file, "phr"
    files = []
    records = []
    for i in range(args.repeat):
        if i > 0:
            time.sleep(args.interval)
        path = "%s.%03d.%s" % (base, i, ext)
        blob = cli.phase_export()
        with open(path, "wb") as fh:
            fh.write(blob)
        d = _phase_describe(blob, path)
        files.append(path)
        records.append(d)
        out.obj(d)

    unident = [r["file"] for r in records if r["epoch"] is None]
    epochs = sorted({r["epoch"] for r in records if r["epoch"] is not None})
    summary = {"records": len(files), "epochs": epochs,
               "analyse_with": "meridian_phase " + " ".join(files)}
    if unident:
        summary["not_joinable"] = unident
    if len(epochs) > 1:
        # Say it here too. The join will refuse, but an operator watching a
        # long capture wants to know at the moment it happened that the loop
        # reset under them, not an hour later when the analysis fails.
        summary["warning"] = ("the capture epoch changed mid-poll: the "
                              "discipline loop reset, and records from either "
                              "side of that cannot be joined")
    out.obj(summary)
    return 0


def cmd_cfg_import(cli: Client, out: Out, args: argparse.Namespace) -> int:
    with open(args.file, "rb") as fh:
        blob = fh.read()
    if blob[:4] != b"MCF1":
        raise SystemExit("meridian_ctl: %s is not a Meridian config export"
                         % args.file)
    res = cli.cfg_import(blob, strict=not args.lenient)
    res["file"] = args.file
    out.obj(res)
    return _warn_persist(out, res)


def cmd_factory_reset(cli: Client, out: Out, args: argparse.Namespace) -> int:
    if not args.yes:
        out.line("Factory reset erases all configuration AND the admin "
                 "credential.")
        out.line("The credential cannot be set over this channel by design; "
                 "afterwards you must set a new admin password from the local "
                 "front-panel UI or the ACM0 console shell before any "
                 "authenticated MCP operation will work again.")
        answer = input("Type FACTORY to confirm: ")
        if answer.strip() != "FACTORY":
            out.line("aborted")
            return 1
    cli.factory_reset()
    out.obj({"factory_reset": True,
             "note": "set a new admin password via the local UI or ACM0 shell"})
    return 0


def cmd_log_tail(cli: Client, out: Out, args: argparse.Namespace) -> int:
    level = LEVELS.index(args.level)
    sub_mask = 0
    if args.subsys:
        for name in args.subsys.split(","):
            key = name.strip().upper()
            if key not in SUBSYS:
                raise SystemExit("meridian_ctl: unknown subsystem %r" % name)
            sub_mask |= 1 << SUBSYS.index(key)

    def emit(records: List[Dict[str, Any]], gap: int) -> None:
        if gap:
            sys.stderr.write("  ... %d record(s) lost (ring overrun)\n" % gap)
        for rec in records:
            if out.json:
                print(json.dumps(rec, sort_keys=True))
            else:
                print("%10u %8.3f %-7s %-6s %s"
                      % (rec["seq"], rec["mono_ms"] / 1000.0, rec["level"],
                         rec["subsys"], rec["msg"]))

    recs, cursor, gap = cli.log_tail(args.cursor, args.follow, args.count,
                                     level, sub_mask)
    emit(recs, gap)
    if not args.follow:
        return 0

    try:
        while True:
            for frame in cli.link.poll_events(0.25):
                if frame["cmd"] != CMD["LOG_TAIL"]:
                    continue
                p = frame["payload"]
                cursor, gap, n = struct.unpack_from("<IIB", p, 0)
                emit(decode_log_records(p, 9, n), gap)
    except KeyboardInterrupt:
        out.line("")
    return 0


def cmd_log_level(cli: Client, out: Out, args: argparse.Namespace) -> int:
    sub = 0xFF if args.subsys.lower() == "all" else SUBSYS.index(
        args.subsys.upper())
    cli.log_level(sub, LEVELS.index(args.level))
    out.obj({"subsys": args.subsys, "level": args.level})
    return 0


def cmd_fw_info(cli: Client, out: Out, args: argparse.Namespace) -> int:
    del args
    out.obj(cli.fw_info())
    return 0


def cmd_fw_upload(cli: Client, out: Out, args: argparse.Namespace) -> int:
    with open(args.image, "rb") as fh:
        image = fh.read()
    if len(image) < 4:
        raise SystemExit("meridian_ctl: %s is too small to be an image"
                         % args.image)
    magic = struct.unpack_from("<I", image, 0)[0]
    if magic != MCUBOOT_MAGIC and not args.force:
        raise SystemExit(
            "meridian_ctl: %s does not start with the MCUboot header magic "
            "(0x%08X); pass --force to send it anyway" % (args.image, magic))

    digest = hashlib.sha256(image).digest()
    out.line("image  %s  %s  sha256 %s"
             % (os.path.basename(args.image), human_size(len(image)),
                digest.hex()[:16]))

    session = cli.fw_begin(len(image), digest)
    block = session["write_block"]
    # Every non-final chunk must be a multiple of the flash write block (the
    # device buffers only the final short one), so round the chunk size down to
    # a whole number of blocks. The trailing remainder rides in the final chunk,
    # which may be any length.
    chunk = min(args.chunk, session["chunk_max"])
    chunk = max(block, (chunk // block) * block)
    off = session["next"]
    if session["resumed"]:
        out.line("resuming at offset %d (%.1f%% already staged)"
                 % (off, 100.0 * off / len(image)))

    started = time.monotonic()
    while off < len(image):
        piece = image[off:off + chunk]
        try:
            status, expected = cli.fw_data(off, piece)
        except OSError:  # pragma: no cover - hardware dependent
            out.line("\nlink lost - reconnecting and resuming")
            cli.link.reopen()
            session = cli.fw_begin(len(image), digest)
            off = session["next"]
            continue

        if status == 0:
            off = expected
        elif status == 5:  # ERR_OFFSET: the device knows better than we do
            off = expected
        else:
            progress(off, len(image), started)
            sys.stderr.write("\n")
            raise SystemExit("meridian_ctl: FW_DATA at %d -> %s"
                             % (off, ERR.get(status, status)))
        progress(off, len(image), started)

    if sys.stderr.isatty():
        sys.stderr.write("\n")

    staged = cli.fw_end()
    out.obj({"staged_bytes": staged, "verified": True,
             "seconds": round(time.monotonic() - started, 1)})

    if args.no_reboot:
        out.line("image staged and pending; reboot to test it")
        return 0

    out.line("rebooting into the staged image")
    cli.reboot(0)
    out.line("after it comes up and passes its health checks, run `fw-confirm` "
             "(an unconfirmed image reverts on the next boot)")
    return 0


def cmd_fw_confirm(cli: Client, out: Out, args: argparse.Namespace) -> int:
    del args
    cli.fw_confirm()
    out.obj({"confirmed": True})
    return 0


def cmd_fw_revert(cli: Client, out: Out, args: argparse.Namespace) -> int:
    del args
    cli.fw_revert()
    out.obj({"revert_requested": True})
    return 0


def cmd_reboot(cli: Client, out: Out, args: argparse.Namespace) -> int:
    mode = 1 if args.recovery else (2 if args.halt else 0)
    cli.reboot(mode)
    out.obj({"reboot": ["normal", "recovery", "halt"][mode]})
    return 0


def cmd_auth(cli: Client, out: Out, args: argparse.Namespace) -> int:
    password = _resolve_password(args)
    if password is None:
        password = getpass.getpass("Meridian password: ")
    out.obj(cli.auth(password))
    return 0


def cmd_diag(cli: Client, out: Out, args: argparse.Namespace) -> int:
    out.obj(cli.diag(args.sub))
    return 0


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="meridian_ctl",
        description="Reference client for the STS1000 Meridian console protocol.")
    ap.add_argument("--port", default=os.environ.get("MERIDIAN_PORT",
                                                     "/dev/ttyACM1"),
                    help="serial device (default: %(default)s)")
    ap.add_argument("--baud", type=int, default=115200,
                    help="baud rate; ignored by CDC-ACM (default: %(default)s)")
    ap.add_argument("--timeout", type=float, default=2.0,
                    help="per-request timeout in seconds (default: %(default)s)")
    ap.add_argument("--retries", type=int, default=3,
                    help="request retries before giving up (default: %(default)s)")
    ap.add_argument("--json", action="store_true",
                    help="machine-readable output")
    ap.add_argument("--password", default=None,
                    help="admin password used to open a session before a "
                         "mutating command; falls back to $MERIDIAN_PASSWORD, "
                         "and to a prompt for mutating commands on a terminal")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="trace requests and responses on stderr")

    sub = ap.add_subparsers(dest="command", required=True)

    def add(name: str, fn: Callable[..., int], help_text: str
            ) -> argparse.ArgumentParser:
        p = sub.add_parser(name, help=help_text)
        p.set_defaults(func=fn)
        return p

    add("info", cmd_info, "protocol, firmware and board identity")

    p = add("status", cmd_status, "one status group, or all of them")
    p.add_argument("group", nargs="?", default="summary",
                   choices=GROUPS + ["all"])

    p = add("watch", cmd_watch, "stream telemetry events")
    p.add_argument("--rate", type=int, default=1, choices=[1, 2, 3, 4])
    p.add_argument("--groups", default="summary")

    p = add("cfg-list", cmd_cfg_list, "list configuration keys")
    p.add_argument("group", nargs="?", help="restrict to one group prefix")

    p = add("cfg-get", cmd_cfg_get, "read one configuration key")
    p.add_argument("key")
    p.add_argument("--staged", action="store_true",
                   help="read the staged value instead of the live one")

    p = add("cfg-set", cmd_cfg_set, "stage a configuration change")
    p.add_argument("key")
    p.add_argument("value")

    add("cfg-commit", cmd_cfg_commit, "apply the staged changes atomically")
    add("cfg-revert", cmd_cfg_revert, "discard the staged changes")

    p = add("cfg-export", cmd_cfg_export, "save the configuration to a file")
    p.add_argument("file")
    p.add_argument("--secrets", action="store_true",
                   help="include secret keys (needs an authenticated session)")

    p = add("phase-export", cmd_phase_export,
            "save the discipline loop's phase record for offline analysis")
    p.add_argument("file")
    p.add_argument("--repeat", type=int, default=1, metavar="N",
                   help="poll N times, writing FILE.000.phr .. FILE.NNN.phr; "
                        "meridian_phase joins them into one long record")
    p.add_argument("--interval", type=float, default=240.0, metavar="S",
                   help="seconds between polls with --repeat (default 240). "
                        "Must stay under the ring's 512 s so consecutive "
                        "records overlap and the join can be verified")

    p = add("cfg-import", cmd_cfg_import, "restore a configuration file")
    p.add_argument("file")
    p.add_argument("--lenient", action="store_true",
                   help="skip unknown keys instead of failing")

    p = add("factory-reset", cmd_factory_reset, "erase all configuration")
    p.add_argument("--yes", action="store_true", help="skip the confirmation")

    p = add("log-tail", cmd_log_tail, "read or follow the log ring")
    p.add_argument("--cursor", type=int, default=0)
    p.add_argument("--count", type=int, default=32)
    p.add_argument("--level", default="debug", choices=LEVELS)
    p.add_argument("--subsys", help="comma-separated subsystem filter")
    p.add_argument("--follow", action="store_true")

    p = add("log-level", cmd_log_level, "set a subsystem's log level")
    p.add_argument("subsys")
    p.add_argument("level", choices=LEVELS)

    add("fw-info", cmd_fw_info, "slot table and DFU session state")

    p = add("fw-upload", cmd_fw_upload, "stream a signed image into slot 1")
    p.add_argument("image")
    p.add_argument("--chunk", type=int, default=FW_CHUNK_MAX)
    p.add_argument("--no-reboot", action="store_true",
                   help="stage the image but do not reboot into it")
    p.add_argument("--force", action="store_true",
                   help="upload even without an MCUboot header magic")

    add("fw-confirm", cmd_fw_confirm, "confirm the running image")
    add("fw-revert", cmd_fw_revert, "revert to the previous image on next boot")

    p = add("reboot", cmd_reboot, "reboot the device")
    p.add_argument("--recovery", action="store_true",
                   help="stay in the bootloader for serial recovery")
    p.add_argument("--halt", action="store_true", help="halt for test")

    add("auth", cmd_auth, "open an authenticated session (uses the global "
                          "--password / $MERIDIAN_PASSWORD, else prompts)")

    p = add("diag", cmd_diag, "run a diagnostic sub-function")
    p.add_argument("sub", type=int, choices=[0, 1, 2, 3, 4])

    return ap


# Subcommands that mutate state and therefore need a session when the device
# has sec.auth.req set. The engine reset_session()s on disconnect (dropping any
# staged config), so a mutating flow MUST run inside a single invocation — a
# separate `auth` process does not carry a session into the next one.
MUTATING = frozenset({
    "reboot", "cfg-set", "cfg-commit", "cfg-revert", "cfg-import",
    "factory-reset", "log-level", "fw-upload", "fw-confirm", "fw-revert",
})


def _resolve_password(args: argparse.Namespace) -> Optional[str]:
    if getattr(args, "password", None) is not None:
        return args.password
    return os.environ.get("MERIDIAN_PASSWORD")


def _authenticate_if_needed(cli: Client, args: argparse.Namespace,
                            out: Out) -> None:
    """Open a session on connect when a mutating command may require one.

    Authenticates when a password is available (``--password`` or
    ``$MERIDIAN_PASSWORD``); for a mutating command on a terminal with none set,
    it prompts. A device with auth disabled may reject the AUTH (no credential),
    which is not fatal here — the command itself is still attempted.
    """
    if args.command == "auth":
        return  # the subcommand authenticates itself
    if "auth" not in cli.hello().get("capabilities", []):
        return  # this build cannot evaluate AUTH at all

    password = _resolve_password(args)
    if password is None:
        if args.command in MUTATING and sys.stdin.isatty():
            password = getpass.getpass("Meridian password: ")
        else:
            return  # read-only, or no way to obtain a password non-interactively
    try:
        cli.auth(password)
    except McpError as exc:
        # Auth may simply be disabled (no credential provisioned); let the
        # command proceed and fail on its own if it really needed a session.
        sys.stderr.write("meridian_ctl: authentication failed (%s); "
                         "continuing unauthenticated\n" % exc)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    out = Out(args.json)

    with Link(args.port, args.baud, args.timeout, args.retries,
              args.verbose) as link:
        cli = Client(link)
        try:
            _authenticate_if_needed(cli, args, out)
            return args.func(cli, out, args)
        except McpError as exc:
            sys.stderr.write("meridian_ctl: %s\n" % exc)
            if exc.status == 1:
                sys.stderr.write("  (run `auth` first: this command mutates "
                                 "state)\n")
            return 2
        except ProtocolError as exc:
            sys.stderr.write("meridian_ctl: protocol error: %s\n" % exc)
            return 3


if __name__ == "__main__":
    sys.exit(main())
