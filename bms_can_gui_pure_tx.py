#!/usr/bin/env python3
"""
bms_can_gui.py -- Dreamfly Eionic BMS CAN GUI (PEAK-CAN / CAN 2.0B Extended)

Companion tool to stm32_cli_tool_v6.py (the existing UART2 GUI). Talks to the
BMS over a PEAK-System PCAN-USB adapter using the SAME CLI_CMD_* protocol
UART2 uses -- live data, fault thresholds, calibration, Solterra charger
config/status/stop-override, device identity (BMS/pack serial, product ID),
and error logs -- via the new generic CAN-CLI tunnel added to the firmware
(can_cli_bridge.h/.c). UART2 is completely independent of this tool; nothing
here touches it.

------------------------------------------------------------------------------
REQUIREMENTS
    pip install python-can ttkbootstrap
    Windows: install PEAK-System's "PCAN-Basic" driver/API (this is the
             python-can 'pcan' backend's dependency, NOT a Python package).
    Linux:   the peak_usb kernel module (usually already in-tree) brings the
             adapter up as a SocketCAN interface; either use bustype='pcan'
             (needs PCAN-Basic-for-Linux) or switch BUS_KWARGS below to
             {'interface': 'socketcan', 'channel': 'can0'} and set the
             bitrate with `ip link` instead.

    ttkbootstrap supplies the theme -- pick any theme in THEME_CHOICES below
    (or type another valid ttkbootstrap theme name into the selector at
    runtime) via the dropdown in the top bar; default is "darkly".

FIRMWARE SIDE
    FDCAN1 in this project runs at 500 kbps Classic CAN 2.0B Extended
    (see fdcan.c, FDCAN_BAUD_RATE) -- the default below matches. If you
    change that #define, change DEFAULT_BITRATE here too.

WIRE PROTOCOL (must match Core/Src/can_cli_bridge.c exactly)
    Two extended CAN IDs:
        CANCLI_ID_REQ  = 0x1FF907  (this GUI -> BMS)
        CANCLI_ID_RESP = 0x1FF908  (BMS -> this GUI)
    Virtual payload (VP) = [cmd byte] + [0..199 data bytes], segmented into
    Classic CAN's 8-byte frames with a minimal ISO-15765-2-style Single /
    First / Consecutive-Frame header (no Flow-Control frame -- see
    can_cli_bridge.h for why that's fine at this message size). See
    CanCliCodec/CanCliReassembler below for the exact byte-level rules.

    The legacy fixed-format IDs (0x1FF900-0x1FF906, can_gui_cmds.h) are NOT
    used by this tool -- they're kept firmware-side for whatever already
    targets them (e.g. an existing BUSMASTER panel), but everything new
    goes through the generic tunnel instead.

NOTE: This variant has NO background ping or sender threads.  TX is locked
by default; you must click "Unlock TX" before sending commands.  The bus
stays silent except during explicit user-initiated transmissions.
------------------------------------------------------------------------------
"""
import queue
import random
import struct
import threading
import time
import csv
import os
import tkinter as tk
from tkinter import filedialog
from dataclasses import dataclass, field
from typing import Optional, Callable

try:
    import can
except ImportError:
    raise SystemExit(
        "python-can is not installed. Run:  pip install python-can\n"
        "(Windows also needs PEAK-System's PCAN-Basic driver installed "
        "separately -- that is not a Python package.)"
    )

try:
    import ttkbootstrap as ttk
    from ttkbootstrap.constants import PRIMARY, SECONDARY, SUCCESS, INFO, WARNING, DANGER, LIGHT, DARK
    from ttkbootstrap.dialogs import Messagebox
except ImportError:
    raise SystemExit(
        "ttkbootstrap is not installed. Run:  pip install ttkbootstrap"
    )

# ============================================================================
# Protocol constants -- MUST match Core/Inc/cli_frame.h and
# Core/Inc/can_cli_bridge.h exactly.
# ============================================================================

CANCLI_ID_REQ = 0x1FF907
CANCLI_ID_RESP = 0x1FF908
DEFAULT_CHANNEL = "PCAN_USBBUS1"
DEFAULT_BITRATE = 500000  # matches FDCAN_BAUD_RATE in fdcan.c

DEFAULT_THEME = "darkly"
THEME_CHOICES = [
    "darkly", "solarized-dark", "one-dark", "nord-dark", "dracula-dark",
    "vapor-dark", "bootstrap-dark", "tokyo-night-dark",
    "bootstrap-light", "sandstone-light", "united-light",
]

# --- Bus-politeness tuning (prevents flooding the BMS and starving charger/display) ---
FRAME_DELAY_S = 0.005       # 5 ms between ISO-TP multi-frame segments
CMD_SPACING_S = 0.05        # 50 ms between distinct commands (manual send spacing)


class Cmd:
    """CLI_CMD_* -- see Core/Inc/cli_frame.h. Only what this GUI uses."""
    SOC_DATA = 0x01
    TEMP_DATA = 0x03
    FAULT_STATUS = 0x04
    RTC_DATA = 0x05
    ERROR_LOG_DATA = 0x06
    TEMP2_DATA = 0x07
    CELL_EXTREMA = 0x08
    BMS_LIFE = 0x0A
    CELL_VOLT_ALL = 0x0C

    ZERO_CAL = 0x10
    KNOWN_CAL = 0x11
    SOC_SET_FULL = 0x12
    SOC_SET_EMPTY = 0x13
    SAVE_CAL = 0x14
    SET_TEMP_CAL = 0x15
    SET_TEMP2_CAL = 0x16

    SET_FAULT_THRESH = 0x20
    SET_SOC_CONFIG = 0x21
    SET_CHARGER_CONFIG = 0x22
    GET_CHARGER_CONFIG = 0x23
    CHARGER_CONFIG_DATA = 0x24
    CHARGER_STATUS_DATA = 0x25
    CAN_HEALTH_DATA = 0x26
    GET_SOC_CONFIG = 0x27
    SOC_CONFIG_DATA = 0x28

    SET_RTC = 0x30
    READ_ERROR_LOGS = 0x31
    CLEAR_ERROR_LOGS = 0x32
    ERROR_LOG_CLEARED = 0x33

    SET_LC_ZERO = 0x51
    SET_SOH = 0x52
    RESET_LIFE = 0x53
    CLEAR_FAULTS = 0x54
    RESET_STATE = 0x55

    SET_DEVICE_IDS = 0x60
    GET_DEVICE_IDS = 0x61
    DEVICE_IDS_DATA = 0x62
    SET_CHARGER_OVERRIDE = 0x63
    GET_CHARGER_OVERRIDE = 0x64
    CHARGER_OVERRIDE_DATA = 0x65
    CAN_PING = 0x66

    LC_DATA = 0x50


FAULT_BIT_NAMES = {
    # Must match Core/Inc/fault_detector.h's FAULT_BIT_* bit positions exactly.
    # (Previous version had bit 10 mislabeled "CHG_OC" -- that's actually UT2 --
    # and was missing bits 11-13 entirely.)
    0: "OV", 1: "UV", 2: "OC(dischg)", 3: "CELL_DIFF", 4: "OT1", 5: "UT1",
    6: "THERM1_MISSING", 7: "OT2", 8: "CELL_OV", 9: "CELL_UV", 10: "UT2",
    11: "THERM2_MISSING", 12: "CHARGE_OC", 13: "CURRENT_SENSOR_FAULT",
}
FAULT_BIT_COUNT = 14  # number of bits actually defined above

CHARGER_STATE_NAMES = {
    0: "DISCONNECTED", 1: "READY", 2: "CHARGING", 3: "FAULT",
    4: "INHIBITED", 5: "TIMEOUT",
}

CHARGER_STATUS_BIT_CV_PHASE = 0x01
CHARGER_STATUS_BIT_CHARGE_DONE = 0x02
CHARGER_STATUS_BIT_TEMP_BLOCKED = 0x04
CHARGER_STATUS_BIT_HOST_STOPPED = 0x08
CHARGER_STATUS_BIT_STORAGE_MODE = 0x10

# ============================================================================
# ISO-TP-lite framing -- exact mirror of Core/Src/can_cli_bridge.c
# ============================================================================


class CanCliCodec:
    """Encode (cmd, data) into a list of <=8-byte CAN payloads."""

    @staticmethod
    def encode(cmd: int, data: bytes) -> list:
        vp = bytes([cmd]) + data
        total = len(vp)
        if total > 199:
            raise ValueError(f"payload too large for tunnel: {total} bytes (max 199)")

        frames = []
        if total <= 7:
            frames.append(bytes([0x00 | total]) + vp)
            return frames

        # First Frame: PCI(2 bytes) + first 6 bytes of VP
        frames.append(bytes([0x10 | ((total >> 8) & 0x0F), total & 0xFF]) + vp[0:6])
        sent = 6
        seq = 1
        while sent < total:
            chunk = vp[sent:sent + 7]
            frames.append(bytes([0x20 | seq]) + chunk)
            sent += len(chunk)
            seq = (seq + 1) & 0x0F
        return frames


class ReassemblyStats:
    def __init__(self):
        self.frames_rx = 0
        self.msgs_ok = 0
        self.rej_seq = 0
        self.rej_len = 0
        self.rej_orphan_cf = 0


class CanCliReassembler:
    """Single-session SF/FF/CF reassembler -- mirrors can_cli_bridge.c's
    CanCli_RxFrame() byte-for-byte. feed() returns (cmd, data) once a
    message completes, else None."""

    def __init__(self):
        self.stats = ReassemblyStats()
        self._active = False
        self._total_len = 0
        self._got_len = 0
        self._expected_seq = 0
        self._buf = bytearray()

    def feed(self, payload: bytes):
        if not payload:
            return None
        self.stats.frames_rx += 1
        pci = payload[0]
        ftype = pci >> 4

        if ftype == 0x0:
            l = pci & 0x0F
            if l == 0 or l > 7 or l > len(payload) - 1:
                self.stats.rej_len += 1
                self._active = False
                return None
            vp = payload[1:1 + l]
            self._active = False
            self.stats.msgs_ok += 1
            return vp[0], bytes(vp[1:])

        if ftype == 0x1:
            if len(payload) < 3:
                self.stats.rej_len += 1
                self._active = False
                return None
            l = ((pci & 0x0F) << 8) | payload[1]
            if l < 8 or l > 199:
                self.stats.rej_len += 1
                self._active = False
                return None
            n = min(len(payload) - 2, 6)
            self._buf = bytearray(payload[2:2 + n])
            self._total_len = l
            self._got_len = n
            self._expected_seq = 1
            self._active = True
            return None

        if ftype == 0x2:
            if not self._active:
                self.stats.rej_orphan_cf += 1
                return None
            seq = pci & 0x0F
            if seq != self._expected_seq:
                self.stats.rej_seq += 1
                self._active = False
                return None
            remain = self._total_len - self._got_len
            n = min(len(payload) - 1, 7, remain)
            self._buf += payload[1:1 + n]
            self._got_len += n
            self._expected_seq = (self._expected_seq + 1) & 0x0F
            if self._got_len >= self._total_len:
                self._active = False
                self.stats.msgs_ok += 1
                return self._buf[0], bytes(self._buf[1:])
            return None

        return None  # unknown frame type -- ignored, not a rejection


# ============================================================================
# Struct decoders -- byte layouts per Core/Inc/cli_frame.h,
# charger_config.h, fault_detector.h, error_log_store.h, lc_estimator.h,
# soh_estimator.h. All little-endian, all as actually sent (no implicit
# struct padding -- firmware builds every one of these byte-by-byte).
# ============================================================================

FAULT_THRESH_FIELDS = (
    "ovVoltage_V", "uvVoltage_V", "ocCurrent_A", "cellDiff_mV",
    "otTemp_C", "utTemp_C", "cellMax_V", "cellMin_V", "chargeOCLimit_A",
)

# ============================================================================
# CSV data logging -- column schema for the "Data Logging" panel on the
# Live Data tab. First three columns are always filled in at write time
# (see BmsCanGui._log_tick()); every column after that is looked up by
# exact key match in self._log_cache, which _handle_decoded() keeps
# updated from whichever telemetry command last touched that value. A
# missing key (nothing received yet since connecting) logs as "".
# ============================================================================

LOG_CSV_COLUMNS = (
    "date", "time", "epoch_s",
    "pack_v", "current_a", "soc_percent", "remaining_ah", "remaining_wh",
    "temp1_c", "temp2_c",
    "cell_min_v", "cell_max_v", "cell_diff_mV",
    *[f"cell_{i}_v" for i in range(16)],
    "soh_percent", "cycles", "life_years",
    "active_faults", "fault_mask_hex",
    "charger_state", "charger_comm_ok", "charger_actual_v", "charger_actual_i",
    "charger_cmd_v", "charger_cmd_i", "charger_cv_phase", "charger_charge_done",
    "charger_temp_blocked", "charger_host_stopped",
    "bus_off_recoveries", "error_passive_seconds",
)


def decode_soc_data(d):
    soc, ah, i, v = struct.unpack_from("<4f", d)
    return {"soc_percent": soc, "remaining_ah": ah, "current_a": i, "pack_v": v}


def decode_fault_status(d):
    mask = struct.unpack_from("<H", d)[0]
    vals = struct.unpack_from("<9f", d, 2)
    flags = [FAULT_BIT_NAMES[b] for b in range(FAULT_BIT_COUNT) if (mask >> b) & 1]
    return {"mask": mask, "flags": flags,
            "thresholds": dict(zip(FAULT_THRESH_FIELDS, vals))}


def decode_cell_extrema(d):
    # Wire values are millivolts, not volts -- Core/Inc/daly_bms.h documents
    # minCellmV/maxCellmV as "raw from 0x95, before / 1000" and main.c sends
    # them as-is with no conversion. Convert once here so every consumer
    # below (the Cell Delta V card, CSV cell_min_v/max_v/diff_mV) is
    # correctly scaled instead of treating millivolt-sized numbers as volts.
    min_idx, min_v_mv, max_idx, max_v_mv = struct.unpack_from("<BfBf", d)
    return {"min_idx": min_idx, "min_v": min_v_mv / 1000.0,
            "max_idx": max_idx, "max_v": max_v_mv / 1000.0}


def decode_cell_volt_all(d):
    # Same millivolt-on-the-wire convention as decode_cell_extrema() above.
    n = d[0]
    vals_mv = struct.unpack_from(f"<{n}f", d, 1)
    return {"cells": [v / 1000.0 for v in vals_mv]}


def decode_charger_config(d):
    # CLI_ChargerConfig_t is now <4f2BH> (20 bytes) -- storageVoltageSetpoint_mV
    # replaces what used to be 2 padding bytes, same total size.
    cellv, ichg, itaper, soctaper, cells, enable, storage_mv = struct.unpack_from("<4f2BH", d)
    return {"cellVoltageSetpoint_V": cellv, "chargeCurrentFull_A": ichg,
            "cvCutoffCurrent_A": itaper, "socTaperStart_pct": soctaper,
            "cellCount": cells, "chargeEnable": enable,
            "storageVoltageSetpoint_V": storage_mv / 1000.0}


SOC_OCV_POINTS = 11  # must match Core/Inc/soc.h's SOC_OCV_POINTS exactly


def decode_soc_config(d):
    # CLI_SOC_Config_Payload_t: pack_uv_limit, battery_capacity_ah,
    # pack_full_voltage (3 floats), then soc_ref[11], pack_ocv[11] -- 100
    # bytes total, all little-endian float32.
    vals = struct.unpack_from(f"<3f{SOC_OCV_POINTS}f{SOC_OCV_POINTS}f", d)
    pack_uv_limit, battery_capacity_ah, pack_full_voltage = vals[0:3]
    soc_ref = list(vals[3:3 + SOC_OCV_POINTS])
    pack_ocv = list(vals[3 + SOC_OCV_POINTS:3 + 2 * SOC_OCV_POINTS])
    return {
        "pack_uv_limit": pack_uv_limit,
        "battery_capacity_ah": battery_capacity_ah,
        "pack_full_voltage": pack_full_voltage,
        "soc_ref": soc_ref,
        "pack_ocv": pack_ocv,
    }


def decode_charger_status(d):
    state, comm_ok, flags, bits = struct.unpack_from("<4B", d)
    av, ai, cv, ci = struct.unpack_from("<4f", d, 4)
    return {
        "state": state, "state_name": CHARGER_STATE_NAMES.get(state, "?"),
        "comm_ok": bool(comm_ok), "raw_flags": flags,
        "cv_phase": bool(bits & CHARGER_STATUS_BIT_CV_PHASE),
        "charge_done": bool(bits & CHARGER_STATUS_BIT_CHARGE_DONE),
        "temp_blocked": bool(bits & CHARGER_STATUS_BIT_TEMP_BLOCKED),
        "host_stopped": bool(bits & CHARGER_STATUS_BIT_HOST_STOPPED),
        "storage_mode": bool(bits & CHARGER_STATUS_BIT_STORAGE_MODE),
        "actual_v": av, "actual_i": ai, "cmd_v": cv, "cmd_i": ci,
    }


def decode_can_health(d):
    recov, errpass_s = struct.unpack_from("<2I", d)
    return {"bus_off_recoveries": recov, "error_passive_seconds": errpass_s}


def decode_rtc(d):
    y, mo, day, h, mi, s = struct.unpack_from("<6B", d)
    return {"text": f"20{y:02d}-{mo:02d}-{day:02d} {h:02d}:{mi:02d}:{s:02d}"}


SEVERITY_NAMES = {0: "INFO", 1: "WARNING", 2: "CRITICAL"}
SEVERITY_BOOTSTYLE = {0: SECONDARY, 1: WARNING, 2: DANGER}


def decode_error_log(d):
    y, mo, day, h, mi, s = struct.unpack_from("<6B", d)
    seq, fault_bits, severity, _reserved = struct.unpack_from("<IHBB", d, 6)
    msg = d[14:14 + 58].split(b"\x00", 1)[0].decode("ascii", "replace")
    fault_names = [FAULT_BIT_NAMES[b] for b in range(FAULT_BIT_COUNT) if (fault_bits >> b) & 1]
    return {
        "time": f"20{y:02d}-{mo:02d}-{day:02d} {h:02d}:{mi:02d}:{s:02d}",
        "seq": seq,
        "fault_bits": fault_bits,
        "fault_names": ", ".join(fault_names) if fault_names else "",
        "severity": severity,
        "severity_name": SEVERITY_NAMES.get(severity, f"?{severity}"),
        "message": msg,
    }


def decode_lc_data(d):
    lc_pct, wh, secs, conf, phase = struct.unpack_from("<4fB", d)
    return {"lc_percent": lc_pct, "remaining_wh": wh,
            "remaining_time_s": secs, "confidence": conf, "phase": phase}


def decode_bms_life(d):
    soh, cycles, life_years, ah_cycled = struct.unpack_from("<fHfI", d)
    return {"soh_percent": soh, "cycles": cycles,
            "life_years": life_years, "total_ah_cycled": ah_cycled}


def decode_device_ids(d):
    # Wire layout must match Core/Inc/cli_frame.h's CLI_DeviceIdentity_t
    # exactly -- nodeId (DroneCAN local node ID) was appended after
    # mfgDate, same append-at-the-end convention every prior field on
    # this struct followed (52 bytes total now, was 48).
    bms_sn, pack_sn, pid, mfg_date, node_id = struct.unpack_from("<16s16sI12sI", d)
    return {"bms_serial": bms_sn.split(b"\x00", 1)[0].decode("ascii", "replace"),
            "pack_serial": pack_sn.split(b"\x00", 1)[0].decode("ascii", "replace"),
            "product_id": pid,
            "mfg_date": mfg_date.split(b"\x00", 1)[0].decode("ascii", "replace"),
            "node_id": node_id}


def decode_charger_override(d):
    return {"force_stop": bool(d[0])}


def decode_error_log_cleared(d):
    return {"ok": bool(d[0])}


DECODERS = {
    Cmd.SOC_DATA: decode_soc_data,
    Cmd.TEMP_DATA: lambda d: {"temp_c": struct.unpack_from("<f", d)[0]},
    Cmd.TEMP2_DATA: lambda d: {"temp2_c": struct.unpack_from("<f", d)[0]},
    Cmd.FAULT_STATUS: decode_fault_status,
    Cmd.RTC_DATA: decode_rtc,
    Cmd.ERROR_LOG_DATA: decode_error_log,
    Cmd.CELL_EXTREMA: decode_cell_extrema,
    Cmd.CELL_VOLT_ALL: decode_cell_volt_all,
    Cmd.CHARGER_CONFIG_DATA: decode_charger_config,
    Cmd.SOC_CONFIG_DATA: decode_soc_config,
    Cmd.CHARGER_STATUS_DATA: decode_charger_status,
    Cmd.CAN_HEALTH_DATA: decode_can_health,
    Cmd.LC_DATA: decode_lc_data,
    Cmd.BMS_LIFE: decode_bms_life,
    Cmd.DEVICE_IDS_DATA: decode_device_ids,
    Cmd.CHARGER_OVERRIDE_DATA: decode_charger_override,
    Cmd.ERROR_LOG_CLEARED: decode_error_log_cleared,
    Cmd.CAN_PING: lambda d: {"raw": d.hex()},
}

# ---------------------------------------------------------------------------
# Fixed-ID broadcast decoders (passive-mode fallback -- same wire layouts as
# bms_passive_monitor.py).  These IDs are unconditional BMS broadcasts; they
# do NOT require an active CanCli client.
# ---------------------------------------------------------------------------

CHARGER_TX_ID_CMD = 0x1806E5F4
CHARGER_RX_ID_STATUS = 0x18FF50E5
DISPLAY_ID_CELL_DELTAV = 0x1FF710
DISPLAY_ID_OVRVIEW = 0x1FF810
DISPLAY_ID_VIT = 0x1FF820

CHARGER_CONTROL_NAMES = {0: "ENABLED/START", 1: "DISABLED/STOP"}
CHARGING_STATUS_NAMES = {0: "Idle", 1: "Charging", 2: "Fault", 3: "Full"}


def _decode_charger_command(d):
    v_raw, i_raw = struct.unpack_from(">HH", d, 0)
    ctrl = d[4] if len(d) > 4 else None
    return {
        "voltage_setpoint": v_raw / 10.0,
        "current_setpoint": i_raw / 10.0,
        "control": CHARGER_CONTROL_NAMES.get(ctrl, f"?{ctrl}"),
    }


def _decode_charger_status(d):
    v_raw, i_raw = struct.unpack_from(">HH", d, 0)
    flags = d[4] if len(d) > 4 else 0
    return {
        "actual_voltage": v_raw / 10.0,
        "actual_current": i_raw / 10.0,
        "fault_code": flags & 0x07,
        "inhibited": bool(flags & 0x08),
    }


def _decode_bms_vit(d):
    v_raw, i_raw, t1_raw, t2_raw = struct.unpack_from("<Hhhh", d, 0)
    return {
        "pack_voltage": v_raw * 0.001,
        "pack_current": i_raw * 0.01,
        "temp1": t1_raw * 0.01,
        "temp2": t2_raw * 0.01,
    }


def _decode_bms_ovrview(d):
    soc, soh = d[0], d[1]
    cycles, remcap_raw = struct.unpack_from("<HH", d, 2)
    charging_status = d[6] if len(d) > 6 else None
    charging_cmd = d[7] if len(d) > 7 else None
    return {
        "soc": soc, "soh": soh, "cycles": cycles,
        "remaining_capacity": remcap_raw * 0.001,
        "charging_status": CHARGING_STATUS_NAMES.get(charging_status, f"?{charging_status}"),
        "charging_cmd": "On" if charging_cmd else "Off",
    }


def _decode_cell_deltav(d):
    delta, minv, maxv = struct.unpack_from("<HHH", d, 0)
    return {"delta_v": delta * 0.001, "min_cell_v": minv * 0.001,
            "max_cell_v": maxv * 0.001}


FIXED_ID_DECODERS = {
    CHARGER_TX_ID_CMD: ("Charger_Command", _decode_charger_command),
    CHARGER_RX_ID_STATUS: ("Charger_Status", _decode_charger_status),
    DISPLAY_ID_VIT: ("BMS_VIT", _decode_bms_vit),
    DISPLAY_ID_OVRVIEW: ("BMS_OvrView", _decode_bms_ovrview),
    DISPLAY_ID_CELL_DELTAV: ("Cell_DeltaV", _decode_cell_deltav),
}



# ============================================================================
# CAN client
# ============================================================================

@dataclass
class ClientStats:
    tx_frames: int = 0
    rx_frames: int = 0
    rx_msgs: int = 0
    rx_errors: int = 0
    last_rx_tick: float = 0.0


class BmsCanClient:
    """Owns the python-can Bus and a background RX thread.
    Transmits are purely synchronous (no background sender / ping threads)
    so the bus stays silent except when the user explicitly sends a command."""

    def __init__(self, on_error: Optional[Callable[[str], None]] = None):
        self.bus: Optional["can.BusABC"] = None
        self.rx_queue: "queue.Queue" = queue.Queue()
        self.stats = ClientStats()
        self._reassembler = CanCliReassembler()
        self._stop = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None
        self._send_lock = threading.Lock()
        self._on_error = on_error
        self._tx_enabled = False      # simple gate: must be True to transmit
        self._last_tx_time = 0.0      # for polite inter-command spacing
        self._last_tunnel_rx = 0.0    # timestamp of last CANCLI_ID_RESP frame

    @property
    def connected(self) -> bool:
        return self.bus is not None

    def connect(self, channel: str, bitrate: int):
        self.bus = can.Bus(interface="pcan", channel=channel, bitrate=bitrate)
        self._stop.clear()
        self._tx_enabled = False
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def disconnect(self):
        self._stop.set()
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass
        self.bus = None
        self._tx_enabled = False

    @property
    def tx_enabled(self) -> bool:
        return self._tx_enabled

    def set_tx_enabled(self, enabled: bool):
        self._tx_enabled = enabled

    @property
    def tunnel_alive(self) -> bool:
        """True if we have seen a CanCli tunnel frame in the last 3.5 s."""
        return (time.time() - self._last_tunnel_rx) < 3.5

    # ---- low-level send/receive -------------------------------------------

    def _send_raw(self, cmd: int, data: bytes = b""):
        """Synchronous low-level send with inter-frame and inter-command spacing."""
        if not self.bus:
            raise RuntimeError("not connected")
        if not self._tx_enabled:
            raise RuntimeError("TX is locked -- enable transmission first")
        with self._send_lock:
            # polite spacing between distinct commands
            elapsed = time.time() - self._last_tx_time
            if elapsed < CMD_SPACING_S:
                time.sleep(CMD_SPACING_S - elapsed)
            frames = CanCliCodec.encode(cmd, data)
            for i, frame in enumerate(frames):
                msg = can.Message(arbitration_id=CANCLI_ID_REQ, data=frame,
                                   is_extended_id=True)
                self.bus.send(msg)
                self.stats.tx_frames += 1
                if i < len(frames) - 1:
                    time.sleep(FRAME_DELAY_S)
            self._last_tx_time = time.time()

    def send(self, cmd: int, data: bytes = b""):
        """Synchronous transmit -- blocks briefly while sending."""
        self._send_raw(cmd, data)

    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                msg = self.bus.recv(timeout=0.2)
            except Exception as exc:
                if self._on_error:
                    self._on_error(f"CAN RX error: {exc}")
                time.sleep(0.5)
                continue
            if msg is None:
                continue
            self.stats.rx_frames += 1
            self.stats.last_rx_tick = time.time()

            # --- CanCli tunnel (0x1FF908) --------------------------------
            if msg.arbitration_id == CANCLI_ID_RESP:
                self._last_tunnel_rx = time.time()
                result = self._reassembler.feed(bytes(msg.data))
                if result is not None:
                    cmd, payload = result
                    self.stats.rx_msgs += 1
                    decoder = DECODERS.get(cmd)
                    try:
                        decoded = decoder(payload) if decoder else {"raw": payload.hex()}
                    except Exception as exc:
                        self.stats.rx_errors += 1
                        decoded = {"decode_error": str(exc), "raw": payload.hex()}
                    self.rx_queue.put((cmd, decoded))
                continue

            # --- Fixed-ID unconditional broadcasts (always readable)
            if msg.arbitration_id in FIXED_ID_DECODERS:
                name, decoder = FIXED_ID_DECODERS[msg.arbitration_id]
                try:
                    decoded = decoder(msg.data)
                except Exception as exc:
                    decoded = {"decode_error": str(exc)}
                # Use a synthetic negative cmd key so _handle_decoded can route it
                self.rx_queue.put((-msg.arbitration_id, decoded))
                continue

    # ---- convenience command methods --------------------------------------

    def get_device_ids(self):
        self.send(Cmd.GET_DEVICE_IDS)

    def set_device_ids(self, bms_serial: str, pack_serial: str, product_id: int,
                        mfg_date: str = "", node_id: int = 0):
        payload = struct.pack("<16s16sI12sI",
                               bms_serial.encode("ascii", "replace")[:15],
                               pack_serial.encode("ascii", "replace")[:15],
                               product_id & 0xFFFFFFFF,
                               mfg_date.encode("ascii", "replace")[:11],
                               node_id & 0xFFFFFFFF)
        self.send(Cmd.SET_DEVICE_IDS, payload)

    def get_charger_config(self):
        self.send(Cmd.GET_CHARGER_CONFIG)

    def set_charger_config(self, cell_v, ichg_a, itaper_a, cell_count, storage_v):
        storage_mv = int(round(storage_v * 1000.0)) & 0xFFFF
        payload = struct.pack("<4f2BH", cell_v, ichg_a, itaper_a, 0.0,
                               cell_count, 1, storage_mv)
        self.send(Cmd.SET_CHARGER_CONFIG, payload)

    def get_soc_config(self):
        self.send(Cmd.GET_SOC_CONFIG)

    def set_soc_config(self, pack_uv_limit, battery_capacity_ah,
                        pack_full_voltage, soc_ref, pack_ocv):
        """soc_ref/pack_ocv must each have exactly SOC_OCV_POINTS (11)
        values -- the OCV-vs-SOC lookup curve the firmware's SOC estimator
        interpolates against. See CLI_SOC_Config_Payload_t (cli_frame.h)."""
        if len(soc_ref) != SOC_OCV_POINTS or len(pack_ocv) != SOC_OCV_POINTS:
            raise ValueError(f"soc_ref/pack_ocv must each have exactly {SOC_OCV_POINTS} points")
        payload = struct.pack(f"<3f{SOC_OCV_POINTS}f{SOC_OCV_POINTS}f",
                               pack_uv_limit, battery_capacity_ah, pack_full_voltage,
                               *soc_ref, *pack_ocv)
        self.send(Cmd.SET_SOC_CONFIG, payload)

    def set_charger_override(self, force_stop: bool):
        self.send(Cmd.SET_CHARGER_OVERRIDE, bytes([1 if force_stop else 0]))

    def get_charger_override(self):
        self.send(Cmd.GET_CHARGER_OVERRIDE)

    def set_fault_thresholds(self, values: dict):
        payload = struct.pack("<9f", *(values[k] for k in FAULT_THRESH_FIELDS))
        self.send(Cmd.SET_FAULT_THRESH, payload)

    def zero_cal(self):
        self.send(Cmd.ZERO_CAL)

    def known_cal(self, known_current_ma: float):
        self.send(Cmd.KNOWN_CAL, struct.pack("<f", known_current_ma))

    def set_temp_cal(self, known_temp_c: float):
        self.send(Cmd.SET_TEMP_CAL, struct.pack("<f", known_temp_c))

    def set_temp2_cal(self, known_temp_c: float):
        self.send(Cmd.SET_TEMP2_CAL, struct.pack("<f", known_temp_c))

    def soc_set_full(self):
        self.send(Cmd.SOC_SET_FULL)

    def soc_set_empty(self):
        self.send(Cmd.SOC_SET_EMPTY)

    def set_soh(self, soh_pct: float):
        self.send(Cmd.SET_SOH, struct.pack("<f", soh_pct))

    def lc_zero(self):
        self.send(Cmd.SET_LC_ZERO)

    def reset_life(self):
        self.send(Cmd.RESET_LIFE)

    def clear_faults(self):
        self.send(Cmd.CLEAR_FAULTS)

    def reset_state(self):
        self.send(Cmd.RESET_STATE)

    def save_to_flash(self):
        self.send(Cmd.SAVE_CAL)

    def read_error_logs(self, offset: int = 0):
        self.send(Cmd.READ_ERROR_LOGS, struct.pack("<I", offset))

    def clear_error_logs(self):
        self.send(Cmd.CLEAR_ERROR_LOGS)

    def set_rtc_now(self):
        t = time.localtime()
        payload = bytes([t.tm_year - 2000, t.tm_mon, t.tm_mday,
                          t.tm_hour, t.tm_min, t.tm_sec])
        self.send(Cmd.SET_RTC, payload)




# ============================================================================
# GUI  (ttkbootstrap -- card-based layout, live theme switching)
# ============================================================================

CHARGER_STATE_BOOTSTYLE = {
    0: SECONDARY,  # DISCONNECTED
    1: SUCCESS,    # READY
    2: INFO,       # CHARGING
    3: DANGER,     # FAULT
    4: WARNING,    # INHIBITED
    5: WARNING,    # TIMEOUT
}


def _soc_bootstyle(pct: float) -> str:
    if pct < 20:
        return DANGER
    if pct < 50:
        return WARNING
    return SUCCESS


class StatCard(ttk.Frame):
    """Small labelled-value tile -- 'label' above a big bold 'value',
    optional units. Used everywhere a Meter's fixed 0-100 range doesn't fit
    the quantity (pack V, current, Ah, temps -- all config/chemistry
    dependent ranges)."""

    def __init__(self, master, label: str, unit: str = "", bootstyle=SECONDARY, **kw):
        super().__init__(master, bootstyle=bootstyle, padding=(12, 8), **kw)
        self.var = tk.StringVar(value="--")
        ttk.Label(self, text=label, bootstyle=(bootstyle, "inverse"),
                  font=("TkDefaultFont", 9)).pack(anchor="w")
        row = ttk.Frame(self, bootstyle=bootstyle)
        row.pack(anchor="w", fill="x")
        ttk.Label(row, textvariable=self.var, bootstyle=(bootstyle, "inverse"),
                  font=("TkDefaultFont", 18, "bold")).pack(side="left")
        if unit:
            ttk.Label(row, text=" " + unit, bootstyle=(bootstyle, "inverse"),
                      font=("TkDefaultFont", 10)).pack(side="left", anchor="s")

    def set(self, text: str):
        self.var.set(text)

    def set_bootstyle(self, bootstyle):
        self.configure(bootstyle=bootstyle)
        for child in self.winfo_children():
            try:
                child.configure(bootstyle=bootstyle if not isinstance(child, ttk.Frame)
                                 else bootstyle)
            except tk.TclError:
                pass


class Badge(ttk.Label):
    """Small solid pill-style status indicator (inverse bootstyle)."""

    def __init__(self, master, text="--", bootstyle=SECONDARY, **kw):
        super().__init__(master, text=text, bootstyle=(bootstyle, "inverse"),
                          padding=(10, 4), **kw)

    def set(self, text: str, bootstyle):
        self.configure(text=text, bootstyle=(bootstyle, "inverse"))


class BmsCanGui(ttk.Window):
    def __init__(self):
        super().__init__(title="Dreamfly Eionic BMS -- CAN GUI (PEAK-CAN)",
                          themename=DEFAULT_THEME, size=(1180, 780),
                          resizable=(True, True))
        self.minsize(980, 640)

        self.client = BmsCanClient(on_error=self._log_error)

        # Keys in FAULT_THRESH_FIELDS the user is currently editing in the
        # Fault Thresholds tab. Live FAULT_STATUS telemetry (arriving every
        # ~100ms via the CanCli keepalive mirror)
        # must NOT overwrite a field while it's dirty, or every keystroke gets
        # stomped by the next telemetry frame before Apply can ever be
        # pressed -- see _handle_decoded()/FAULT_STATUS and _build_thresh_tab().
        self._thresh_dirty: set = set()

        # ---- CSV data logging --------------------------------------------
        self._log_cache: dict = {}        # latest known live values, keyed
                                           # to match LOG_CSV_COLUMNS; kept
                                           # fresh in _handle_decoded()
        self._log_path: Optional[str] = None
        self._log_fh = None               # open file handle while logging
        self._log_writer = None           # csv.writer bound to _log_fh
        self._log_rows_written = 0
        self._log_after_id = None         # self.after() handle for the
                                           # periodic tick, so it can be
                                           # cancelled cleanly on stop/close

        # ---- Error-log pagination (CLI_CMD_READ_ERROR_LOGS fetch-all) -----
        self._log_fetch_in_progress = False
        self._log_fetch_offset = 0
        self._log_fetch_page_count = 0
        self._log_fetch_total = 0
        self._log_fetch_pages_requested = 0

        self._build_header()
        self._build_tabs()
        self._build_status_bar()

        self.after(100, self._poll_rx_queue)

    # ---- header: title + theme picker + connection card -------------------

    def _build_header(self):
        header = ttk.Frame(self, padding=(12, 10))
        header.pack(side="top", fill="x")

        title_row = ttk.Frame(header)
        title_row.pack(fill="x")
        ttk.Label(title_row, text="\u26A1 Dreamfly Eionic BMS",
                  font=("TkDefaultFont", 16, "bold")).pack(side="left")
        ttk.Label(title_row, text="  CAN 2.0B Extended \u00b7 PEAK-CAN",
                  bootstyle=SECONDARY).pack(side="left", padx=(4, 0))

        ttk.Label(title_row, text="Theme:").pack(side="right", padx=(0, 4))
        self.theme_var = tk.StringVar(value=DEFAULT_THEME)
        theme_box = ttk.Combobox(title_row, textvariable=self.theme_var, width=16,
                                  values=THEME_CHOICES, state="readonly")
        theme_box.pack(side="right")
        theme_box.bind("<<ComboboxSelected>>", lambda e: self.style.theme_use(self.theme_var.get()))

        conn = ttk.Labelframe(header, text="Connection", padding=10, bootstyle=PRIMARY)
        conn.pack(fill="x", pady=(10, 0))

        ttk.Label(conn, text="Channel").grid(row=0, column=0, sticky="w", padx=(0, 4))
        self.channel_var = tk.StringVar(value=DEFAULT_CHANNEL)
        ttk.Entry(conn, textvariable=self.channel_var, width=16).grid(row=0, column=1, padx=(0, 16))

        ttk.Label(conn, text="Bitrate").grid(row=0, column=2, sticky="w", padx=(0, 4))
        self.bitrate_var = tk.StringVar(value=str(DEFAULT_BITRATE))
        ttk.Combobox(conn, textvariable=self.bitrate_var, width=10,
                     values=["250000", "500000", "1000000"]).grid(row=0, column=3, padx=(0, 16))

        self.connect_btn = ttk.Button(conn, text="Connect", bootstyle=SUCCESS,
                                       command=self._toggle_connect)
        self.connect_btn.grid(row=0, column=4, padx=(0, 16))

        self.conn_badge = Badge(conn, text="DISCONNECTED", bootstyle=DANGER)
        self.conn_badge.grid(row=0, column=5, sticky="w", padx=(0, 12))

        # --- TX Lock control --------------------------------------------------
        tx_frame = ttk.Frame(conn)
        tx_frame.grid(row=0, column=6, sticky="w")
        self.tx_badge = Badge(tx_frame, text="TX LOCKED", bootstyle=DANGER)
        self.tx_badge.pack(side="left")
        self.tx_toggle_btn = ttk.Button(
            tx_frame, text="Unlock TX", bootstyle=PRIMARY,
            command=self._toggle_tx_lock, width=10)
        self.tx_toggle_btn.pack(side="left", padx=(6, 0))

    # ---- tabs -----------------------------------------------------------

    def _build_tabs(self):
        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=10, pady=(4, 10))

        self.tab_live = ttk.Frame(nb, padding=10)
        self.tab_thresh = ttk.Frame(nb, padding=10)
        self.tab_cal = ttk.Frame(nb, padding=10)
        self.tab_soc_config = ttk.Frame(nb, padding=10)
        self.tab_charger = ttk.Frame(nb, padding=10)
        self.tab_id = ttk.Frame(nb, padding=10)
        self.tab_logs = ttk.Frame(nb, padding=10)
        self.tab_diag = ttk.Frame(nb, padding=10)

        nb.add(self.tab_live, text="  Live Data  ")
        nb.add(self.tab_thresh, text="  Fault Thresholds  ")
        nb.add(self.tab_cal, text="  Calibration  ")
        nb.add(self.tab_soc_config, text="  SOC Configuration  ")
        nb.add(self.tab_charger, text="  Charger  ")
        nb.add(self.tab_id, text="  Device Identity  ")
        nb.add(self.tab_logs, text="  Error Logs  ")
        nb.add(self.tab_diag, text="  CAN Diagnostics  ")

        self._build_live_tab()
        self._build_thresh_tab()
        self._build_cal_tab()
        self._build_soc_config_tab()
        self._build_charger_tab()
        self._build_id_tab()
        self._build_logs_tab()
        self._build_diag_tab()

    # -- Live Data ----------------------------------------------------------

    def _build_live_tab(self):
        f = self.tab_live
        for c in range(4):
            f.columnconfigure(c, weight=1)

        gauges = ttk.Frame(f)
        gauges.grid(row=0, column=0, columnspan=4, sticky="ew", pady=(0, 10))
        gauges.columnconfigure((0, 1), weight=1)

        self.soc_meter = ttk.Meter(
            gauges, amount_used=0, amount_total=100, meter_type="semi",
            subtext="State of Charge", text_right="%", bootstyle=SECONDARY,
            metersize=180, stripethickness=8)
        self.soc_meter.grid(row=0, column=0, padx=20)

        self.soh_meter = ttk.Meter(
            gauges, amount_used=0, amount_total=100, meter_type="semi",
            subtext="State of Health", text_right="%", bootstyle=INFO,
            metersize=180, stripethickness=8)
        self.soh_meter.grid(row=0, column=1, padx=20)

        self.fault_banner = Badge(f, text="\u2713  No active faults", bootstyle=SUCCESS)
        self.fault_banner.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(0, 10))

        self.live_cards = {}
        card_defs = [
            ("packv", "Pack Voltage", "V", SECONDARY),
            ("current", "Current", "A", SECONDARY),
            ("ah", "Remaining Capacity", "Ah", SECONDARY),
            ("celldiff", "Cell Delta V", "V", SECONDARY),
            ("temp1", "Temp 1", "\u00b0C", SECONDARY),
            ("temp2", "Temp 2", "\u00b0C", SECONDARY),
            ("wh", "Remaining Energy", "Wh", SECONDARY),
            ("cycles", "Cycle Count", "", SECONDARY),
            ("life_years", "Life Remaining", "yr", SECONDARY),
            ("recov", "Bus-off Recoveries", "", SECONDARY),
            ("errpass", "Error-Passive Time", "s", SECONDARY),
        ]
        for i, (key, label, unit, style) in enumerate(card_defs):
            card = StatCard(f, label, unit, bootstyle=style)
            card.grid(row=2 + i // 4, column=i % 4, sticky="nsew", padx=4, pady=4)
            self.live_cards[key] = card

        cells = ttk.Labelframe(f, text="Cell voltages (V)", padding=10, bootstyle=PRIMARY)
        cells.grid(row=5, column=0, columnspan=4, sticky="nsew", pady=(10, 0))
        self.cell_badges = []
        for i in range(16):
            b = Badge(cells, text="--", bootstyle=SECONDARY)
            b.grid(row=i // 8, column=i % 8, padx=3, pady=3, sticky="nsew")
            self.cell_badges.append(b)
        self._min_cell_idx = None
        self._max_cell_idx = None

        self._build_logging_panel(f, row=6)

    def _build_logging_panel(self, parent, row):
        grp = ttk.Labelframe(parent, text="Data Logging (CSV)", padding=10, bootstyle=INFO)
        grp.grid(row=row, column=0, columnspan=4, sticky="ew", pady=(10, 0))
        grp.columnconfigure(1, weight=1)

        ttk.Label(grp, text="File:").grid(row=0, column=0, sticky="w", padx=(0, 6))
        self.log_path_var = tk.StringVar(value="(not set -- click Choose file)")
        ttk.Label(grp, textvariable=self.log_path_var, bootstyle=SECONDARY).grid(
            row=0, column=1, sticky="w")
        ttk.Button(grp, text="Choose file...", bootstyle=SECONDARY,
                   command=self._choose_log_file).grid(row=0, column=2, padx=6)

        ttk.Label(grp, text="Interval (s):").grid(row=0, column=3, sticky="e", padx=(12, 4))
        self.log_interval_var = tk.StringVar(value="1")
        ttk.Combobox(grp, textvariable=self.log_interval_var, width=5, state="readonly",
                     values=["0.5", "1", "2", "5", "10"]).grid(row=0, column=4)

        self.log_toggle_btn = ttk.Button(grp, text="Start Logging", bootstyle=SUCCESS,
                                          command=self._toggle_logging)
        self.log_toggle_btn.grid(row=0, column=5, padx=(12, 0))

        self.log_status_var = tk.StringVar(value="Not logging.")
        ttk.Label(grp, textvariable=self.log_status_var, bootstyle=SECONDARY).grid(
            row=1, column=0, columnspan=6, sticky="w", pady=(6, 0))

    def _choose_log_file(self):
        default_name = f"bms_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        path = filedialog.asksaveasfilename(
            title="Save BMS CSV log as...",
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if path:
            self._log_path = path
            self.log_path_var.set(path)

    def _toggle_logging(self):
        if self._log_fh is not None:
            self._stop_logging()
        else:
            self._start_logging()

    def _start_logging(self):
        if not self._log_path:
            self._choose_log_file()
            if not self._log_path:
                return  # user cancelled the file picker

        try:
            # Append if the chosen file already has rows (resume an existing
            # log); otherwise start fresh with a header row.
            file_is_new = (not os.path.exists(self._log_path)) or \
                           os.path.getsize(self._log_path) == 0
            self._log_fh = open(self._log_path, "a", newline="")
            self._log_writer = csv.writer(self._log_fh)
            if file_is_new:
                self._log_writer.writerow(LOG_CSV_COLUMNS)
                self._log_fh.flush()
        except OSError as exc:
            Messagebox.show_error(f"Could not open log file:\n{exc}", "Logging failed")
            self._log_fh = None
            self._log_writer = None
            return

        self._log_rows_written = 0
        self.log_toggle_btn.configure(text="Stop Logging", bootstyle=DANGER)
        self._log_tick()  # write the first row now, then keep going on the timer

    def _stop_logging(self):
        was_active = self._log_fh is not None
        if self._log_after_id is not None:
            self.after_cancel(self._log_after_id)
            self._log_after_id = None
        if self._log_fh is not None:
            try:
                self._log_fh.flush()
                self._log_fh.close()
            except OSError:
                pass
        self._log_fh = None
        self._log_writer = None
        self.log_toggle_btn.configure(text="Start Logging", bootstyle=SUCCESS)
        if was_active:
            self.log_status_var.set(
                f"Stopped -- {self._log_rows_written} rows written to {self._log_path}.")

    @staticmethod
    def _format_log_value(v):
        """Cap floats at 3 decimals (e.g. 4.2222222 -> '4.222') for the CSV
        log. Bools/ints/strings pass through unchanged -- bool is checked
        first since bool is a float-incompatible subclass of int in
        Python, not because of any real ambiguity here."""
        if isinstance(v, bool):
            return v
        if isinstance(v, float):
            return f"{v:.3f}"
        return v

    def _log_tick(self):
        if self._log_writer is not None:
            now = time.time()
            lt = time.localtime(now)
            row = [
                time.strftime("%Y-%m-%d", lt),
                time.strftime("%H:%M:%S", lt) + f".{int((now % 1) * 1000):03d}",
                f"{now:.3f}",
            ]
            row += [self._format_log_value(self._log_cache.get(col, ""))
                    for col in LOG_CSV_COLUMNS[3:]]
            try:
                self._log_writer.writerow(row)
                self._log_fh.flush()
                self._log_rows_written += 1
                self.log_status_var.set(
                    f"Logging to {self._log_path} -- {self._log_rows_written} rows written.")
            except OSError as exc:
                self._log_error(f"CSV log write failed, stopping: {exc}")
                self._stop_logging()
                return

            interval_ms = max(200, int(float(self.log_interval_var.get()) * 1000))
            self._log_after_id = self.after(interval_ms, self._log_tick)

    # -- Fault Thresholds -----------------------------------------------------

    def _build_thresh_tab(self):
        f = self.tab_thresh
        ttk.Label(f, text="Live values shown below auto-refresh from telemetry "
                          "while connected. Edit and Apply to write all nine at once.",
                  bootstyle=SECONDARY, wraplength=700, justify="left").grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 10))

        card = ttk.Labelframe(f, text="Thresholds", padding=12, bootstyle=PRIMARY)
        card.grid(row=1, column=0, sticky="nsew")

        self.thresh_vars = {}
        self.thresh_entries = {}
        labels = {
            "ovVoltage_V": "Pack OV (V)", "uvVoltage_V": "Pack UV (V)",
            "ocCurrent_A": "Discharge OC (A)", "cellDiff_mV": "Cell imbalance (mV)",
            "otTemp_C": "Over-temp (\u00b0C)", "utTemp_C": "Under-temp (\u00b0C)",
            "cellMax_V": "Cell OV (V)", "cellMin_V": "Cell UV (V)",
            "chargeOCLimit_A": "Charge OC (A)",
        }
        for i, key in enumerate(FAULT_THRESH_FIELDS):
            ttk.Label(card, text=labels[key]).grid(row=i, column=0, sticky="w", pady=3)
            var = tk.StringVar(value="--")
            self.thresh_vars[key] = var
            entry = ttk.Entry(card, textvariable=var, width=14)
            entry.grid(row=i, column=1, sticky="w", padx=8)
            # Mark dirty the moment the user touches this field (focus or a
            # keystroke) so the next incoming FAULT_STATUS telemetry frame
            # skips it instead of overwriting what they're typing/pasting.
            entry.bind("<FocusIn>", lambda e, k=key: self._thresh_dirty.add(k))
            entry.bind("<Key>", lambda e, k=key: self._thresh_dirty.add(k))
            self.thresh_entries[key] = entry

        btns = ttk.Frame(card)
        btns.grid(row=len(FAULT_THRESH_FIELDS), column=0, columnspan=2, pady=(12, 0), sticky="w")
        ttk.Button(btns, text="Apply", bootstyle=PRIMARY, command=self._apply_thresholds).pack(side="left", padx=4)
        ttk.Button(btns, text="Save to Flash", bootstyle=SECONDARY, command=self._save_to_flash).pack(side="left", padx=4)
        ttk.Button(btns, text="Refresh from BMS", bootstyle=SECONDARY,
                   command=self._refresh_thresholds_from_bms).pack(side="left", padx=4)

        side = ttk.Labelframe(f, text="Active faults", padding=12, bootstyle=DANGER)
        side.grid(row=1, column=1, sticky="nsew", padx=(12, 0))
        self.active_faults_var = tk.StringVar(value="--")
        ttk.Label(side, textvariable=self.active_faults_var, wraplength=280,
                  justify="left").pack(anchor="w")

    def _apply_thresholds(self):
        try:
            values = {k: float(self.thresh_vars[k].get()) for k in FAULT_THRESH_FIELDS}
        except ValueError:
            Messagebox.show_error("All threshold fields must be numbers.", "Invalid input")
            return
        self._guarded(lambda: self.client.set_fault_thresholds(values))
        # Sent (or attempted) -- let live FAULT_STATUS telemetry resume
        # driving these fields again, so the next frame confirms what the
        # firmware actually accepted (see ValidateFaultThresholds()'s
        # whole-struct reject-or-commit behavior on the firmware side).
        self._thresh_dirty.clear()

    def _refresh_thresholds_from_bms(self):
        """Abandon any in-progress edits and let live telemetry repopulate
        every field with the BMS's current values on the next FAULT_STATUS
        frame (no command sent -- FAULT_STATUS is already arriving
        continuously via the keepalive telemetry mirror; this just stops
        ignoring it for these fields)."""
        self._thresh_dirty.clear()
        self.focus()  # move focus off whichever Entry has it, if any

    # -- Calibration --------------------------------------------------------

    def _build_cal_tab(self):
        f = self.tab_cal
        for c in range(2):
            f.columnconfigure(c, weight=1)

        grp1 = ttk.Labelframe(f, text="Current sensor", padding=12, bootstyle=INFO)
        grp1.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)
        ttk.Button(grp1, text="Zero calibration", bootstyle=PRIMARY,
                   command=lambda: self._guarded(self.client.zero_cal)).grid(row=0, column=0, columnspan=2, pady=4, sticky="w")
        self.known_cal_var = tk.StringVar(value="1000")
        ttk.Label(grp1, text="Known current (mA)").grid(row=1, column=0, sticky="w")
        ttk.Entry(grp1, textvariable=self.known_cal_var, width=10).grid(row=1, column=1)
        ttk.Button(grp1, text="Apply known-current cal", bootstyle=PRIMARY,
                   command=lambda: self._guarded(lambda: self.client.known_cal(float(self.known_cal_var.get())))
                   ).grid(row=2, column=0, columnspan=2, pady=4, sticky="w")

        grp2 = ttk.Labelframe(f, text="Temperature", padding=12, bootstyle=INFO)
        grp2.grid(row=0, column=1, sticky="nsew", padx=6, pady=6)
        self.temp1_cal_var = tk.StringVar(value="25.0")
        self.temp2_cal_var = tk.StringVar(value="25.0")
        ttk.Label(grp2, text="Known Temp1 (\u00b0C)").grid(row=0, column=0, sticky="w")
        ttk.Entry(grp2, textvariable=self.temp1_cal_var, width=10).grid(row=0, column=1)
        ttk.Button(grp2, text="Apply", bootstyle=PRIMARY, command=lambda: self._guarded(
            lambda: self.client.set_temp_cal(float(self.temp1_cal_var.get())))).grid(row=0, column=2, padx=4)
        ttk.Label(grp2, text="Known Temp2 (\u00b0C)").grid(row=1, column=0, sticky="w")
        ttk.Entry(grp2, textvariable=self.temp2_cal_var, width=10).grid(row=1, column=1)
        ttk.Button(grp2, text="Apply", bootstyle=PRIMARY, command=lambda: self._guarded(
            lambda: self.client.set_temp2_cal(float(self.temp2_cal_var.get())))).grid(row=1, column=2, padx=4)

        grp3 = ttk.Labelframe(f, text="SOC / SOH / LC", padding=12, bootstyle=INFO)
        grp3.grid(row=1, column=0, sticky="nsew", padx=6, pady=6)
        ttk.Button(grp3, text="SOC = 100% (full)", bootstyle=SUCCESS,
                   command=lambda: self._guarded(self.client.soc_set_full)).grid(row=0, column=0, sticky="w", pady=2)
        ttk.Button(grp3, text="SOC = 0% (empty)", bootstyle=WARNING,
                   command=lambda: self._guarded(self.client.soc_set_empty)).grid(row=1, column=0, sticky="w", pady=2)
        ttk.Button(grp3, text="LC = 0", bootstyle=SECONDARY,
                   command=lambda: self._guarded(self.client.lc_zero)).grid(row=2, column=0, sticky="w", pady=2)
        self.soh_var = tk.StringVar(value="100")
        ttk.Label(grp3, text="Set SOH (%)").grid(row=3, column=0, sticky="w")
        ttk.Entry(grp3, textvariable=self.soh_var, width=8).grid(row=3, column=1)
        ttk.Button(grp3, text="Apply", bootstyle=PRIMARY, command=lambda: self._guarded(
            lambda: self.client.set_soh(float(self.soh_var.get())))).grid(row=3, column=2, padx=4)
        ttk.Button(grp3, text="Reset life counters", bootstyle=WARNING,
                   command=lambda: self._guarded(self.client.reset_life)).grid(row=4, column=0, sticky="w", pady=2)

        grp4 = ttk.Labelframe(f, text="System", padding=12, bootstyle=INFO)
        grp4.grid(row=1, column=1, sticky="nsew", padx=6, pady=6)
        ttk.Button(grp4, text="Save all config to flash", bootstyle=SECONDARY,
                   command=self._save_to_flash).grid(row=0, column=0, sticky="w", pady=2)
        ttk.Button(grp4, text="Clear latched faults", bootstyle=DANGER,
                   command=lambda: self._guarded(self.client.clear_faults)).grid(row=1, column=0, sticky="w", pady=2)
        ttk.Button(grp4, text="Reset drive state to IDLE", bootstyle=WARNING,
                   command=lambda: self._guarded(self.client.reset_state)).grid(row=2, column=0, sticky="w", pady=2)
        ttk.Button(grp4, text="Set RTC to PC time now", bootstyle=SECONDARY,
                   command=lambda: self._guarded(self.client.set_rtc_now)).grid(row=3, column=0, sticky="w", pady=2)

    def _save_to_flash(self):
        self._guarded(self.client.save_to_flash)

    # -- SOC Configuration ----------------------------------------------------

    def _build_soc_config_tab(self):
        f = self.tab_soc_config

        scalars = ttk.Labelframe(f, text="Pack Parameters", padding=12, bootstyle=PRIMARY)
        scalars.pack(fill="x", pady=(0, 10))

        self.soc_cfg_vars = {}
        for i, (key, label) in enumerate([
            ("pack_uv_limit", "Pack UV Limit (V)"),
            ("battery_capacity_ah", "Battery Capacity (Ah)"),
            ("pack_full_voltage", "Pack Full Voltage (V)"),
        ]):
            ttk.Label(scalars, text=label).grid(row=0, column=2 * i, sticky="w", padx=(0 if i == 0 else 16, 6))
            var = tk.StringVar(value="--")
            self.soc_cfg_vars[key] = var
            ttk.Entry(scalars, textvariable=var, width=12).grid(row=0, column=2 * i + 1, sticky="w")

        curve = ttk.Labelframe(f, text=f"OCV Curve ({SOC_OCV_POINTS} points) -- SOC%% reference vs. Pack Open-Circuit Voltage",
                                padding=12, bootstyle=INFO)
        curve.pack(fill="both", expand=True)
        ttk.Label(curve, text="Point").grid(row=0, column=0, padx=4, pady=(0, 6))
        ttk.Label(curve, text="SOC ref (%)").grid(row=0, column=1, padx=4, pady=(0, 6))
        ttk.Label(curve, text="Pack OCV (V)").grid(row=0, column=2, padx=4, pady=(0, 6))

        self.soc_ref_vars = []
        self.pack_ocv_vars = []
        for i in range(SOC_OCV_POINTS):
            ttk.Label(curve, text=str(i)).grid(row=i + 1, column=0, padx=4, pady=2)
            rv = tk.StringVar(value="--")
            ov = tk.StringVar(value="--")
            self.soc_ref_vars.append(rv)
            self.pack_ocv_vars.append(ov)
            ttk.Entry(curve, textvariable=rv, width=10).grid(row=i + 1, column=1, padx=4, pady=2)
            ttk.Entry(curve, textvariable=ov, width=10).grid(row=i + 1, column=2, padx=4, pady=2)

        ttk.Label(curve, text="Point 0 should be the lowest SOC/voltage pair, point "
                  f"{SOC_OCV_POINTS - 1} the highest -- the firmware interpolates "
                  "between consecutive points and does not re-sort them.",
                  bootstyle=SECONDARY, wraplength=420).grid(
            row=SOC_OCV_POINTS + 1, column=0, columnspan=3, sticky="w", pady=(8, 0))

        btns = ttk.Frame(f)
        btns.pack(fill="x", pady=(10, 0))
        ttk.Button(btns, text="Read current", bootstyle=SECONDARY,
                   command=lambda: self._guarded(self.client.get_soc_config)).pack(side="left", padx=4)
        ttk.Button(btns, text="Apply", bootstyle=PRIMARY, command=self._apply_soc_config).pack(side="left", padx=4)
        ttk.Button(btns, text="Save to Flash", bootstyle=SECONDARY, command=self._save_to_flash).pack(side="left", padx=4)

    def _apply_soc_config(self):
        try:
            pack_uv_limit = float(self.soc_cfg_vars["pack_uv_limit"].get())
            battery_capacity_ah = float(self.soc_cfg_vars["battery_capacity_ah"].get())
            pack_full_voltage = float(self.soc_cfg_vars["pack_full_voltage"].get())
            soc_ref = [float(v.get()) for v in self.soc_ref_vars]
            pack_ocv = [float(v.get()) for v in self.pack_ocv_vars]
        except ValueError:
            Messagebox.show_error("All SOC configuration fields must be numbers.", "Invalid input")
            return
        self._guarded(lambda: self.client.set_soc_config(
            pack_uv_limit, battery_capacity_ah, pack_full_voltage, soc_ref, pack_ocv))

    # -- Charger -----------------------------------------------------------

    def _build_charger_tab(self):
        f = self.tab_charger
        f.columnconfigure((0, 1), weight=1)

        cfg = ttk.Labelframe(f, text="Solterra charge profile (V/I setpoints)", padding=12, bootstyle=PRIMARY)
        cfg.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)

        self.chg_cellv_var = tk.StringVar(value="--")
        self.chg_ichg_var = tk.StringVar(value="--")
        self.chg_itaper_var = tk.StringVar(value="--")
        self.chg_cells_var = tk.StringVar(value="--")
        self.chg_storagev_var = tk.StringVar(value="--")

        rows = [
            ("Cell V setpoint (V/cell)", self.chg_cellv_var),
            ("CC current ceiling (A)", self.chg_ichg_var),
            ("CV cutoff/taper current (A)", self.chg_itaper_var),
            ("Series cell count", self.chg_cells_var),
            ("Storage mode cell V (V/cell)", self.chg_storagev_var),
        ]
        for i, (label, var) in enumerate(rows):
            ttk.Label(cfg, text=label).grid(row=i, column=0, sticky="w", pady=3)
            ttk.Entry(cfg, textvariable=var, width=12).grid(row=i, column=1, padx=8, pady=3)
        ttk.Label(cfg, text=(
            "Used instead of Cell V setpoint whenever an external CAN signal "
            "requests storage mode -- typ. ~3.80V/cell for long-term storage, "
            "must not exceed Cell V setpoint above."
        ), bootstyle=SECONDARY, wraplength=280, justify="left").grid(
            row=len(rows), column=0, columnspan=2, sticky="w", pady=(4, 8))

        btns = ttk.Frame(cfg)
        btns.grid(row=len(rows) + 1, column=0, columnspan=2, pady=(4, 0), sticky="w")
        ttk.Button(btns, text="Read current", bootstyle=SECONDARY,
                   command=lambda: self._guarded(self.client.get_charger_config)).pack(side="left", padx=4)
        ttk.Button(btns, text="Apply", bootstyle=PRIMARY, command=self._apply_charger_config).pack(side="left", padx=4)
        ttk.Button(btns, text="Save to Flash", bootstyle=SECONDARY, command=self._save_to_flash).pack(side="left", padx=4)

        status = ttk.Labelframe(f, text="Live charger status", padding=12, bootstyle=INFO)
        status.grid(row=0, column=1, sticky="nsew", padx=6, pady=6)

        self.chg_state_badge = Badge(status, text="UNKNOWN", bootstyle=SECONDARY)
        self.chg_state_badge.grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 8))

        self.chg_storage_badge = Badge(status, text="STORAGE MODE: UNKNOWN", bootstyle=SECONDARY)
        self.chg_storage_badge.grid(row=0, column=2, sticky="w", padx=(12, 0), pady=(0, 8))

        self.chg_status_vars = {}
        for i, (key, label) in enumerate([
            ("comm_ok", "Comm OK"),
            ("actual_v", "Actual V"), ("actual_i", "Actual I"),
            ("cmd_v", "Commanded V"), ("cmd_i", "Commanded I"),
        ], start=1):
            ttk.Label(status, text=label).grid(row=i, column=0, sticky="w")
            var = tk.StringVar(value="--")
            self.chg_status_vars[key] = var
            ttk.Label(status, textvariable=var, font=("TkDefaultFont", 10, "bold")).grid(
                row=i, column=1, sticky="w", padx=8)

        self.chg_flags_frame = ttk.Frame(status)
        self.chg_flags_frame.grid(row=6, column=0, columnspan=2, sticky="w", pady=(8, 0))

        override = ttk.Labelframe(f, text="Manual stop / resume", padding=12, bootstyle=WARNING)
        override.grid(row=1, column=0, columnspan=2, sticky="nsew", padx=6, pady=(6, 0))
        ttk.Label(override, text=(
            "STOP forces the charge command off immediately, independent of "
            "the automatic CC/CV logic. RESUME clears the override and hands "
            "control back to the automatic thermal/fault-gated state machine "
            "-- it cannot force a start through an active safety trip."
        ), wraplength=700, justify="left").grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))
        ttk.Button(override, text="\u25A0  STOP CHARGING", bootstyle=DANGER,
                   command=lambda: self._guarded(lambda: self.client.set_charger_override(True))
                   ).grid(row=1, column=0, padx=4, sticky="w")
        ttk.Button(override, text="\u25B6  RESUME (AUTO)", bootstyle=SUCCESS,
                   command=lambda: self._guarded(lambda: self.client.set_charger_override(False))
                   ).grid(row=1, column=1, padx=4, sticky="w")
        self.chg_override_badge = Badge(override, text="UNKNOWN", bootstyle=SECONDARY)
        self.chg_override_badge.grid(row=1, column=2, padx=12, sticky="w")

    def _apply_charger_config(self):
        try:
            cell_v = float(self.chg_cellv_var.get())
            ichg = float(self.chg_ichg_var.get())
            itaper = float(self.chg_itaper_var.get())
            cells = int(self.chg_cells_var.get())
            storage_v = float(self.chg_storagev_var.get())
        except ValueError:
            Messagebox.show_error("Charger config fields must be numbers.", "Invalid input")
            return
        if storage_v <= 0 or storage_v > cell_v:
            Messagebox.show_error(
                "Storage mode cell V must be positive and must not exceed "
                "Cell V setpoint.", "Invalid input")
            return
        self._guarded(lambda: self.client.set_charger_config(cell_v, ichg, itaper, cells, storage_v))

    # -- Device Identity -----------------------------------------------------

    def _build_id_tab(self):
        f = self.tab_id
        ttk.Label(f, text="RAM only until you click Save to Flash (same as "
                          "every other config on this bus).", bootstyle=SECONDARY,
                  wraplength=600, justify="left").grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 10))

        card = ttk.Labelframe(f, text="BMS / Pack Identity", padding=12, bootstyle=PRIMARY)
        card.grid(row=1, column=0, sticky="nsew")

        self.bms_sn_var = tk.StringVar(value="")
        self.pack_sn_var = tk.StringVar(value="")
        self.product_id_var = tk.StringVar(value="")
        self.mfg_date_var = tk.StringVar(value="")
        self.node_id_var = tk.StringVar(value="")

        ttk.Label(card, text="BMS Serial Number").grid(row=0, column=0, sticky="w", pady=4)
        ttk.Entry(card, textvariable=self.bms_sn_var, width=22).grid(row=0, column=1, sticky="w", padx=8)
        ttk.Label(card, text="Pack Serial Number").grid(row=1, column=0, sticky="w", pady=4)
        ttk.Entry(card, textvariable=self.pack_sn_var, width=22).grid(row=1, column=1, sticky="w", padx=8)
        ttk.Label(card, text="Product ID").grid(row=2, column=0, sticky="w", pady=4)
        ttk.Entry(card, textvariable=self.product_id_var, width=22).grid(row=2, column=1, sticky="w", padx=8)
        ttk.Label(card, text="Manufacture Date").grid(row=3, column=0, sticky="w", pady=4)
        ttk.Entry(card, textvariable=self.mfg_date_var, width=22).grid(row=3, column=1, sticky="w", padx=8)
        ttk.Label(card, text="  free-form, e.g. 06-08-2026 (max 11 chars)",
                  bootstyle=SECONDARY).grid(row=3, column=2, sticky="w", padx=(4, 0))
        ttk.Label(card, text="DroneCAN Node ID").grid(row=4, column=0, sticky="w", pady=4)
        ttk.Entry(card, textvariable=self.node_id_var, width=22).grid(row=4, column=1, sticky="w", padx=8)
        ttk.Label(card, text="  0-127; 0 = anonymous / request dynamic allocation",
                  bootstyle=SECONDARY).grid(row=4, column=2, sticky="w", padx=(4, 0))

        btns = ttk.Frame(card)
        btns.grid(row=5, column=0, columnspan=2, pady=(12, 0), sticky="w")
        ttk.Button(btns, text="Read current", bootstyle=SECONDARY,
                   command=lambda: self._guarded(self.client.get_device_ids)).pack(side="left", padx=4)
        ttk.Button(btns, text="Apply", bootstyle=PRIMARY, command=self._apply_device_ids).pack(side="left", padx=4)
        ttk.Button(btns, text="Save to Flash", bootstyle=SECONDARY, command=self._save_to_flash).pack(side="left", padx=4)

    def _apply_device_ids(self):
        try:
            pid = int(self.product_id_var.get() or "0")
        except ValueError:
            Messagebox.show_error("Product ID must be an integer.", "Invalid input")
            return
        try:
            node_id = int(self.node_id_var.get() or "0")
        except ValueError:
            Messagebox.show_error("Node ID must be an integer.", "Invalid input")
            return
        if not (0 <= node_id <= 127):
            Messagebox.show_error(
                "Node ID must be 0-127 (0 = anonymous / dynamic allocation).",
                "Invalid input")
            return
        self._guarded(lambda: self.client.set_device_ids(
            self.bms_sn_var.get(), self.pack_sn_var.get(), pid, self.mfg_date_var.get(), node_id))

    # -- Error Logs -----------------------------------------------------------

    def _build_logs_tab(self):
        f = self.tab_logs
        btns = ttk.Frame(f)
        btns.pack(side="top", fill="x", pady=(0, 8))
        ttk.Button(btns, text="Refresh (fetch all)", bootstyle=PRIMARY,
                   command=self._refresh_error_logs).pack(side="left")
        ttk.Button(btns, text="Save as CSV...", bootstyle=SECONDARY,
                   command=self._save_error_logs_csv).pack(side="left", padx=(8, 0))
        ttk.Button(btns, text="Clear Log (BMS)...", bootstyle=DANGER,
                   command=self._clear_error_logs).pack(side="left", padx=(8, 0))
        ttk.Label(btns, text="  Severity is color-coded: grey=info, amber=warning, red=critical.",
                  bootstyle=SECONDARY).pack(side="left", padx=(12, 0))

        self.log_fetch_status_var = tk.StringVar(value="Not loaded yet -- click Refresh.")
        ttk.Label(f, textvariable=self.log_fetch_status_var, bootstyle=SECONDARY).pack(
            side="top", anchor="w", pady=(0, 6))

        cols = ("seq", "time", "severity", "faults", "message")
        self.log_tree = ttk.Treeview(f, columns=cols, show="headings", height=20,
                                      bootstyle=PRIMARY)
        self.log_tree.heading("seq", text="Seq")
        self.log_tree.heading("time", text="Time")
        self.log_tree.heading("severity", text="Severity")
        self.log_tree.heading("faults", text="Fault(s)")
        self.log_tree.heading("message", text="Message")
        self.log_tree.column("seq", width=70, anchor="e")
        self.log_tree.column("time", width=150)
        self.log_tree.column("severity", width=90)
        self.log_tree.column("faults", width=220)
        self.log_tree.column("message", width=420)
        self.log_tree.tag_configure("sev_info", foreground="#888888")
        self.log_tree.tag_configure("sev_warning", foreground="#c9891a")
        self.log_tree.tag_configure("sev_critical", foreground="#d93b3b")
        self.log_tree.pack(fill="both", expand=True)

    def _refresh_error_logs(self):
        """Fetch the ENTIRE log (up to firmware capacity, ~1000+ entries) by
        repeatedly requesting 20-entry pages at increasing offsets --
        CLI_CMD_READ_ERROR_LOGS only ever returns 20 entries per request
        (deliberate, keeps each request's main-loop-blocking time on the
        BMS bounded), so this drives the paging loop from the GUI side.
        Stops when a page comes back with fewer than 20 entries (or an
        empty page), which means the log is exhausted, or after a fixed
        number of pages as a hard safety net against ever looping forever
        on a lossy bus."""
        for row in self.log_tree.get_children():
            self.log_tree.delete(row)
        self._log_fetch_offset = 0
        self._log_fetch_page_count = 0
        self._log_fetch_total = 0
        self._log_fetch_in_progress = True
        self._log_fetch_pages_requested = 0
        self.log_fetch_status_var.set("Loading...")
        self._request_error_log_page()

    def _request_error_log_page(self):
        self._log_fetch_page_count = 0
        self._log_fetch_pages_requested += 1
        self._guarded(lambda: self.client.read_error_logs(self._log_fetch_offset))
        # Give the bus ~600ms to deliver this page's frames before checking
        # progress -- generous relative to the ~100ms/frame CAN-CLI tunnel
        # cadence used elsewhere in this GUI, since up to 20 frames need to
        # arrive for a full page.
        self.after(600, self._check_error_log_page_progress)

    def _check_error_log_page_progress(self):
        if not self._log_fetch_in_progress:
            return
        # Hard safety net: firmware capacity is ~1060 entries (see
        # ERRLOG_SECTOR_COUNT, error_log_store.c) -- 55 pages of 20 comfortably
        # covers that with margin, so this can only trip on a genuinely
        # stuck/looping condition, not normal use.
        MAX_PAGES = 60
        if self._log_fetch_page_count >= 20 and self._log_fetch_pages_requested < MAX_PAGES:
            # Full page arrived -- there may be more, fetch the next one.
            self._log_fetch_offset += self._log_fetch_page_count
            self._request_error_log_page()
        else:
            self._finish_error_log_fetch()

    def _finish_error_log_fetch(self):
        self._log_fetch_in_progress = False
        self.log_fetch_status_var.set(
            f"Loaded {self._log_fetch_total} entries.")

    def _clear_error_logs(self):
        confirm = Messagebox.yesno(
            "This erases the BMS's entire fault log storage (all entries, "
            "not just what's currently loaded here). This cannot be undone. "
            "Continue?",
            "Clear error log?")
        if confirm not in ("Yes", "yes", True):
            return
        self._guarded(self.client.clear_error_logs)
        self.log_fetch_status_var.set("Clear command sent -- click Refresh to confirm the log is empty.")

    def _save_error_logs_csv(self):
        rows = self.log_tree.get_children()
        if not rows:
            Messagebox.show_error("No log entries loaded -- click Refresh first.", "Nothing to save")
            return
        path = filedialog.asksaveasfilename(
            title="Save error log as...",
            defaultextension=".csv",
            initialfile=f"bms_error_log_{time.strftime('%Y%m%d_%H%M%S')}.csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        try:
            with open(path, "w", newline="") as fh:
                writer = csv.writer(fh)
                writer.writerow(["seq", "time", "severity", "faults", "message"])
                for row_id in rows:
                    writer.writerow(self.log_tree.item(row_id, "values"))
            self.log_fetch_status_var.set(f"Saved {len(rows)} entries to {path}")
        except OSError as exc:
            Messagebox.show_error(f"Could not save file:\n{exc}", "Save failed")

    # -- CAN Diagnostics -----------------------------------------------------

    def _build_diag_tab(self):
        f = self.tab_diag
        card = ttk.Labelframe(f, text="Link statistics", padding=12, bootstyle=INFO)
        card.pack(fill="x")

        self.diag_vars = {}
        for i, (key, label) in enumerate([
            ("tx", "TX frames sent"), ("rx", "RX frames seen"),
            ("msgs", "Messages decoded"), ("errs", "Decode errors"),
            ("rejseq", "Reassembly: seq mismatch"), ("rejlen", "Reassembly: bad length"),
            ("rejorphan", "Reassembly: orphan CF"), ("age", "Time since last RX (s)"),
        ]):
            ttk.Label(card, text=label).grid(row=i // 2, column=(i % 2) * 2, sticky="w", padx=(0, 6), pady=4)
            var = tk.StringVar(value="--")
            self.diag_vars[key] = var
            ttk.Label(card, textvariable=var, font=("TkDefaultFont", 10, "bold")).grid(
                row=i // 2, column=(i % 2) * 2 + 1, sticky="w", padx=(0, 24), pady=4)

        ttk.Label(f, text="If 'RX frames seen' stays at 0 while connected: "
                          "check the PEAK-CAN adapter is wired to the same "
                          "bus as the BMS, the bitrate matches FDCAN_BAUD_RATE "
                          "(fdcan.c), and the BMS firmware includes the "
                          "CANFILT_IDX_GUICMD filter fix (fdcan.c).",
                  bootstyle=SECONDARY, wraplength=800, justify="left").pack(
            anchor="w", pady=(12, 8))

        self.error_text = ttk.Text(f, height=10, width=90)
        self.error_text.pack(fill="both", expand=True)
        self.error_text.configure(state="disabled")

    def _log_error(self, text: str):
        def _append():
            self.error_text.configure(state="normal")
            self.error_text.insert("end", f"{time.strftime('%H:%M:%S')}  {text}\n")
            self.error_text.see("end")
            self.error_text.configure(state="disabled")
        self.after(0, _append)

    # ---- status bar -------------------------------------------------------

    def _build_status_bar(self):
        bar = ttk.Frame(self, padding=(12, 4))
        bar.pack(side="bottom", fill="x")
        self.status_var = tk.StringVar(value="Not connected.")
        ttk.Label(bar, textvariable=self.status_var, bootstyle=SECONDARY).pack(side="left")

    # ---- connect/disconnect -------------------------------------------------

    def _toggle_tx_lock(self):
        if not self.client.connected:
            Messagebox.show_warning("Connect to the CAN adapter first.", "Not connected")
            return
        new_state = not self.client.tx_enabled
        self.client.set_tx_enabled(new_state)
        if new_state:
            self.tx_badge.set("TX UNLOCKED", WARNING)
            self.tx_toggle_btn.configure(text="Lock TX", bootstyle=DANGER)
            self.status_var.set("TX UNLOCKED -- you can now send commands.")
        else:
            self.tx_badge.set("TX LOCKED", SUCCESS)
            self.tx_toggle_btn.configure(text="Unlock TX", bootstyle=PRIMARY)
            self.status_var.set("TX LOCKED -- read-only, zero bus impact.")

    def _toggle_connect(self):
        if self.client.connected:
            self.client.disconnect()
            self.connect_btn.configure(text="Connect", bootstyle=SUCCESS)
            self.conn_badge.set("DISCONNECTED", DANGER)
            self.tx_badge.set("TX LOCKED", DANGER)
            self.tx_toggle_btn.configure(text="Unlock TX", bootstyle=PRIMARY)
            self.status_var.set("Not connected.")
            return
        try:
            self.client.connect(self.channel_var.get(), int(self.bitrate_var.get()))
        except Exception as exc:
            Messagebox.show_error(
                f"{exc}\n\nCheck: PEAK-CAN adapter plugged in, PCAN-Basic driver "
                "installed, channel name matches Device Manager / PCAN-View "
                "(commonly PCAN_USBBUS1), and bitrate matches FDCAN_BAUD_RATE "
                "in fdcan.c (default 500000).",
                "Connect failed")
            return
        self.connect_btn.configure(text="Disconnect", bootstyle=DANGER)
        self.conn_badge.set("CONNECTED", SUCCESS)
        self.tx_badge.set("TX LOCKED", SUCCESS)
        self.tx_toggle_btn.configure(text="Unlock TX", bootstyle=PRIMARY)
        self.status_var.set("Connected -- TX LOCKED (read-only). Click 'Unlock TX' to write.")

    # ---- helpers ------------------------------------------------------------

    def _guarded(self, fn, *, allow_monitor: bool = False):
        """
        Wrap a transmit function with connection + TX lock checks.
        If TX is locked, prompt the user to unlock it first.
        """
        if not self.client.connected:
            Messagebox.show_warning("Connect to the CAN adapter first.", "Not connected")
            return
        if not self.client.tx_enabled and not allow_monitor:
            choice = Messagebox.yesno(
                "TX is currently LOCKED (read-only).\n\n"
                "This action requires transmitting to the BMS.\n"
                "Unlock TX now?",
                "Transmit required")
            if choice in ("Yes", "yes", True):
                self._toggle_tx_lock()
            else:
                return
        try:
            fn()
        except Exception as exc:
            Messagebox.show_error(str(exc), "Send failed")

    # ---- RX pump / periodic UI refresh --------------------------------------

    def _poll_rx_queue(self):
        drained = 0
        while drained < 200:
            try:
                cmd, decoded = self.client.rx_queue.get_nowait()
            except queue.Empty:
                break
            drained += 1
            self._handle_decoded(cmd, decoded)

        self._refresh_diag()
        self.after(100, self._poll_rx_queue)

    def _handle_decoded(self, cmd, d):
        lc = self.live_cards
        lcache = self._log_cache  # sample cache for the CSV logger -- see _log_tick()

        if cmd == Cmd.SOC_DATA:
            self.soc_meter.configure(amount_used=max(0, min(100, d["soc_percent"])),
                                      bootstyle=_soc_bootstyle(d["soc_percent"]))
            lc["packv"].set(f"{d['pack_v']:.2f}")
            lc["current"].set(f"{d['current_a']:.2f}")
            lc["ah"].set(f"{d['remaining_ah']:.2f}")
            lcache["pack_v"] = d["pack_v"]
            lcache["current_a"] = d["current_a"]
            lcache["soc_percent"] = d["soc_percent"]
            lcache["remaining_ah"] = d["remaining_ah"]
        elif cmd == Cmd.TEMP_DATA:
            lc["temp1"].set(f"{d['temp_c']:.1f}")
            lcache["temp1_c"] = d["temp_c"]
        elif cmd == Cmd.TEMP2_DATA:
            lc["temp2"].set(f"{d['temp2_c']:.1f}")
            lcache["temp2_c"] = d["temp2_c"]
        elif cmd == Cmd.CELL_EXTREMA:
            lc["celldiff"].set(f"{d['max_v'] - d['min_v']:.3f}")
            self._min_cell_idx, self._max_cell_idx = d["min_idx"], d["max_idx"]
            lcache["cell_min_v"] = d["min_v"]
            lcache["cell_max_v"] = d["max_v"]
            lcache["cell_diff_mV"] = (d["max_v"] - d["min_v"]) * 1000
        elif cmd == Cmd.CELL_VOLT_ALL:
            for i, v in enumerate(d["cells"]):
                if i >= len(self.cell_badges):
                    break
                style = SECONDARY
                if i == self._max_cell_idx:
                    style = WARNING
                elif i == self._min_cell_idx:
                    style = INFO
                self.cell_badges[i].set(f"{v:.3f}", style)
                lcache[f"cell_{i}_v"] = v
        elif cmd == Cmd.FAULT_STATUS:
            if d["flags"]:
                self.fault_banner.set("\u26A0  FAULTS: " + ", ".join(d["flags"]), DANGER)
                self.active_faults_var.set(", ".join(d["flags"]))
            else:
                self.fault_banner.set("\u2713  No active faults", SUCCESS)
                self.active_faults_var.set("none")
            for k, v in d["thresholds"].items():
                if k in self.thresh_vars and k not in self._thresh_dirty:
                    self.thresh_vars[k].set(f"{v:g}")
            lcache["active_faults"] = ", ".join(d["flags"]) if d["flags"] else "none"
            lcache["fault_mask_hex"] = f"0x{d['mask']:04X}"
        elif cmd == Cmd.LC_DATA:
            lc["wh"].set(f"{d['remaining_wh']:.1f}")
            lcache["remaining_wh"] = d["remaining_wh"]
        elif cmd == Cmd.BMS_LIFE:
            self.soh_meter.configure(amount_used=max(0, min(100, d["soh_percent"])))
            lc["cycles"].set(str(d["cycles"]))
            lc["life_years"].set(f"{d['life_years']:.2f}")
            lcache["soh_percent"] = d["soh_percent"]
            lcache["cycles"] = d["cycles"]
            lcache["life_years"] = d["life_years"]
        elif cmd == Cmd.CAN_HEALTH_DATA:
            lc["recov"].set(str(d["bus_off_recoveries"]))
            lc["errpass"].set(str(d["error_passive_seconds"]))
            lcache["bus_off_recoveries"] = d["bus_off_recoveries"]
            lcache["error_passive_seconds"] = d["error_passive_seconds"]

        # ---- Fixed-ID broadcast fallbacks (passive mode) ----------------
        elif cmd == -DISPLAY_ID_VIT:
            lc["packv"].set(f"{d['pack_voltage']:.2f}")
            lc["current"].set(f"{d['pack_current']:.2f}")
            lc["temp1"].set(f"{d['temp1']:.1f}")
            lc["temp2"].set(f"{d['temp2']:.1f}")
            lcache["pack_v"] = d["pack_voltage"]
            lcache["current_a"] = d["pack_current"]
            lcache["temp1_c"] = d["temp1"]
            lcache["temp2_c"] = d["temp2"]
        elif cmd == -DISPLAY_ID_OVRVIEW:
            self.soc_meter.configure(amount_used=max(0, min(100, d["soc"])),
                                      bootstyle=_soc_bootstyle(d["soc"]))
            self.soh_meter.configure(amount_used=max(0, min(100, d["soh"])))
            lc["cycles"].set(str(d["cycles"]))
            lcache["soc_percent"] = d["soc"]
            lcache["soh_percent"] = d["soh"]
            lcache["cycles"] = d["cycles"]
            # Derive remaining Ah from remaining_capacity (kWh -> Ah is approximate,
            # but better than blank).  Use last known pack_v if available.
            pack_v = lcache.get("pack_v", 48.0)
            if pack_v > 0:
                rem_ah = d["remaining_capacity"] * 1000.0 / pack_v
                lc["ah"].set(f"{rem_ah:.2f}")
                lcache["remaining_ah"] = rem_ah
        elif cmd == -DISPLAY_ID_CELL_DELTAV:
            lc["celldiff"].set(f"{d['delta_v']:.3f}")
            lcache["cell_min_v"] = d["min_cell_v"]
            lcache["cell_max_v"] = d["max_cell_v"]
            lcache["cell_diff_mV"] = d["delta_v"] * 1000
        elif cmd == -CHARGER_RX_ID_STATUS:
            # FIX: previously cache-only -- every other fixed-ID fallback
            # above (VIT, OvrView, Cell_DeltaV) drives its GUI var directly
            # so Monitor mode (no ping thread -> firmware's
            # CanCli_ClientActive() times out after CANCLI_CLIENT_TIMEOUT_MS
            # and the CLI-tunnel CHARGER_STATUS_DATA broadcast silently
            # stops, can_cli_bridge.c) still has SOMETHING refreshing the
            # tile. This one and the CHARGER_TX_ID_CMD one below only ever
            # wrote lcache, which nothing reads for display -- so "Actual
            # V/I" and "Commanded V/I" on the Charger tab froze at their
            # last tunnel-fed value (or "--") within ~3s of connecting in
            # the default Monitor mode, even though the real charger CAN
            # traffic these decode from was arriving the whole time.
            self.chg_status_vars["actual_v"].set(f"{d['actual_voltage']:.2f} V")
            self.chg_status_vars["actual_i"].set(f"{d['actual_current']:.2f} A")
            lcache["charger_actual_v"] = d["actual_voltage"]
            lcache["charger_actual_i"] = d["actual_current"]
        elif cmd == -CHARGER_TX_ID_CMD:
            # FIX: see -CHARGER_RX_ID_STATUS just above -- same cache-only
            # bug. This is literally the "charging CMD" frame (the BMS's
            # actual voltage/current command TO the charger, CHARGER_TX_ID_
            # CMD = 0x1806E5F4), so it's the one most visibly stuck at a
            # stale value in Monitor mode without this.
            self.chg_status_vars["cmd_v"].set(f"{d['voltage_setpoint']:.2f} V")
            self.chg_status_vars["cmd_i"].set(f"{d['current_setpoint']:.2f} A")
            lcache["charger_cmd_v"] = d["voltage_setpoint"]
            lcache["charger_cmd_i"] = d["current_setpoint"]

        elif cmd == Cmd.CHARGER_CONFIG_DATA:
            self.chg_cellv_var.set(f"{d['cellVoltageSetpoint_V']:g}")
            self.chg_ichg_var.set(f"{d['chargeCurrentFull_A']:g}")
            self.chg_itaper_var.set(f"{d['cvCutoffCurrent_A']:g}")
            self.chg_cells_var.set(str(d["cellCount"]))
            self.chg_storagev_var.set(f"{d['storageVoltageSetpoint_V']:g}")
        elif cmd == Cmd.SOC_CONFIG_DATA:
            self.soc_cfg_vars["pack_uv_limit"].set(f"{d['pack_uv_limit']:g}")
            self.soc_cfg_vars["battery_capacity_ah"].set(f"{d['battery_capacity_ah']:g}")
            self.soc_cfg_vars["pack_full_voltage"].set(f"{d['pack_full_voltage']:g}")
            for i in range(SOC_OCV_POINTS):
                self.soc_ref_vars[i].set(f"{d['soc_ref'][i]:g}")
                self.pack_ocv_vars[i].set(f"{d['pack_ocv'][i]:g}")
        elif cmd == Cmd.CHARGER_STATUS_DATA:
            self.chg_state_badge.set(d["state_name"], CHARGER_STATE_BOOTSTYLE.get(d["state"], SECONDARY))
            if d["storage_mode"]:
                self.chg_storage_badge.set("STORAGE MODE: ACTIVE", WARNING)
            else:
                self.chg_storage_badge.set("STORAGE MODE: OFF", SECONDARY)
            csv_vars = self.chg_status_vars
            csv_vars["comm_ok"].set("Yes" if d["comm_ok"] else "No")
            csv_vars["actual_v"].set(f"{d['actual_v']:.2f} V")
            csv_vars["actual_i"].set(f"{d['actual_i']:.2f} A")
            csv_vars["cmd_v"].set(f"{d['cmd_v']:.2f} V")
            csv_vars["cmd_i"].set(f"{d['cmd_i']:.2f} A")
            lcache["charger_state"] = d["state_name"]
            lcache["charger_comm_ok"] = d["comm_ok"]
            lcache["charger_actual_v"] = d["actual_v"]
            lcache["charger_actual_i"] = d["actual_i"]
            lcache["charger_cmd_v"] = d["cmd_v"]
            lcache["charger_cmd_i"] = d["cmd_i"]
            lcache["charger_cv_phase"] = d["cv_phase"]
            lcache["charger_charge_done"] = d["charge_done"]
            lcache["charger_temp_blocked"] = d["temp_blocked"]
            lcache["charger_host_stopped"] = d["host_stopped"]
            lcache["charger_storage_mode"] = d["storage_mode"]

            for child in self.chg_flags_frame.winfo_children():
                child.destroy()
            flag_defs = [
                ("CV_PHASE", d["cv_phase"], INFO), ("CHARGE_DONE", d["charge_done"], SUCCESS),
                ("TEMP_BLOCKED", d["temp_blocked"], WARNING), ("HOST_STOPPED", d["host_stopped"], DANGER),
                ("STORAGE_MODE", d["storage_mode"], WARNING),
            ]
            for text, active, style in flag_defs:
                if active:
                    Badge(self.chg_flags_frame, text=text, bootstyle=style).pack(side="left", padx=2)
        elif cmd == Cmd.CHARGER_OVERRIDE_DATA:
            if d["force_stop"]:
                self.chg_override_badge.set("FORCE_STOP", DANGER)
            else:
                self.chg_override_badge.set("AUTO", SUCCESS)
        elif cmd == Cmd.DEVICE_IDS_DATA:
            self.bms_sn_var.set(d["bms_serial"])
            self.pack_sn_var.set(d["pack_serial"])
            self.product_id_var.set(str(d["product_id"]))
            self.mfg_date_var.set(d["mfg_date"])
            self.node_id_var.set(str(d["node_id"]))
        elif cmd == Cmd.ERROR_LOG_DATA:
            sev_tag = {0: "sev_info", 1: "sev_warning", 2: "sev_critical"}.get(d["severity"], "sev_info")
            self.log_tree.insert("", "end", tags=(sev_tag,), values=(
                d["seq"], d["time"], d["severity_name"], d["fault_names"], d["message"]))
            if self._log_fetch_in_progress:
                self._log_fetch_page_count += 1
                self._log_fetch_total += 1
                self.log_fetch_status_var.set(f"Loading... {self._log_fetch_total} entries so far")
        elif cmd == Cmd.ERROR_LOG_CLEARED:
            if d["ok"]:
                for row in self.log_tree.get_children():
                    self.log_tree.delete(row)
                self.log_fetch_status_var.set("Log cleared on the BMS -- 0 entries.")
            else:
                self.log_fetch_status_var.set("BMS reported the clear did NOT succeed.")
        elif cmd == Cmd.RTC_DATA:
            pass  # not surfaced in this build; SOC_DATA/etc. update far more often

    def _refresh_diag(self):
        st = self.client.stats
        dv = self.diag_vars
        dv["tx"].set(str(st.tx_frames))
        dv["rx"].set(str(st.rx_frames))
        dv["msgs"].set(str(st.rx_msgs))
        dv["errs"].set(str(st.rx_errors))
        rstats = self.client._reassembler.stats
        dv["rejseq"].set(str(rstats.rej_seq))
        dv["rejlen"].set(str(rstats.rej_len))
        dv["rejorphan"].set(str(rstats.rej_orphan_cf))
        if st.last_rx_tick:
            dv["age"].set(f"{time.time() - st.last_rx_tick:.1f}")
        if self.client.connected:
            tunnel = "\u2705 tunnel" if self.client.tunnel_alive else "\u26a0 idle"
            tx_state = "TX-ON" if self.client.tx_enabled else "TX-OFF"
            self.status_var.set(
                f"{tx_state} | {tunnel} | "
                f"{st.rx_msgs} msgs | {st.tx_frames} TX frames")
        else:
            self.status_var.set("Not connected.")

    def destroy(self):
        try:
            self._stop_logging()
        except Exception:
            pass
        try:
            self.client.disconnect()
        except Exception:
            pass
        super().destroy()


if __name__ == "__main__":
    app = BmsCanGui()
    app.protocol("WM_DELETE_WINDOW", app.destroy)
    app.mainloop()