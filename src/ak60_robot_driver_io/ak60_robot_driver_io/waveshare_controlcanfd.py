#!/usr/bin/env python3
"""Waveshare USB-CAN-FD-B (ControlCANFD / ZCAN) driver for Linux.

This device uses vendor API (libcontrolcanfd.so) and does NOT expose SocketCAN.
We init the channel using CANFD init type (per vendor examples), and transmit classic CAN frames (<=8 bytes).
"""

from __future__ import annotations
from ctypes import *
from typing import List, Tuple, Optional
import os

VCI_USBCAN2 = 41
STATUS_OK = 1
INVALID_DEVICE_HANDLE = 0
INVALID_CHANNEL_HANDLE = 0

TYPE_CAN = 0
TYPE_CANFD = 1
CANFD_STANDARD_ISO = 0

class _ZCAN_CHANNEL_CANFD_INIT_CONFIG(Structure):
    _fields_ = [
        ("acc_code", c_uint),
        ("acc_mask", c_uint),
        ("abit_timing", c_uint),
        ("dbit_timing", c_uint),
        ("brp", c_uint),
        ("filter", c_ubyte),
        ("mode", c_ubyte),
        ("pad", c_ushort),
        ("reserved", c_uint),
    ]

class _ZCAN_CHANNEL_INIT_CONFIG(Union):
    _fields_ = [("canfd", _ZCAN_CHANNEL_CANFD_INIT_CONFIG)]

class ZCAN_CHANNEL_INIT_CONFIG(Structure):
    _fields_ = [("can_type", c_uint), ("config", _ZCAN_CHANNEL_INIT_CONFIG)]

class ZCAN_CAN_FRAME(Structure):
    _fields_ = [
        ("can_id", c_uint, 29),
        ("err", c_uint, 1),
        ("rtr", c_uint, 1),
        ("eff", c_uint, 1),
        ("can_dlc", c_ubyte),
        ("__pad", c_ubyte),
        ("__res0", c_ubyte),
        ("__res1", c_ubyte),
        ("data", c_ubyte * 8),
    ]

class ZCAN_Transmit_Data(Structure):
    _fields_ = [("frame", ZCAN_CAN_FRAME), ("transmit_type", c_uint)]

class ZCAN_Receive_Data(Structure):
    _fields_ = [("frame", ZCAN_CAN_FRAME), ("timestamp", c_ulonglong)]

class WaveshareControlCANFD:
    """Classic CAN send/recv over USB-CAN-FD-B using libcontrolcanfd.so."""

    def __init__(self, so_path: str, device_index: int = 0):
        if not os.path.isabs(so_path):
            so_path = os.path.abspath(so_path)
        self.so_path = so_path
        self.device_index = int(device_index)
        self.dll = cdll.LoadLibrary(self.so_path)

        self.dll.ZCAN_OpenDevice.restype = c_void_p
        self.dll.ZCAN_SetAbitBaud.argtypes = (c_void_p, c_ulong, c_ulong)
        self.dll.ZCAN_SetDbitBaud.argtypes = (c_void_p, c_ulong, c_ulong)
        if hasattr(self.dll, "ZCAN_SetCANFDStandard"):
            self.dll.ZCAN_SetCANFDStandard.argtypes = (c_void_p, c_ulong, c_ulong)
        self.dll.ZCAN_InitCAN.argtypes = (c_void_p, c_ulong, c_void_p)
        self.dll.ZCAN_InitCAN.restype = c_void_p
        self.dll.ZCAN_StartCAN.argtypes = (c_void_p,)

        self.dll.ZCAN_Transmit.argtypes = (c_void_p, c_void_p, c_ulong)
        self.dll.ZCAN_GetReceiveNum.argtypes = (c_void_p, c_ulong)
        self.dll.ZCAN_Receive.argtypes = (c_void_p, c_void_p, c_ulong, c_long)

        self.dll.ZCAN_ResetCAN.argtypes = (c_void_p,)
        self.dll.ZCAN_CloseDevice.argtypes = (c_void_p,)

        self.dev: Optional[c_void_p] = None
        self.ch: Optional[c_void_p] = None

    def open(self, channel: int, abit_baud: int, dbit_baud: int = 5_000_000, set_all_channels: bool = True):
        self.dev = c_void_p(self.dll.ZCAN_OpenDevice(VCI_USBCAN2, self.device_index, 0))
        if int(self.dev.value) == INVALID_DEVICE_HANDLE:
            raise RuntimeError("ZCAN_OpenDevice failed")

        channels = (0, 1) if set_all_channels else (int(channel),)
        for ch in channels:
            if int(self.dll.ZCAN_SetAbitBaud(self.dev, ch, int(abit_baud))) != STATUS_OK:
                raise RuntimeError(f"ZCAN_SetAbitBaud failed (ch={ch}, bitrate={abit_baud})")
            try:
                self.dll.ZCAN_SetDbitBaud(self.dev, ch, int(dbit_baud))
            except Exception:
                pass
            if hasattr(self.dll, "ZCAN_SetCANFDStandard"):
                try:
                    self.dll.ZCAN_SetCANFDStandard(self.dev, ch, int(CANFD_STANDARD_ISO))
                except Exception:
                    pass

        init_cfg = ZCAN_CHANNEL_INIT_CONFIG()
        init_cfg.can_type = TYPE_CANFD
        init_cfg.config.canfd.mode = 0

        self.ch = c_void_p(self.dll.ZCAN_InitCAN(self.dev, int(channel), byref(init_cfg)))
        if int(self.ch.value) == INVALID_CHANNEL_HANDLE:
            raise RuntimeError("ZCAN_InitCAN failed")

        if int(self.dll.ZCAN_StartCAN(self.ch)) != STATUS_OK:
            raise RuntimeError("ZCAN_StartCAN failed")

    def close(self):
        try:
            if self.ch and int(self.ch.value):
                self.dll.ZCAN_ResetCAN(self.ch)
        except Exception:
            pass
        try:
            if self.dev and int(self.dev.value):
                self.dll.ZCAN_CloseDevice(self.dev)
        except Exception:
            pass
        self.ch = None
        self.dev = None

    def send(self, can_id: int, data: bytes, eff: int = 1) -> bool:
        if self.ch is None or int(self.ch.value) == 0:
            raise RuntimeError("Device not opened")
        if len(data) > 8:
            raise ValueError("Classic CAN max 8 bytes")

        msg = ZCAN_Transmit_Data()
        msg.transmit_type = 0
        msg.frame.eff = int(eff)
        msg.frame.rtr = 0
        msg.frame.err = 0
        msg.frame.can_id = int(can_id) & (0x1FFFFFFF if eff else 0x7FF)
        msg.frame.can_dlc = len(data)

        for i in range(8):
            msg.frame.data[i] = 0
        for i, b in enumerate(data):
            msg.frame.data[i] = b

        ret = int(self.dll.ZCAN_Transmit(self.ch, byref(msg), 1))
        return ret == 1

    def recv_all(self, max_frames: int = 256) -> List[Tuple[int, bytes, int, int]]:
        if self.ch is None or int(self.ch.value) == 0:
            return []
        n = int(self.dll.ZCAN_GetReceiveNum(self.ch, TYPE_CAN))
        if n <= 0:
            return []
        n = min(n, int(max_frames))
        arr = (ZCAN_Receive_Data * n)()
        got = int(self.dll.ZCAN_Receive(self.ch, byref(arr), n, 0))

        # CHỐT CHẶN: Nếu DLL trả về mã lỗi (số âm), thoát ngay lập tức
        if got <= 0:
            return []
        
        out: List[Tuple[int, bytes, int, int]] = []

        for i in range(got):
            raw_id = int(arr[i].frame.can_id)
            # Dùng mặt nạ 0x1FFFFFFF để chỉ lấy đúng giá trị ID (29-bit hoặc 11-bit)
            rx_id = raw_id & 0x1FFFFFFF

            # bit 31 chính là cờ báo Extended (EFF):
            # eff = bool(raw_id & 0x80000000) if not hasattr(arr[i].frame, 'eff') else bool(arr[i].frame.eff)
            
            eff = bool(arr[i].frame.eff)
            dlc = int(arr[i].frame.can_dlc)
            payload = bytes(arr[i].frame.data[:dlc])
            out.append((rx_id, payload, int(arr[i].timestamp), eff))
        return out
