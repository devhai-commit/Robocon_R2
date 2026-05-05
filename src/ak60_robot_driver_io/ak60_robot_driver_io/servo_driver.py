#!/usr/bin/env python3
import time

# ============================================================================
# 1. HIWONDER BUS SERVO DRIVER (MINIMAL & FAST POLLING)
# ============================================================================
class HiwonderBusServo:
    def __init__(self, serial_port, servo_ids, centers, invert, default_move_time_ms=80):
        self.ser = serial_port
        self.servo_ids = servo_ids
        self.centers = centers
        self.invert = invert
        self.default_time = default_move_time_ms

    def load_all(self):
        """Cấp điện và giữ lực (Torque ON) cho toàn bộ 8 Servo"""
        # Tự động tìm cổng Serial (Hỗ trợ cả bản gộp và bản tách file)
        serial_port = self.ser if hasattr(self, 'ser') else self.controller.ser
        
        for sid in self.servo_ids:
            # Packet: 0x55 0x55 ID LENGTH(4) CMD(31) PARAM(1) CHECKSUM
            pkt = [0x55, 0x55, sid, 4, 31, 1]
            pkt.append(255 - (sum(pkt[2:]) % 256))
            serial_port.write(bytes(pkt))

    def unload_all(self):
        """Ngắt điện và nhả lực (Torque OFF) cho toàn bộ 8 Servo"""
        serial_port = self.ser if hasattr(self, 'ser') else self.controller.ser
        
        for sid in self.servo_ids:
            # Packet: 0x55 0x55 ID LENGTH(4) CMD(31) PARAM(0) CHECKSUM
            pkt = [0x55, 0x55, sid, 4, 31, 0]
            pkt.append(255 - (sum(pkt[2:]) % 256))
            serial_port.write(bytes(pkt))

    def move_single(self, sid, position, move_time=None):
        if sid not in self.servo_ids: return
        idx = self.servo_ids.index(sid)
        actual_pos = - position if self.invert[idx] else position
        actual_pos = max(0, min(875, actual_pos))
        m_time = move_time if move_time is not None else self.default_time
        
        pkt = [0x55, 0x55, sid, 7, 1, actual_pos & 0xFF, (actual_pos >> 8) & 0xFF, m_time & 0xFF, (m_time >> 8) & 0xFF]
        pkt.append(255 - (sum(pkt[2:]) % 256))
        self.ser.write(bytes(pkt))


    def read_position(self):
        positions = []
        old_timeout = self.ser.timeout
        self.ser.timeout = 0.005

        for sid in self.servo_ids:
            pkt = [0x55, 0x55, sid, 3, 28] # Lệnh 28: Đọc vị trí
            pkt.append(255 - (sum(pkt[2:]) % 256))
            raw_pos = None

            for _ in range(2): # Thử tối đa 2 lần
                try:
                    self.ser.reset_input_buffer()
                    self.ser.write(bytes(pkt))
                    hdr = self.ser.read(4)
                    if len(hdr) == 4 and list(hdr[0:2]) == [0x55, 0x55]:
                        payload = self.ser.read(hdr[3] - 1)
                        data = list(hdr) + list(payload)
                        if len(data) > 3 and data[-1] == (255 - (sum(data[2:-1]) % 256)):
                            raw_pos = data[5] + (data[6] << 8)
                            if raw_pos > 32767: raw_pos -= 65536
                            break
                except: pass

            if raw_pos is not None:
                idx = self.servo_ids.index(sid)
                positions.append(- raw_pos if self.invert[idx] else raw_pos)
            else:
                positions.append(None)
                
        self.ser.timeout = old_timeout # Trả lại timeout cũ
        return positions


# ============================================================================
# 2. WAVESHARE CF35 GRIPPER DRIVER (MINIMAL, SAFE & ENDIAN-AWARE)
# ============================================================================
class WaveShareCF35:
    def __init__(self, serial_port, end=0):
        self.ser = serial_port
        self.end = end

    def _host2scs(self, v):
        # Chuyển đổi số nguyên 16-bit thành 2 byte gửi đi (hỗ trợ Endianness)
        return [(v >> 8) & 0xFF, v & 0xFF] if self.end else [v & 0xFF, (v >> 8) & 0xFF]

    def _scs2host(self, l, h):
        # Ghép 2 byte nhận được thành số nguyên 16-bit (hỗ trợ Endianness)
        return (l << 8 | h) if self.end else (h << 8 | l)

    def _checksum(self, data):
        return (~sum(data)) & 0xFF

    def _write_packet(self, sid, inst, addr, data):
        self.ser.reset_input_buffer()
        pkt = [0xFF, 0xFF, sid, len(data) + 3, inst, addr] + data
        pkt.append(self._checksum(pkt[2:]))
        self.ser.write(bytes(pkt))

    def _read_packet(self):
        buf = []
        while True:
            b = self.ser.read(1)
            if not b: return None
            buf.append(b[0])
            if len(buf) >= 2 and buf[-2:] == [0xFF, 0xFF]: break

        hdr = self.ser.read(3)
        if len(hdr) != 3: return None
        sid, length, status = hdr
        data = self.ser.read(length - 2)
        chk = self.ser.read(1)
        
        if not chk or chk[0] != self._checksum([sid, length, status] + list(data)):
            return None
        return data

    def EnableTorque(self, sid, enable=True):
        self._write_packet(sid, 0x03, 40, [1 if enable else 0])

    def WritePosEx(self, sid, pos, speed=0, acc=0, torque=500):
        # Chuyển số âm thành định dạng của Waveshare (Bit cao nhất = 1)
        if pos < 0: pos = (-pos) | 0x8000
        if speed < 0: speed = (-speed) | 0x8000
        if torque < 0: torque = (-torque) | 0x8000
        
        # Dùng _host2scs để tách byte theo đúng chuẩn Endianness
        data = [acc] + self._host2scs(pos) + self._host2scs(torque) + self._host2scs(speed)
        self._write_packet(sid, 0x03, 41, data)

    def _read_word(self, sid, addr):
        self._write_packet(sid, 0x02, addr, [2])
        data = self._read_packet()
        if data and len(data) >= 2:
            # Dùng _scs2host để ghép byte theo đúng chuẩn Endianness
            return self._scs2host(data[0], data[1])
        return None

    def ReadPosition(self, sid):
        v = self._read_word(sid, 56)
        return -(v & 0x7FFF) if v and (v & 0x8000) else v

    def ReadCurrent(self, sid):
        v = self._read_word(sid, 69)
        return -(v & 0x7FFF) if v and (v & 0x8000) else v