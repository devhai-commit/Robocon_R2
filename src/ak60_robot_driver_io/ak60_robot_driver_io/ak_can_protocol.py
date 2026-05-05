
import struct

class AK60MIT:
    """AK60-6 MIT mode protocol (CAN extended ID)."""

    MODE_MIT = 8  # 0x08

    # AK60-6 limits (from manual)
    P_MIN, P_MAX = -12.56, 12.56
    V_MIN, V_MAX = -60.0, 60.0
    T_MIN, T_MAX = -12.0, 12.0
    KP_MIN, KP_MAX = 0.0, 500.0
    KD_MIN, KD_MAX = 0.0, 5.0

    @staticmethod
    def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
        span = x_max - x_min
        if span <= 0.0:
            raise ValueError("Invalid range")
        if x < x_min:
            x = x_min
        elif x > x_max:
            x = x_max
        return int((x - x_min) * ((1 << bits) / span))

    @classmethod
    def pack_mit(cls, p_des: float, v_des: float, kp: float, kd: float, t_ff: float) -> bytes:
        p_int = cls.float_to_uint(p_des, cls.P_MIN, cls.P_MAX, 16)
        v_int = cls.float_to_uint(v_des, cls.V_MIN, cls.V_MAX, 12)
        kp_int = cls.float_to_uint(kp, cls.KP_MIN, cls.KP_MAX, 12)
        kd_int = cls.float_to_uint(kd, cls.KD_MIN, cls.KD_MAX, 12)
        t_int = cls.float_to_uint(t_ff, cls.T_MIN, cls.T_MAX, 12)

        b0 = (kp_int >> 4) & 0xFF
        b1 = ((kp_int & 0xF) << 4) | ((kd_int >> 8) & 0xF)
        b2 = kd_int & 0xFF
        b3 = (p_int >> 8) & 0xFF
        b4 = p_int & 0xFF
        b5 = (v_int >> 4) & 0xFF
        b6 = ((v_int & 0xF) << 4) | ((t_int >> 8) & 0xF)
        b7 = t_int & 0xFF

        return bytes([b0, b1, b2, b3, b4, b5, b6, b7])

    @classmethod
    def mit_eid(cls, motor_id: int) -> int:
        """Extended CAN ID = (0x08 << 8) | motor_id"""
        return (cls.MODE_MIT << 8) | (motor_id & 0xFF)

    @staticmethod
    def parse_feedback(can_id_29: int, data: bytes):
        """Parse upload frame: [posH,posL, spdH,spdL, curH,curL,temp,err]"""
        if len(data) < 8:
            return None

        motor_id = can_id_29 & 0xFF

        pos_int = (data[0] << 8) | data[1]
        spd_int = (data[2] << 8) | data[3]
        cur_int = (data[4] << 8) | data[5]

        # signed conversion
        if pos_int & 0x8000:
            pos_int -= 0x10000
        if spd_int & 0x8000:
            spd_int -= 0x10000
        if cur_int & 0x8000:
            cur_int -= 0x10000

        pos = pos_int * 0.1
        speed = spd_int * 10.0
        current = cur_int * 0.01
        temp = data[6]
        error = data[7]

        return {
            "motor_id": motor_id,
            "position": pos,
            "speed": speed,
            "current": current,
            "temperature": temp,
            "error": error,
        }

class AK60Servo:
    """Servo mode helpers. We uv_wheelse Velocity Loop Mode (Mode ID = 3)."""
    MODE_RPM = 3
    MODE_POS_SPD = 6
    MODE_SET_ORIGIN = 5 

    @classmethod
    def rpm_eid(cls, motor_id: int) -> int:
        return (cls.MODE_RPM << 8) | (motor_id & 0xFF)

    @staticmethod
    def pack_erpm(erpm: int) -> bytes:
        """Pack ERPM (electrical RPM) into 4 bytes big-endian signed int32."""
        # limit to int32
        if erpm < -0x80000000:
            erpm = -0x80000000
        elif erpm > 0x7FFFFFFF:
            erpm = 0x7FFFFFFF
        return int(erpm).to_bytes(4, byteorder="big", signed=True)
    
    MODE_SET_ORIGIN = 5
    MODE_POS_SPD = 6

    def servo_eid(mode: int, motor_id: int) -> int:
        # Extended ID: [28:8]=mode, [7:0]=motor_id
        return (mode << 8) | motor_id

    def pack_set_origin(temp_or_perm: int) -> bytes:
        return bytes([temp_or_perm & 0xFF])

    def pack_pos_spd(pos_deg: float, spd_erpm: float, acc_erpm_s2: float) -> bytes:
        pos_i32 = int(pos_deg * 10000.0)
        spd_i16 = int(spd_erpm / 10.0)
        acc_i16 = int(acc_erpm_s2 / 10.0)
        return struct.pack(">i h h", pos_i32, spd_i16, acc_i16)
    
    