import time
import random
import threading
import subprocess
import os
import struct
from enum import IntEnum

# --- HARDWARE INTEGRATION GUARDS ---
try:
    import board
    import busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
    from DFRobot_ICG20660L import DFRobot_ICG20660L_IIC
    I2C_AVAILABLE = True
except ImportError:
    I2C_AVAILABLE = False

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False


class CAN_PACKET_ID(IntEnum):
    CAN_PACKET_SET_DUTY = 0
    CAN_PACKET_SET_CURRENT = 1
    CAN_PACKET_SET_CURRENT_BRAKE = 2
    CAN_PACKET_SET_RPM = 3
    CAN_PACKET_SET_POS = 4
    CAN_PACKET_SET_ORIGIN_HERE = 5
    CAN_PACKET_SET_POS_SPD = 6


def build_can_id(controller_id: int, mode: CAN_PACKET_ID) -> int:
    return ((int(mode) & 0xFF) << 8) | (controller_id & 0xFF)

def pack_pos_spd(controller_id: int, pos_deg: float, spd_erpm: float, accel_erpm2: float) -> tuple[int, bytes]:
    can_id = build_can_id(controller_id, CAN_PACKET_ID.CAN_PACKET_SET_POS_SPD)
    pos_int = int(pos_deg * 10000.0)
    spd_int = int(spd_erpm / 10.0)
    acc_int = int(accel_erpm2)
    data = struct.pack(">i", pos_int) + struct.pack(">h", spd_int) + struct.pack(">h", acc_int)
    return can_id, data

def pack_position(controller_id: int, pos_deg: float) -> tuple[int, bytes]:
    can_id = build_can_id(controller_id, CAN_PACKET_ID.CAN_PACKET_SET_POS)
    data = struct.pack(">i", int(pos_deg * 10000.0))
    return can_id, data

def pack_set_origin(controller_id: int, set_origin_mode: int = 0) -> tuple[int, bytes]:
    can_id = build_can_id(controller_id, CAN_PACKET_ID.CAN_PACKET_SET_ORIGIN_HERE)
    data = struct.pack("B", set_origin_mode & 0xFF)
    return can_id, data

def pack_brake(controller_id: int, current_brake_a: float = 0.0) -> tuple[int, bytes]:
    can_id = build_can_id(controller_id, CAN_PACKET_ID.CAN_PACKET_SET_CURRENT_BRAKE)
    data = struct.pack(">i", int(current_brake_a * 1000.0))
    return can_id, data

def parse_telemetry(data: bytes) -> dict:
    if len(data) < 8:
        return {"position_deg": 0.0, "speed_erpm": 0.0, "current_a": 0.0, "temperature_c": 0, "error_code": 0}
    pos_raw, spd_raw, cur_raw, temp_raw, err_raw = struct.unpack(">hhhbB", data[:8])
    return {
        "position_deg": pos_raw * 0.1,
        "speed_erpm": spd_raw * 10.0,
        "current_a": cur_raw * 0.01,
        "temperature_c": temp_raw,
        "error_code": err_raw
    }


class CubeMarsCANManager:
    def __init__(self, controller_ids: list[int]):
        self.controller_ids = controller_ids
        self.bus = None
        self.bus_lock = threading.RLock()
        self.running = False
        self.bg_thread = None

        self.telemetry = {
            cid: {"position_deg": 0.0, "speed_erpm": 0.0, "current_a": 0.0, "temperature_c": 0, "error_code": 0}
            for cid in controller_ids
        }
        self.motor_states = {
            cid: {"state": "UNLOCKED", "target_pos": 0.0, "speed": 1000.0, "accel": 200.0}
            for cid in controller_ids
        }

    def init_can(self, interface: str = 'can0', bitrate: int = 1000000) -> bool:
        if not CAN_AVAILABLE:
            return False
        try:
            cmd_prefix = [] if os.geteuid() == 0 else ['sudo']
            subprocess.run(cmd_prefix + ['ip', 'link', 'set', interface, 'down'], capture_output=True)
            time.sleep(0.05)
            cmd = cmd_prefix + ['ip', 'link', 'set', interface, 'up', 'type', 'can', 'bitrate', str(bitrate), 'restart-ms', '100']
            subprocess.run(cmd, check=True)
            self.bus = can.Bus(channel=interface, bustype='socketcan', bitrate=bitrate)
            return True
        except Exception as e:
            print(f"⚠️ CAN Init Error: {e}")
            return False

    def start(self):
        if not self.bus:
            return
        self.running = True
        self.bg_thread = threading.Thread(target=self._background_loop, daemon=True)
        self.bg_thread.start()

    def stop(self):
        self.running = False
        if self.bg_thread:
            self.bg_thread.join(timeout=1.0)
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass

    def _send_can_msg(self, can_id: int, data: bytes):
        if not self.bus:
            return
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=True)
        with self.bus_lock:
            try:
                self.bus.send(msg)
            except Exception:
                pass

    def emergency_release_all(self):
        """Immediately unlocks motors to neutral position upon E-Stop."""
        with self.bus_lock:
            for cid in self.controller_ids:
                self.motor_states[cid]["state"] = "UNLOCKED"
                can_id, data = pack_brake(cid, 0.0)
                self._send_can_msg(can_id, data)

    def _background_loop(self):
        last_keepalive = 0.0
        while self.running:
            now = time.time()
            try:
                msg = self.bus.recv(timeout=0.01)
                if msg is not None and len(msg.data) >= 8:
                    sender_id = msg.arbitration_id & 0xFF
                    if sender_id in self.telemetry:
                        parsed = parse_telemetry(msg.data)
                        with self.bus_lock:
                            self.telemetry[sender_id] = parsed
            except Exception:
                pass

            if now - last_keepalive >= 0.150:
                last_keepalive = now
                with self.bus_lock:
                    for cid, state_info in self.motor_states.items():
                        st = state_info["state"]
                        t_pos = state_info["target_pos"]
                        spd = state_info["speed"]
                        acc = state_info["accel"]
                        cur_pos = self.telemetry[cid]["position_deg"]
                        cur_spd = self.telemetry[cid]["speed_erpm"]

                        if st == "MOVING":
                            if abs(cur_pos - t_pos) <= 2.0 and abs(cur_spd) <= 50.0:
                                state_info["state"] = "HOLDING"
                                cid_pos, data = pack_position(cid, t_pos)
                                self._send_can_msg(cid_pos, data)
                            else:
                                cid_spd, data = pack_pos_spd(cid, t_pos, spd, acc)
                                self._send_can_msg(cid_spd, data)
                        elif st == "HOLDING":
                            cid_pos, data = pack_position(cid, t_pos)
                            self._send_can_msg(cid_pos, data)

    def set_move(self, controller_id: int, position: float, speed: float, acceleration: float):
        with self.bus_lock:
            self.motor_states[controller_id] = {
                "state": "MOVING",
                "target_pos": position,
                "speed": speed,
                "accel": acceleration
            }
            can_id, data = pack_pos_spd(controller_id, position, speed, acceleration)
            self._send_can_msg(can_id, data)

    def set_origin_all(self):
        with self.bus_lock:
            for cid in self.controller_ids:
                can_id, data = pack_set_origin(cid, 0)
                self._send_can_msg(can_id, data)
                time.sleep(0.005)

    def get_motor_state_data(self, cid: int) -> tuple[float, float, float]:
        with self.bus_lock:
            st = self.motor_states.get(cid, {})
            pos = self.telemetry.get(cid, {}).get("position_deg", 0.0)
            spd = st.get("speed", 0.0)
            acc = st.get("accel", 0.0)
            return pos, spd, acc


class SensorAcquisitionWorker:
    def __init__(self, data_queue):
        self.data_queue = data_queue
        self.running = True
        self.is_streaming = False
        self.hardware_initialized = False 
        
        self.ADC1_ADDR = 0x48  
        self.ADC2_ADDR = 0x4A  
        self.ser = None
        self.last_emg = [0, 0]
        
        self.flex_calib = {
            'f1': (1.38, 1.55),
            'f2': (1.34, 1.68),
            'f3': (1.22, 1.56),
            'f4': (1.27, 1.60),
            'f5': (1.58, 1.74),
        }
        
        self.ax_off = self.ay_off = self.az_off = 0.0
        self.gx_off = self.gy_off = self.gz_off = 0.0

        self.motor_ids = [1, 2, 11, 12]
        self.can_mgr = CubeMarsCANManager(self.motor_ids)
        self.current_movement_label = "IDLE"
        self.movement_thread = None

        # --- SAFETY & SMART AAN STATE MACHINE ---
        self.e_stop_triggered = False
        self.e_stop_reason = ""
        self.safety_overshoot_margin_deg = 5.0 # Max allowed overshoot buffer

    def voltage_to_percentage(self, volts, v_min, v_max):
        if volts <= v_min: return 0.0
        if volts >= v_max: return 100.0
        return round(((volts - v_min) / (v_max - v_min)) * 100.0, 1)

    def reinitialize_hardware(self):
        if self.ser and hasattr(self.ser, 'is_open') and self.ser.is_open:
            try: self.ser.close()
            except Exception: pass
        self.ser = None
        self.can_mgr.stop()
        self.hardware_initialized = False
        self.e_stop_triggered = False
        return self.initialize_hardware_inside_thread()

    def initialize_hardware_inside_thread(self):
        if not self.hardware_initialized:
            try:
                if I2C_AVAILABLE:
                    self.i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
                    self.ads1 = ADS.ADS1115(self.i2c, address=self.ADC1_ADDR)
                    self.ads1.gain = 1; self.ads1.data_rate = 860
                    self.ch_f1 = AnalogIn(self.ads1, 0)
                    self.ch_f2 = AnalogIn(self.ads1, 1)
                    self.ch_f3 = AnalogIn(self.ads1, 2)
                    self.ch_f4 = AnalogIn(self.ads1, 3)

                    self.ads2 = ADS.ADS1115(self.i2c, address=self.ADC2_ADDR)
                    self.ads2.gain = 1; self.ads2.data_rate = 860
                    self.ch_f5 = AnalogIn(self.ads2, 0)

                    self.imu = DFRobot_ICG20660L_IIC(addr=DFRobot_ICG20660L_IIC.IIC_ADDR_SDO_H)
                    while self.imu.begin(self.imu.eREG_MODE) != 0: time.sleep(0.5)
                    self.imu.enable_sensor(bit=self.imu.eAXIS_ALL)
                    self.imu.config_gyro(scale=self.imu.eFSR_G_500DPS, bd=self.imu.eGYRO_DLPF_176_1KHZ)
                    self.imu.config_accel(scale=self.imu.eFSR_A_16G, bd=self.imu.eACCEL_DLPF_218_1KHZ)
                    self.imu.set_sample_div(div=19)

                if SERIAL_AVAILABLE:
                    self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.005)
                    self.ser.reset_input_buffer()

                if CAN_AVAILABLE:
                    if self.can_mgr.init_can('can0', 1000000):
                        self.can_mgr.start()
                        self.can_mgr.set_origin_all()

                self.hardware_initialized = True
                return True, "✅ ALL HARDWARE PIPELINES ONLINE"
            except Exception as e:
                return False, f"⚠️ PARTIAL / SIMULATION MODE:\n{str(e)}"
        return True, "💻 SIMULATION MODE ACTIVE"

    def trigger_emergency_stop(self, reason="Manual/Overshoot"):
        """Instant safety lockout: releases motor lock and stops sequence."""
        self.e_stop_triggered = True
        self.e_stop_reason = reason
        self.current_movement_label = "EMERGENCY_STOP"
        if CAN_AVAILABLE and self.can_mgr:
            self.can_mgr.emergency_release_all()
        print(f"🚨 EMERGENCY STOP TRIGGERED: {reason}")

    def reset_emergency_stop(self):
        self.e_stop_triggered = False
        self.e_stop_reason = ""
        self.current_movement_label = "IDLE"
        if CAN_AVAILABLE and self.can_mgr:
            self.can_mgr.set_origin_all()

    # --- SMART ASSIST-AS-NEEDED (AAN) CONTROL PROTOCOL ---
    def trigger_smart_aan_protocol(self, movement_type: str, speed_mode: str, on_complete_callback=None):
        if self.movement_thread and self.movement_thread.is_alive():
            return False, "Movement protocol in progress!"
        if self.e_stop_triggered:
            return False, "Clear Emergency Stop first!"

        self.movement_thread = threading.Thread(
            target=self._run_smart_aan_fsm, 
            args=(movement_type, speed_mode, on_complete_callback),
            daemon=True
        )
        self.movement_thread.start()
        return True, f"Started Smart AAN: {movement_type}"

    def _run_smart_aan_fsm(self, movement_type: str, speed_mode: str, on_complete_callback=None):
        self.is_streaming = True
        self.current_movement_label = "INITIALIZING"
        time.sleep(1.0)

        # Target Speed Configuration
        is_medium = (speed_mode.lower() == "medium")
        spd_1_2 = 3000.0; acc_1_2 = 1000.0
        spd_11_12 = 1000.0 if is_medium else 300.0
        acc_11_12 = 200.0 if is_medium else 50.0

        targets = []
        if movement_type == "flexion":
            targets = [(12, -35.0, spd_11_12, acc_11_12)]
        elif movement_type == "extension":
            targets = [(12, 30.0, spd_11_12, acc_11_12)]
        elif movement_type == "ulnar":
            targets = [(11, 15.0, spd_11_12, acc_11_12)]
        elif movement_type == "radial":
            targets = [(11, -10.0, spd_11_12, acc_11_12)]
        elif movement_type == "grasping":
            targets = [
                (1, -150.0, spd_1_2, acc_1_2),
                (2, 150.0, spd_1_2, acc_1_2),
                (11, 0.0, spd_11_12, acc_11_12),
                (12, 0.0, spd_11_12, acc_11_12)
            ]

        # ----------------------------------------------------------------------
        # PHASE 1: PATIENT TRYING (5-SECOND MONITORING WINDOW)
        # ----------------------------------------------------------------------
        start_time = time.time()
        patient_reached = False
        
        while time.time() - start_time < 5.0:
            if self.e_stop_triggered: return

            remaining_sec = round(5.0 - (time.time() - start_time), 1)
            self.current_movement_label = f"TRYING_{movement_type.upper()}_{remaining_sec}S"

            # Check if patient reaches target independently
            reached_all = True
            for cid, target_angle, _, _ in targets:
                cur_pos, _, _ = self.can_mgr.get_motor_state_data(cid)
                if abs(cur_pos - target_angle) > 3.0:
                    reached_all = False
                    break

            if reached_all:
                patient_reached = True
                break

            time.sleep(0.05)

        if patient_reached:
            self.current_movement_label = "PATIENT_SUCCESS"
            time.sleep(1.0)
            self.is_streaming = False
            self.current_movement_label = "IDLE"
            if on_complete_callback: on_complete_callback()
            return

        # ----------------------------------------------------------------------
        # PHASE 2: MOTOR ASSIST (STUCK FOR 5 SECONDS -> ACTUATE)
        # ----------------------------------------------------------------------
        self.current_movement_label = f"ASSISTING_{movement_type.upper()}"
        for cid, pos, spd, acc in targets:
            self.can_mgr.set_move(cid, pos, spd, acc)
            time.sleep(0.005)

        # Monitor move and execute real-time E-Stop over-angle guard
        assist_start = time.time()
        while time.time() - assist_start < 4.0:
            if self.e_stop_triggered: return

            # REAL-TIME E-STOP OVER-ANGLE GUARD
            for cid, target_angle, _, _ in targets:
                cur_pos, _, _ = self.can_mgr.get_motor_state_data(cid)
                max_allowable_limit = abs(target_angle) + self.safety_overshoot_margin_deg
                
                if abs(cur_pos) > max_allowable_limit:
                    self.trigger_emergency_stop(f"Motor {cid} overshot limit ({cur_pos:.1f}° > {max_allowable_limit:.1f}°)")
                    return

            time.sleep(0.02)

        # ----------------------------------------------------------------------
        # PHASE 3: HOLD TARGET POSITION FOR 15 SECONDS
        # ----------------------------------------------------------------------
        for sec_left in range(15, 0, -1):
            if self.e_stop_triggered: return
            self.current_movement_label = f"ASSIST_HOLD_{sec_left}S"
            time.sleep(1.0)

        # ----------------------------------------------------------------------
        # PHASE 4: RETURN TO NEUTRAL (0°) & STOP PROGRAM
        # ----------------------------------------------------------------------
        self.current_movement_label = "RETURNING_NEUTRAL"
        for cid, _, spd, acc in targets:
            self.can_mgr.set_move(cid, 0.0, spd, acc)
            time.sleep(0.005)

        time.sleep(2.0)
        self.is_streaming = False
        self.current_movement_label = "IDLE"

        if on_complete_callback:
            on_complete_callback()

    # --- SENSOR SAMPLING LOOP ---
    def main_loop(self):
        self.initialize_hardware_inside_thread()
        target_sample_rate = 100    
        sample_interval = 1.0 / target_sample_rate  
        next_sample_time = time.perf_counter()

        while self.running:
            if self.is_streaming:
                current_time = time.perf_counter()
                if current_time < next_sample_time:
                    time_remaining = next_sample_time - current_time
                    if time_remaining > 0.005:
                        time.sleep(time_remaining - 0.002)
                    while time.perf_counter() < next_sample_time: pass

                if I2C_AVAILABLE and self.hardware_initialized:
                    try:
                        f1 = self.voltage_to_percentage(self.read_voltage(self.ch_f1), *self.flex_calib['f1'])
                        f2 = self.voltage_to_percentage(self.read_voltage(self.ch_f2), *self.flex_calib['f2'])
                        f3 = self.voltage_to_percentage(self.read_voltage(self.ch_f3), *self.flex_calib['f3'])
                        f4 = self.voltage_to_percentage(self.read_voltage(self.ch_f4), *self.flex_calib['f4'])
                        f5 = self.voltage_to_percentage(self.read_voltage(self.ch_f5), *self.flex_calib['f5'])
                        
                        emg1, emg2 = self.read_serial_emg()
                        
                        sensor = self.imu.get_sensor_data()
                        ax = sensor['accel']['x'] - self.ax_off
                        ay = sensor['accel']['y'] - self.ay_off
                        az = sensor['accel']['z'] - self.az_off
                        gx = sensor['gyro']['x'] - self.gx_off
                        gy = sensor['gyro']['y'] - self.gy_off
                        gz = sensor['gyro']['z'] - self.gz_off
                    except Exception:
                        f1 = f2 = f3 = f4 = f5 = 0.0
                        emg1 = emg2 = 0
                        ax = ay = az = gx = gy = gz = 0.0
                else:
                    f1, f2, f3, f4, f5 = [round(random.uniform(10.0, 85.0), 1) for _ in range(5)]
                    ax, ay, az = [round(random.uniform(-0.8, 0.8), 2) for _ in range(3)]
                    gx, gy, gz = [round(random.uniform(-120.0, 120.0), 1) for _ in range(3)]
                    emg1 = random.randint(100, 400); emg2 = random.randint(100, 350)

                m1_pos, m1_spd, m1_acc = self.can_mgr.get_motor_state_data(1)
                m2_pos, m2_spd, m2_acc = self.can_mgr.get_motor_state_data(2)
                m11_pos, m11_spd, m11_acc = self.can_mgr.get_motor_state_data(11)
                m12_pos, m12_spd, m12_acc = self.can_mgr.get_motor_state_data(12)

                telemetry_frame = {
                    'movement_label': self.current_movement_label,
                    'flex': [f1, f2, f3, f4, f5],
                    'emg': [emg1, emg2],
                    'accel': [ax, ay, az],
                    'gyro': [gx, gy, gz],
                    'm1': [m1_pos, m1_spd, m1_acc],
                    'm2': [m2_pos, m2_spd, m2_acc],
                    'm11': [m11_pos, m11_spd, m11_acc],
                    'm12': [m12_pos, m12_spd, m12_acc]
                }
                self.data_queue.put(telemetry_frame)
                next_sample_time += sample_interval
            else:
                time.sleep(0.05)
                next_sample_time = time.perf_counter()