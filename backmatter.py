import time
import random
import threading
import subprocess
import os
import struct
import csv
from enum import IntEnum
import numpy as np
from collections import deque
import joblib

from scipy.signal import butter, filtfilt
import tensorflow as tf

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


# ==============================================================================
# CUBEMARS CAN BUS PROTOCOL MANAGER
# ==============================================================================
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
        """Unlocks all motors back to unpowered zero torque / free movement."""
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
            telem = self.telemetry.get(cid, {})
            pos = telem.get("position_deg", 0.0)
            spd = telem.get("speed_erpm", 0.0)
            target_pos = st.get("target_pos", 0.0)
            return pos, spd, target_pos


# ==============================================================================
# SENSOR ACQUISITION WORKER
# ==============================================================================
class SensorAcquisitionWorker:
    def __init__(self, data_queue, model_path="multiLSTM-LOSO.keras", scaler_path="feature_scaler.pkl"):
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

        # --- MODEL & SCALER SETTINGS ---
        self.auto_lstm_enabled = False
        self.model_path = model_path
        self.scaler_path = scaler_path
        self.wrist_scaler_path = "wrist_target_scaler.pkl"
        self.finger_scaler_path = "finger_target_scaler.pkl"

        self.model = None
        self.feature_scaler = None
        self.wrist_scaler = None
        self.finger_scaler = None
        self.class_labels = ['Extension', 'Flexion', 'Grasping', 'Radial', 'Ulnar']
        
        # Buffer of 20 samples to ensure stable filtfilt padding
        self.raw_emg_buffer_f = deque(maxlen=20)
        self.raw_emg_buffer_e = deque(maxlen=20)
        
        # 6-step temporal sequence buffer matching seq_len=6
        self.sequence_tensor_buffer = deque(maxlen=6)

        # OPTIMIZED REAL-TIME THRESHOLDS
        self.class_thresholds = {
            'Extension': 0.45,  
            'Flexion':   0.65,  
            'Grasping':  0.68,  
            'Radial':    0.50,  
            'Ulnar':     0.65
        }

        self.emg_activation_threshold = 30.0   
        self.cooldown_until = 0.0             

        self.e_stop_triggered = False
        self.e_stop_reason = ""
        self.safety_overshoot_margin_deg = 5.0
        self.results_csv_path = "live_test_results.csv"

        self.init_tf_model()
        self.init_feature_scaler()
        self.init_target_scalers()
        self._ensure_csv_header()

    def _ensure_csv_header(self):
        """Ensures live_test_results.csv exists with structured header columns."""
        if not os.path.exists(self.results_csv_path):
            try:
                with open(self.results_csv_path, mode='w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        'Timestamp', 'Movement', 'Samples', 
                        'Wrist_MAE_eRPM', 'Wrist_RMSE_eRPM', 
                        'Finger_MAE_eRPM', 'Finger_RMSE_eRPM', 
                        'Overall_MAE_eRPM', 'Overall_RMSE_eRPM'
                    ])
                print(f"📁 Initialized test results log at '{self.results_csv_path}'")
            except Exception as e:
                print(f"⚠️ Error creating CSV header: {e}")

    def init_tf_model(self):
        if os.path.exists(self.model_path):
            try:
                self.model = tf.keras.models.load_model(self.model_path)
                print(f"🧠 TensorFlow Model '{self.model_path}' loaded successfully!")
            except Exception as e:
                print(f"⚠️ Error loading Keras model: {e}")
        else:
            print(f"⚠️ Model file '{self.model_path}' not found in directory.")

    def init_feature_scaler(self):
        if os.path.exists(self.scaler_path):
            try:
                self.feature_scaler = joblib.load(self.scaler_path)
                print(f"⚙️ Feature MinMaxScaler '{self.scaler_path}' loaded successfully!")
            except Exception as e:
                print(f"⚠️ Error loading feature scaler: {e}")
        else:
            print(f"⚠️ Scaler file '{self.scaler_path}' not found. Using raw feature vectors.")

    def init_target_scalers(self):
        if os.path.exists(self.wrist_scaler_path) and os.path.exists(self.finger_scaler_path):
            try:
                self.wrist_scaler = joblib.load(self.wrist_scaler_path)
                self.finger_scaler = joblib.load(self.finger_scaler_path)
                print("⚙️ Target Scalers for Motor Speed/Accel loaded successfully!")
            except Exception as e:
                print(f"⚠️ Error loading target scalers: {e}")
        else:
            print("⚠️ Target scalers missing. Falling back to preset motor speeds.")

    def toggle_auto_lstm_control(self, enable: bool):
        self.auto_lstm_enabled = enable
        if enable:
            self.calibrate_imu_bias()
            self.current_movement_label = "LSTM_MONITORING"
            self.is_streaming = True
        else:
            self.auto_lstm_enabled = False
            self.is_streaming = False
            self.current_movement_label = "IDLE"
            if CAN_AVAILABLE and self.can_mgr:
                self.can_mgr.emergency_release_all()

    # --- EXACT PREPROCESSING PIPELINE FROM TRAINING CODE ---
    def apply_semg_filter(self, raw_buffer, fs=36.0):
        if len(raw_buffer) < 10:
            return np.array(raw_buffer, dtype=np.float32)

        data = np.array(raw_buffer, dtype=np.float32)
        centered_data = data - np.mean(data)

        nyq = 0.5 * fs
        low_cutoff = 1.0 / nyq
        high_cutoff = min(15.0 / nyq, 0.99)

        try:
            b, a = butter(2, [low_cutoff, high_cutoff], btype='bandpass')
            pad_len = min(len(centered_data) - 1, 6)
            filtered_data = filtfilt(b, a, centered_data, padlen=pad_len)
        except Exception:
            filtered_data = centered_data

        return filtered_data

    def extract_enhanced_td_features(self, w_flex, w_ext, zc_thresh=1e-5, ssc_thresh=1e-5, eps=1e-6):
        """Matches extract_enhanced_td_features() in training code exactly."""
        rms_f, rms_e = np.sqrt(np.mean(w_flex**2)), np.sqrt(np.mean(w_ext**2))
        mav_f, mav_e = np.mean(np.abs(w_flex)), np.mean(np.abs(w_ext))
        wl_f, wl_e = np.sum(np.abs(np.diff(w_flex))), np.sum(np.abs(np.diff(w_ext)))
        
        zc_f = np.sum((w_flex[:-1] * w_flex[1:] < 0) & (np.abs(w_flex[:-1] - w_flex[1:]) >= zc_thresh))
        zc_e = np.sum((w_ext[:-1] * w_ext[1:] < 0) & (np.abs(w_ext[:-1] - w_ext[1:]) >= zc_thresh))
        
        var_f, var_e = np.var(w_flex), np.var(w_ext)
        
        diff_f, diff_e = np.diff(w_flex), np.diff(w_ext)
        ssc_f = np.sum((diff_f[:-1] * diff_f[1:] < 0) & (np.abs(diff_f[:-1] - diff_f[1:]) >= ssc_thresh))
        ssc_e = np.sum((diff_e[:-1] * diff_e[1:] < 0) & (np.abs(diff_e[:-1] - diff_e[1:]) >= ssc_thresh))
        
        log_ratio = np.log((rms_f + eps) / (rms_e + eps))
        
        return [
            rms_f, rms_e, mav_f, mav_e, wl_f, wl_e, zc_f, zc_e,
            var_f, var_e, ssc_f, ssc_e, log_ratio
        ]

    def run_inference_step(self, flex_data, imu_acc, imu_gyr):
        if len(self.raw_emg_buffer_f) < 10 or self.model is None:
            return None, 0.0, None

        # Filter last 10 samples (win_len = 0.3s * 36 Hz = 10)
        raw_f_win = list(self.raw_emg_buffer_f)[-10:]
        raw_e_win = list(self.raw_emg_buffer_e)[-10:]

        filt_f = self.apply_semg_filter(raw_f_win, fs=36.0)
        filt_e = self.apply_semg_filter(raw_e_win, fs=36.0)

        # 13 TD Features
        td_feats = self.extract_enhanced_td_features(filt_f, filt_e)

        # 24 Total Features (13 TD + 5 Flex + 6 IMU)
        unscaled_frame = np.array(td_feats + flex_data + imu_acc + imu_gyr, dtype=np.float32).reshape(1, -1)

        # Apply MinMaxScaler transform
        if self.feature_scaler is not None:
            scaled_frame = self.feature_scaler.transform(unscaled_frame)[0]
        else:
            scaled_frame = unscaled_frame[0]

        self.sequence_tensor_buffer.append(scaled_frame)

        if len(self.sequence_tensor_buffer) < 6:
            return None, 0.0, None

        input_data = np.expand_dims(np.array(self.sequence_tensor_buffer, dtype=np.float32), axis=0)
        outputs = self.model(input_data, training=False)
        
        # UNPACK MULTI-TASK OUTPUTS (Classification + Motor Regression)
        if isinstance(outputs, list):
            class_probs = outputs[0].numpy()[0]          # Branch 1: Classification (Softmax)
            pred_motor_scaled = outputs[1].numpy()[0]    # Branch 2: Motor Dynamics Regression (8 targets)
        else:
            class_probs = outputs.numpy()[0]
            pred_motor_scaled = None

        # UN-SCALE PREDICTED MOTOR SPEEDS AND ACCELERATIONS
        predicted_dynamics = None
        if pred_motor_scaled is not None and self.wrist_scaler is not None and self.finger_scaler is not None:
            wrist_real = self.wrist_scaler.inverse_transform(pred_motor_scaled[:4].reshape(1, -1))[0]
            finger_real = self.finger_scaler.inverse_transform(pred_motor_scaled[4:].reshape(1, -1))[0]
            # Unpacked array: [M1_Spd, M1_Acc, M2_Spd, M2_Acc, M11_Spd, M11_Acc, M12_Spd, M12_Acc]
            predicted_dynamics = np.concatenate([wrist_real, finger_real])

        predicted_idx = np.argmax(class_probs)
        confidence = float(class_probs[predicted_idx])
        predicted_label = self.class_labels[predicted_idx]

        required_thresh = self.class_thresholds.get(predicted_label, 0.60)

        probs_str = " | ".join([f"{lbl[:3]}:{class_probs[i]:.2f}" for i, lbl in enumerate(self.class_labels)])
        if predicted_dynamics is not None:
            print(f"📊 [PROBS] {probs_str} -> Top: {predicted_label} ({confidence:.2f} vs Req: {required_thresh:.2f}) | Pred M12 Spd: {predicted_dynamics[6]:.1f} eRPM")
        else:
            print(f"📊 [PROBS] {probs_str} -> Top: {predicted_label} ({confidence:.2f} vs Req: {required_thresh:.2f})")

        if confidence >= required_thresh:
            return predicted_label, confidence, predicted_dynamics

        return None, 0.0, None

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

    def calibrate_imu_bias(self, samples=30):
        if not I2C_AVAILABLE or not self.hardware_initialized:
            return
        ax_s = ay_s = az_s = gx_s = gy_s = gz_s = 0.0
        valid = 0
        for _ in range(samples):
            try:
                s = self.imu.get_sensor_data()
                ax_s += s['accel']['x']; ay_s += s['accel']['y']; az_s += s['accel']['z']
                gx_s += s['gyro']['x']; gy_s += s['gyro']['y']; gz_s += s['gyro']['z']
                valid += 1
                time.sleep(0.005)
            except Exception: pass
        if valid > 0:
            self.ax_off = ax_s / valid
            self.ay_off = ay_s / valid
            self.az_off = (az_s / valid) - 1.0
            self.gx_off = gx_s / valid
            self.gy_off = gy_s / valid
            self.gz_off = gz_s / valid

    def start_test_stream(self):
        if self.movement_thread and self.movement_thread.is_alive():
            return False, "Movement protocol in progress!"
        self.calibrate_imu_bias()
        self.current_movement_label = "SENSOR_TEST"
        self.is_streaming = True
        return True, "Test Stream Started"

    def stop_test_stream(self):
        self.is_streaming = False
        self.current_movement_label = "IDLE"
        if CAN_AVAILABLE and self.can_mgr:
            self.can_mgr.emergency_release_all()
        return True, "Test Stream Stopped"

    def _trigger_internal_overangle_stop(self, reason: str):
        self.e_stop_triggered = True
        self.e_stop_reason = reason
        self.current_movement_label = "SAFETY_CUTOFF_OVERSHOOT"
        if CAN_AVAILABLE and self.can_mgr:
            self.can_mgr.emergency_release_all()
        print(f"🚨 AUTOMATIC OVER-ANGLE GUARD TRIGGERED: {reason}")

    def auto_trigger_smart_aan(self, detected_gesture: str, speed_mode='Slow', dyn_targets=None):
        if self.movement_thread and self.movement_thread.is_alive():
            return

        self.e_stop_triggered = False
        self.e_stop_reason = ""

        self.movement_thread = threading.Thread(
            target=self._run_smart_aan_fsm, 
            args=(detected_gesture.lower(), speed_mode, dyn_targets),
            daemon=True
        )
        self.movement_thread.start()

    def _return_to_zero_and_finish(self, targets):
        """Drives target motors back to neutral 0.0° and locks holding torque until arrived before releasing."""
        self.current_movement_label = "RETURNING_NEUTRAL"
        for cid, _, spd, acc in targets:
            self.can_mgr.set_move(cid, 0.0, spd, acc)
            time.sleep(0.005)

        return_start = time.time()
        max_return_timeout = 6.0
        
        while time.time() - return_start < max_return_timeout:
            if self.e_stop_triggered:
                break
            
            all_motors_at_zero = True
            for cid, _, _, _ in targets:
                cur_pos, _, _ = self.can_mgr.get_motor_state_data(cid)
                if abs(cur_pos) > 2.5:
                    all_motors_at_zero = False
                    break

            if all_motors_at_zero:
                print("✅ All motors arrived back at neutral (0.0°). Releasing holding torque.")
                break
                
            time.sleep(0.02)

        self._finish_and_set_cooldown()

    def _run_smart_aan_fsm(self, movement_type: str, speed_mode: str, dyn_targets=None):
        self.current_movement_label = f"DETECTED:{movement_type.upper()}"
        time.sleep(0.5)

        is_medium = (speed_mode.lower() == "medium")
        spd_1_2 = 3000.0; acc_1_2 = 1000.0
        spd_11_12 = 1000.0 if is_medium else 300.0
        acc_11_12 = 200.0 if is_medium else 50.0

        # DYNAMIC SPEED & ACCELERATION OVERRIDE FROM MODEL REGRESSION HEAD
        if dyn_targets is not None and len(dyn_targets) == 8:
            m1_spd, m1_acc = max(100.0, float(abs(dyn_targets[0]))), max(50.0, float(abs(dyn_targets[1])))
            m2_spd, m2_acc = max(100.0, float(abs(dyn_targets[2]))), max(50.0, float(abs(dyn_targets[3])))
            m11_spd, m11_acc = max(100.0, float(abs(dyn_targets[4]))), max(50.0, float(abs(dyn_targets[5])))
            m12_spd, m12_acc = max(100.0, float(abs(dyn_targets[6]))), max(50.0, float(abs(dyn_targets[7])))
        else:
            m1_spd = m2_spd = spd_1_2
            m1_acc = m2_acc = acc_1_2
            m11_spd = m12_spd = spd_11_12
            m11_acc = m12_acc = acc_11_12

        targets = []
        if movement_type == "flexion":
            targets = [(12, -35.0, m12_spd, m12_acc)]
        elif movement_type == "extension":
            targets = [(12, 30.0, m12_spd, m12_acc)]
        elif movement_type == "ulnar":
            targets = [(11, 15.0, m11_spd, m11_acc)]
        elif movement_type == "radial":
            targets = [(11, -10.0, m11_spd, m11_acc)]
        elif movement_type == "grasping":
            targets = [
                (1, -150.0, m1_spd, m1_acc),
                (2, 150.0, m2_spd, m2_acc),
                (11, 0.0, m11_spd, m11_acc),
                (12, 0.0, m12_spd, m12_acc)
            ]

        # TRACKING BUFFERS FOR LIVE ERROR CALCULATION
        wrist_true_spds, wrist_pred_spds = [], []
        finger_true_spds, finger_pred_spds = [], []

        # PHASE 1: PATIENT TRYING (5s WINDOW)
        start_time = time.time()
        patient_reached = False
        
        while time.time() - start_time < 5.0:
            if self.e_stop_triggered: 
                self._finish_and_set_cooldown()
                return

            remaining_sec = round(5.0 - (time.time() - start_time), 1)
            self.current_movement_label = f"TRYING_{movement_type.upper()}_{remaining_sec}S"

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

        # IF PATIENT REACHED GOAL, RETURN TO NEUTRAL (0°) THEN UNLOCK
        if patient_reached:
            self.current_movement_label = "PATIENT_SUCCESS"
            time.sleep(1.0)
            self._return_to_zero_and_finish(targets)
            return

        # PHASE 2: MOTOR ASSIST WITH LIVE MAE/RMSE SAMPLING
        self.current_movement_label = f"ASSISTING_{movement_type.upper()}"
        for cid, pos, spd, acc in targets:
            self.can_mgr.set_move(cid, pos, spd, acc)
            time.sleep(0.005)

        assist_start = time.time()
        while time.time() - assist_start < 4.0:
            if self.e_stop_triggered: 
                self._finish_and_set_cooldown()
                return

            for cid, target_angle, pred_spd, _ in targets:
                cur_pos, cur_spd, _ = self.can_mgr.get_motor_state_data(cid)
                
                # Sample actual vs predicted speeds per actuator group
                if cid in [1, 2]:
                    wrist_true_spds.append(abs(cur_spd))
                    wrist_pred_spds.append(abs(pred_spd))
                elif cid in [11, 12]:
                    finger_true_spds.append(abs(cur_spd))
                    finger_pred_spds.append(abs(pred_spd))

                max_allowable_limit = abs(target_angle) + self.safety_overshoot_margin_deg
                if abs(cur_pos) > max_allowable_limit:
                    self._trigger_internal_overangle_stop(
                        f"Motor {cid} position ({cur_pos:.1f}°) exceeded target limit ({max_allowable_limit:.1f}°)"
                    )
                    self._finish_and_set_cooldown()
                    return

            time.sleep(0.02)  # Sample live telemetry at 50 Hz

        # COMPUTE & LOG LIVE MAE & RMSE AT END OF ASSIST PHASE
        self._calculate_and_log_test_metrics(
            movement_type, 
            wrist_true_spds, wrist_pred_spds, 
            finger_true_spds, finger_pred_spds
        )

        # PHASE 3: HOLD TARGET POSITION FOR 15 SECONDS
        for sec_left in range(15, 0, -1):
            if self.e_stop_triggered: 
                self._finish_and_set_cooldown()
                return
            
            for cid, target_angle, _, _ in targets:
                cur_pos, _, _ = self.can_mgr.get_motor_state_data(cid)
                if abs(cur_pos) > (abs(target_angle) + self.safety_overshoot_margin_deg):
                    self._trigger_internal_overangle_stop(f"Motor {cid} drift exceeded limit during hold")
                    self._finish_and_set_cooldown()
                    return

            self.current_movement_label = f"ASSIST_HOLD_{sec_left}S"
            time.sleep(1.0)

        # PHASE 4: RETURN TO NEUTRAL (0°) AND LOCK UNTIL ARRIVED BEFORE UNLOCKING
        self._return_to_zero_and_finish(targets)

    def _calculate_and_log_test_metrics(self, movement_type, w_true, w_pred, f_true, f_pred):
        """Calculates MAE/RMSE across actuator groups and logs to CSV."""
        w_t = np.array(w_true, dtype=np.float32)
        w_p = np.array(w_pred, dtype=np.float32)
        f_t = np.array(f_true, dtype=np.float32)
        f_p = np.array(f_pred, dtype=np.float32)

        w_mae = np.mean(np.abs(w_t - w_p)) if len(w_t) > 0 else 0.0
        w_rmse = np.sqrt(np.mean((w_t - w_p)**2)) if len(w_t) > 0 else 0.0

        f_mae = np.mean(np.abs(f_t - f_p)) if len(f_t) > 0 else 0.0
        f_rmse = np.sqrt(np.mean((f_t - f_p)**2)) if len(f_t) > 0 else 0.0

        all_true = np.concatenate([w_t, f_t]) if len(w_t) > 0 or len(f_t) > 0 else np.array([])
        all_pred = np.concatenate([w_p, f_p]) if len(w_p) > 0 or len(f_p) > 0 else np.array([])

        tot_mae = np.mean(np.abs(all_true - all_pred)) if len(all_true) > 0 else 0.0
        tot_rmse = np.sqrt(np.mean((all_true - all_pred)**2)) if len(all_true) > 0 else 0.0
        sample_count = len(all_true)

        # Print Summary Report to Terminal
        print("\n" + "="*58)
        print(f"📊 LIVE PERFORMANCE TEST RESULTS ({movement_type.upper()})")
        print("="*58)
        print(f" Samples Evaluated : {sample_count}")
        print(f" AK80-8  (Wrist)   : MAE = {w_mae:6.2f} eRPM | RMSE = {w_rmse:6.2f} eRPM")
        print(f" AK40-10 (Finger)  : MAE = {f_mae:6.2f} eRPM | RMSE = {f_rmse:6.2f} eRPM")
        print(f" OVERALL COMBINED  : MAE = {tot_mae:6.2f} eRPM | RMSE = {tot_rmse:6.2f} eRPM")
        print("="*58 + "\n")

        # Append directly to CSV file
        try:
            timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
            with open(self.results_csv_path, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    timestamp, movement_type.upper(), sample_count,
                    round(w_mae, 2), round(w_rmse, 2),
                    round(f_mae, 2), round(f_rmse, 2),
                    round(tot_mae, 2), round(tot_rmse, 2)
                ])
            print(f"💾 Metrics recorded to '{self.results_csv_path}'")
        except Exception as e:
            print(f"⚠️ Error logging to CSV: {e}")

    def _finish_and_set_cooldown(self):
        """Flushes buffers, unlocks motor torque to zero, and activates 3.0s cooldown."""
        if CAN_AVAILABLE and self.can_mgr:
            self.can_mgr.emergency_release_all()  
            
        self.raw_emg_buffer_f.clear()
        self.raw_emg_buffer_e.clear()
        self.sequence_tensor_buffer.clear()
        self.cooldown_until = time.time() + 3.0
        self.current_movement_label = "LSTM_MONITORING" if self.auto_lstm_enabled else "IDLE"

    def voltage_to_percentage(self, volts, v_min, v_max):
        if volts <= v_min: return 0.0
        if volts >= v_max: return 100.0
        return round(((volts - v_min) / (v_max - v_min)) * 100.0, 1)

    def read_serial_emg(self):
        if not self.ser or not hasattr(self.ser, 'is_open') or not self.ser.is_open:
            return self.last_emg
        try:
            latest_line = None
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line and not line.startswith('#'):
                    latest_line = line
            if latest_line:
                parts = latest_line.split(',')
                if len(parts) == 3:
                    raw_f = int(parts[1])
                    raw_e = int(parts[2])
                    self.last_emg = [raw_f if raw_f >= 25 else 0, raw_e if raw_e >= 25 else 0]
        except Exception: pass
        return self.last_emg

    def read_voltage(self, analog_channel_object):
        if not I2C_AVAILABLE or not self.hardware_initialized: return 0.0
        try: return analog_channel_object.voltage
        except Exception: return 0.0

    def main_loop(self):
        self.initialize_hardware_inside_thread()
        target_sample_rate = 36    
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

                self.raw_emg_buffer_f.append(emg1)
                self.raw_emg_buffer_e.append(emg2)

                now = time.time()
                if self.auto_lstm_enabled and (self.movement_thread is None or not self.movement_thread.is_alive()) and now > self.cooldown_until:
                    raw_f_win = list(self.raw_emg_buffer_f)[-10:]
                    raw_e_win = list(self.raw_emg_buffer_e)[-10:]
                    
                    filt_f = self.apply_semg_filter(raw_f_win, fs=36.0)
                    filt_e = self.apply_semg_filter(raw_e_win, fs=36.0)

                    rms_f = np.sqrt(np.mean(filt_f**2)) if len(filt_f) >= 10 else 0
                    rms_e = np.sqrt(np.mean(filt_e**2)) if len(filt_e) >= 10 else 0
                    
                    if (rms_f > self.emg_activation_threshold or rms_e > self.emg_activation_threshold):
                        pred_class, confidence, pred_dynamics = self.run_inference_step([f1, f2, f3, f4, f5], [ax, ay, az], [gx, gy, gz])
                        if pred_class and confidence > 0.0:
                            print(f"🎯 MODEL DETECTED: {pred_class} ({confidence*100:.1f}%) -> Auto-Starting AAN")
                            self.auto_trigger_smart_aan(pred_class, dyn_targets=pred_dynamics)

                m1_pos, m1_spd, m1_tgt = self.can_mgr.get_motor_state_data(1)
                m2_pos, m2_spd, m2_tgt = self.can_mgr.get_motor_state_data(2)
                m11_pos, m11_spd, m11_tgt = self.can_mgr.get_motor_state_data(11)
                m12_pos, m12_spd, m12_tgt = self.can_mgr.get_motor_state_data(12)

                telemetry_frame = {
                    'movement_label': self.current_movement_label,
                    'flex': [f1, f2, f3, f4, f5],
                    'emg': [emg1, emg2],
                    'accel': [ax, ay, az],
                    'gyro': [gx, gy, gz],
                    'm1': [m1_pos, m1_spd, m1_tgt],
                    'm2': [m2_pos, m2_spd, m2_tgt],
                    'm11': [m11_pos, m11_spd, m11_tgt],
                    'm12': [m12_pos, m12_spd, m12_tgt]
                }
                self.data_queue.put(telemetry_frame)
                next_sample_time += sample_interval
            else:
                time.sleep(0.05)
                next_sample_time = time.perf_counter()