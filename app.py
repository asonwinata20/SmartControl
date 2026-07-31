import kivy
from kivy.config import Config
Config.set('graphics', 'width', '1280')
Config.set('graphics', 'height', '720')
Config.set('graphics', 'resizable', False)

from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.uix.spinner import Spinner
from kivy.clock import Clock
from kivy.graphics import Color, Rectangle, Line
from kivy.uix.popup import Popup
from kivy.uix.scrollview import ScrollView

import threading
import queue
import os    
import csv   
import time  
from collections import deque

from backmatter import SensorAcquisitionWorker

class TelemetryDashboard(App):
    def build(self):
        self.telemetry_queue = queue.Queue()
        self.points_count = 100 
        
        self.buffers = [deque([0.0] * self.points_count, maxlen=self.points_count) for _ in range(13)]
        self.flex_peaks = [0.0] * 5
        self.emg_peaks = [0] * 2
        
        self.session_data_log = []
        self.recording_start_time = 0.0
        self.project_directory = os.path.dirname(os.path.abspath(__file__))
        
        root_layout = BoxLayout(orientation='horizontal', spacing=5)
        with root_layout.canvas.before:
            Color(0.09, 0.09, 0.10, 1)
            self.bg_rect = Rectangle(size=(1280, 720), pos=(0, 0))
        root_layout.bind(size=self.update_rect, pos=self.update_rect)

        # CONTROL SIDEBAR
        sidebar_scroll = ScrollView(size_hint=(0.22, 1), do_scroll_x=False, do_scroll_y=True)
        sidebar = BoxLayout(orientation='vertical', size_hint_y=None, padding=10, spacing=8)
        sidebar.bind(minimum_height=sidebar.setter('height'))
        
        with sidebar.canvas.before:
            Color(0.14, 0.15, 0.17, 1)
            self.sidebar_bg = Rectangle()
        sidebar.bind(size=self.update_sidebar_bg, pos=self.update_sidebar_bg)

        # EMERGENCY STOP BUTTON
        self.btn_estop = Button(
            text="🚨 EMERGENCY STOP 🚨", font_size='12sp', bold=True, size_hint_y=None, height=40,
            background_color=(0.9, 0.1, 0.1, 1)
        )
        self.btn_estop.bind(on_press=self.trigger_estop_button)
        sidebar.add_widget(self.btn_estop)

        sidebar.add_widget(Label(text="SYSTEM CONTROL", font_size='14sp', bold=True, size_hint_y=None, height=25))

        self.btn_test_start = Button(text="Start Test Stream", font_size='12sp', size_hint_y=None, height=34, background_color=(0.2, 0.7, 0.9, 1))
        self.btn_test_stop = Button(text="Stop Test Stream", font_size='12sp', size_hint_y=None, height=34, disabled=True, background_color=(0.9, 0.4, 0.3, 1))
        self.btn_set_origin = Button(text="Set Motor Origin (0°)", font_size='12sp', size_hint_y=None, height=34, background_color=(0.9, 0.7, 0.2, 1))
        self.btn_check = Button(text="Reinit Hardware", font_size='12sp', size_hint_y=None, height=34, background_color=(0.3, 0.5, 0.7, 1))
        
        self.lbl_status_indicator = Label(text="STATUS: IDLE", font_size='11sp', bold=True, size_hint_y=None, height=25, color=(0.6, 0.6, 0.6, 1))
        
        self.btn_test_start.bind(on_press=self.trigger_test_start)
        self.btn_test_stop.bind(on_press=self.trigger_test_stop)
        self.btn_set_origin.bind(on_press=self.trigger_set_origin)
        self.btn_check.bind(on_press=self.trigger_reinit_hardware)
        
        sidebar.add_widget(self.btn_test_start)
        sidebar.add_widget(self.btn_test_stop)
        sidebar.add_widget(self.btn_set_origin)
        sidebar.add_widget(self.btn_check)
        sidebar.add_widget(self.lbl_status_indicator)

        # SMART AAN PROTOCOL SECTION
        sidebar.add_widget(Label(text="SMART AAN PROTOCOLS", font_size='14sp', bold=True, size_hint_y=None, height=30, color=(1, 0.8, 0.2, 1)))
        
        sidebar.add_widget(Label(text="Speed Preset:", font_size='11sp', size_hint_y=None, height=20, halign='left'))
        self.spn_speed = Spinner(
            text='Slow',
            values=('Slow', 'Medium'),
            size_hint_y=None, height=35,
            background_color=(0.3, 0.4, 0.5, 1)
        )
        sidebar.add_widget(self.spn_speed)

        self.btn_flexion = Button(text="Flexion AAN", font_size='12sp', size_hint_y=None, height=34)
        self.btn_extension = Button(text="Extension AAN", font_size='12sp', size_hint_y=None, height=34)
        self.btn_ulnar = Button(text="Ulnar Deviation AAN", font_size='12sp', size_hint_y=None, height=34)
        self.btn_radial = Button(text="Radial Deviation AAN", font_size='12sp', size_hint_y=None, height=34)
        self.btn_grasping = Button(text="Grasping AAN", font_size='12sp', size_hint_y=None, height=34, background_color=(0.2, 0.7, 0.4, 1))

        self.btn_flexion.bind(on_press=lambda x: self.trigger_smart_movement("flexion"))
        self.btn_extension.bind(on_press=lambda x: self.trigger_smart_movement("extension"))
        self.btn_ulnar.bind(on_press=lambda x: self.trigger_smart_movement("ulnar"))
        self.btn_radial.bind(on_press=lambda x: self.trigger_smart_movement("radial"))
        self.btn_grasping.bind(on_press=lambda x: self.trigger_smart_movement("grasping"))

        sidebar.add_widget(self.btn_flexion)
        sidebar.add_widget(self.btn_extension)
        sidebar.add_widget(self.btn_ulnar)
        sidebar.add_widget(self.btn_radial)
        sidebar.add_widget(self.btn_grasping)

        self.lbl_movement_indicator = Label(
            text="ACTION: IDLE", font_size='11sp', bold=True, size_hint_y=None, height=30, color=(0.2, 1.0, 0.4, 1)
        )
        sidebar.add_widget(self.lbl_movement_indicator)

        sidebar_scroll.add_widget(sidebar)
        root_layout.add_widget(sidebar_scroll)

        # MAIN WORKSPACE AREA
        right_workspace = BoxLayout(orientation='vertical', size_hint=(0.78, 1), padding=10, spacing=6)
        
        scroll_window = ScrollView(size_hint=(1, 0.73), do_scroll_x=False, do_scroll_y=True)
        dashboard_grid = GridLayout(cols=3, spacing=8, size_hint_y=None)
        dashboard_grid.bind(minimum_height=dashboard_grid.setter('height'))
        
        graph_meta = [
            {"label": "F1 - PINKY (%)", "color": (1, 0.2, 0.2, 1)},
            {"label": "F2 - RING (%)", "color": (1, 0.6, 0.1, 1)},
            {"label": "F3 - MIDDLE (%)", "color": (1, 0.9, 0.1, 1)},
            {"label": "F4 - INDEX (%)", "color": (0.2, 0.9, 0.2, 1)},
            {"label": "F5 - THUMB (%)", "color": (0.1, 0.7, 1, 1)},
            {"label": "EMG 1 - FLEXOR (RAW)", "color": (1, 0.1, 1, 1)},
            {"label": "EMG 2 - EXTENSOR (RAW)", "color": (0.8, 0.2, 1, 1)},
            {"label": "ACCELEROMETER - X (g)", "color": (0.3, 0.9, 0.6, 1)},
            {"label": "ACCELEROMETER - Y (g)", "color": (0.8, 0.4, 1, 1)},
            {"label": "ACCELEROMETER - Z (g)", "color": (0.9, 0.8, 0.6, 1)},
            {"label": "GYROSCOPE - X (dps)", "color": (0.4, 0.7, 1, 1)},
            {"label": "GYROSCOPE - Y (dps)", "color": (0.7, 0.4, 1, 1)},
            {"label": "GYROSCOPE - Z (dps)", "color": (1, 0.4, 0.7, 1)}
        ]
        
        self.graph_areas = []
        self.plot_lines = []
        self.plot_bgs = []
        
        for meta in graph_meta:
            card = BoxLayout(orientation='vertical', padding=3, size_hint_y=None, height=160)
            card_title = Label(text=meta["label"], font_size='11sp', bold=True, size_hint=(1, 0.15), halign='left')
            card_title.bind(size=card_title.setter('text_size'))
            
            plot_box = BoxLayout(size_hint=(1, 0.85))
            with plot_box.canvas.before:
                Color(0.05, 0.05, 0.06, 1)
                bg_rect = Rectangle()
                self.plot_bgs.append(bg_rect)
            with plot_box.canvas.after:
                Color(*meta["color"])
                line_trace = Line(width=1.8, points=[])
                self.plot_lines.append(line_trace)
                
            card.add_widget(card_title)
            card.add_widget(plot_box)
            dashboard_grid.add_widget(card)
            self.graph_areas.append(plot_box)
            
        scroll_window.add_widget(dashboard_grid)
        right_workspace.add_widget(scroll_window)
        
        # TELEMETRY DISPLAY PANEL
        data_panel = GridLayout(cols=4, size_hint=(1, 0.27), padding=6, spacing=6)
        with data_panel.canvas.before:
            Color(0.12, 0.13, 0.15, 1)
            self.panel_bg = Rectangle()
        data_panel.bind(size=self.update_panel_bg, pos=self.update_panel_bg)
        
        self.lbl_hand_data = Label(text="FLEX TRACKING:\nF1: --% | F2: --%\nF3: --% | F4: --%\nF5: --%", font_size='10sp', bold=True, halign='left', valign='middle')
        self.lbl_hand_data.bind(size=self.lbl_hand_data.setter('text_size'))
        
        self.lbl_emg_data = Label(text="EMG PROFILES:\nFLEXOR: -- (--)\nEXTENSOR: -- (--)", font_size='10sp', bold=True, halign='left', valign='middle', color=(1.0, 0.4, 1.0, 1))
        self.lbl_emg_data.bind(size=self.lbl_emg_data.setter('text_size'))
        
        self.lbl_motion_data = Label(text="6-AXIS IMU METRICS:\nACC: X:--g | Y:--g | Z:--g\nGYR: X:--dps | Y:--dps | Z:--dps", font_size='10sp', bold=True, halign='left', valign='middle', color=(0.0, 0.8, 1.0, 1))
        self.lbl_motion_data.bind(size=self.lbl_motion_data.setter('text_size'))

        self.lbl_motor_data = Label(text="CUBEMARS POSITIONS:\nM1: --° | M2: --°\nM11: --° | M12: --°", font_size='10sp', bold=True, halign='left', valign='middle', color=(1.0, 0.8, 0.2, 1))
        self.lbl_motor_data.bind(size=self.lbl_motor_data.setter('text_size'))

        data_panel.add_widget(self.lbl_hand_data)
        data_panel.add_widget(self.lbl_emg_data)
        data_panel.add_widget(self.lbl_motion_data)
        data_panel.add_widget(self.lbl_motor_data)
        right_workspace.add_widget(data_panel)
        
        root_layout.add_widget(right_workspace)
        return root_layout

    def on_start(self):
        self.worker = SensorAcquisitionWorker(self.telemetry_queue)
        self.worker_thread = threading.Thread(target=self.worker.main_loop)
        self.worker_thread.daemon = True
        self.worker_thread.start()
        Clock.schedule_interval(self.update_telemetry_ui, 0.016)

    def trigger_estop_button(self, instance):
        self.worker.trigger_emergency_stop("Manual User Trigger")
        content = BoxLayout(orientation='vertical', padding=12, spacing=12)
        close_btn = Button(text="Reset E-Stop & Neutralize", size_hint=(1, 0.35), background_color=(0.2, 0.8, 0.3, 1))
        content.add_widget(Label(text="🚨 EMERGENCY STOP ACTIVATED!\nMotors unlocked to neutral position.", font_size='13sp', bold=True, halign='center', color=(1, 0.3, 0.3, 1)))
        content.add_widget(close_btn)
        popup = Popup(title="Safety Lockout", content=content, size_hint=(0.55, 0.40), auto_dismiss=False)
        
        def handle_reset(btn_inst):
            self.worker.reset_emergency_stop()
            popup.dismiss()

        close_btn.bind(on_press=handle_reset)
        popup.open()

    def trigger_test_start(self, instance):
        success, msg = self.worker.start_test_stream()
        if success:
            self.btn_test_start.disabled = True
            self.btn_test_stop.disabled = False
            self.lbl_status_indicator.text = "STATUS: TESTING"

    def trigger_test_stop(self, instance):
        self.worker.stop_test_stream()
        self.btn_test_start.disabled = False
        self.btn_test_stop.disabled = True
        self.lbl_status_indicator.text = "STATUS: IDLE"

    def trigger_set_origin(self, instance):
        success, msg = self.worker.can_mgr.set_origin_all()
        content = BoxLayout(orientation='vertical', padding=12, spacing=12)
        close_btn = Button(text="Dismiss", size_hint=(1, 0.3))
        content.add_widget(Label(text="Zero origin calibrated for all motors!", font_size='13sp', halign='center'))
        content.add_widget(close_btn)
        popup = Popup(title="⚙️ Origin Calibration", content=content, size_hint=(0.55, 0.35))
        close_btn.bind(on_press=popup.dismiss)
        popup.open()

    def trigger_smart_movement(self, movement_type: str):
        self.session_data_log.clear()
        self.recording_start_time = time.time()
        self.lbl_status_indicator.text = "STATUS: SMART AAN"
        speed_mode = self.spn_speed.text
        
        success, msg = self.worker.trigger_smart_aan_protocol(
            movement_type, 
            speed_mode, 
            on_complete_callback=lambda: Clock.schedule_once(lambda dt: self.save_recorded_session_to_csv())
        )
        if not success:
            self.lbl_movement_indicator.text = f"BUSY: {msg}"

    def save_recorded_session_to_csv(self):
        if not self.session_data_log:
            self.lbl_status_indicator.text = "STATUS: IDLE"
            return
        counter = 1
        while True:
            check_path = os.path.join(self.project_directory, f"emgimuflex-data_{counter}.csv")
            if not os.path.exists(check_path): break
            counter += 1
        target_filename = f"emgimuflex-data_{counter}.csv"
        absolute_export_path = os.path.join(self.project_directory, target_filename)
        try:
            with open(absolute_export_path, mode='w', newline='') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow([
                    'Relative_Time_Sec', 'Movement_Label',
                    'Flex_1_Pct', 'Flex_2_Pct', 'Flex_3_Pct', 'Flex_4_Pct', 'Flex_5_Pct',
                    'EMG_Flexor_Raw', 'EMG_Extensor_Raw',
                    'Accel_X', 'Accel_Y', 'Accel_Z',
                    'Gyro_X', 'Gyro_Y', 'Gyro_Z',
                    'M1_Actual_Deg', 'M1_Speed', 'M1_Accel',
                    'M2_Actual_Deg', 'M2_Speed', 'M2_Accel',
                    'M11_Actual_Deg', 'M11_Speed', 'M11_Accel',
                    'M12_Actual_Deg', 'M12_Speed', 'M12_Accel'
                ])
                writer.writerows(self.session_data_log)
            self.lbl_status_indicator.text = f"SAVED: _{counter}.csv"
        except Exception as e:
            self.lbl_status_indicator.text = "STATUS: WRITE ERR"
        self.session_data_log.clear()

    def trigger_reinit_hardware(self, instance):
        _, status_msg = self.worker.reinitialize_hardware()
        content = BoxLayout(orientation='vertical', padding=12, spacing=12)
        close_btn = Button(text="Dismiss", size_hint=(1, 0.3))
        content.add_widget(Label(text=status_msg, font_size='13sp', halign='center'))
        content.add_widget(close_btn)
        popup = Popup(title="🎛️ Hardware Re-initialization", content=content, size_hint=(0.55, 0.42))
        close_btn.bind(on_press=popup.dismiss)
        popup.open()

    def update_telemetry_ui(self, dt):
        has_new_data = False
        latest_f = latest_emg = latest_a = latest_g = None
        latest_m1 = latest_m2 = latest_m11 = latest_m12 = None
        current_label = "IDLE"

        while True:
            try:
                frame = self.telemetry_queue.get_nowait()
                current_label = frame['movement_label']
                latest_f = frame['flex']
                latest_emg = frame['emg']
                latest_a = frame['accel']
                latest_g = frame['gyro']
                latest_m1 = frame['m1']
                latest_m2 = frame['m2']
                latest_m11 = frame['m11']
                latest_m12 = frame['m12']

                for i in range(5):
                    if latest_f[i] > self.flex_peaks[i]: self.flex_peaks[i] = latest_f[i]
                    self.buffers[i].append(latest_f[i])
                    
                for i in range(2):
                    if latest_emg[i] > self.emg_peaks[i]: self.emg_peaks[i] = latest_emg[i]
                    self.buffers[5 + i].append(latest_emg[i])
                    
                for i in range(3): self.buffers[7 + i].append(latest_a[i])
                for i in range(3): self.buffers[10 + i].append(latest_g[i])
                
                if self.worker.is_streaming and current_label != "SENSOR_TEST":
                    relative_timestamp = time.time() - self.recording_start_time
                    row_entry = [
                        f"{relative_timestamp:.4f}", current_label,
                        latest_f[0], latest_f[1], latest_f[2], latest_f[3], latest_f[4],
                        latest_emg[0], latest_emg[1],
                        latest_a[0], latest_a[1], latest_a[2],
                        latest_g[0], latest_g[1], latest_g[2],
                        latest_m1[0], latest_m1[1], latest_m1[2],
                        latest_m2[0], latest_m2[1], latest_m2[2],
                        latest_m11[0], latest_m11[1], latest_m11[2],
                        latest_m12[0], latest_m12[1], latest_m12[2]
                    ]
                    self.session_data_log.append(row_entry)
                
                has_new_data = True
            except queue.Empty:
                break 
            
        if has_new_data:
            self.lbl_movement_indicator.text = f"ACTION: {current_label}"
            self.lbl_hand_data.text = f"FLEX SENSORS:\nF1: {latest_f[0]:.1f}% | F2: {latest_f[1]:.1f}%\nF3: {latest_f[2]:.1f}% | F4: {latest_f[3]:.1f}%\nF5: {latest_f[4]:.1f}%"
            self.lbl_emg_data.text = f"EMG PROFILES:\nFLEXOR: {latest_emg[0]} ({self.emg_peaks[0]})\nEXTENSOR: {latest_emg[1]} ({self.emg_peaks[1]})"
            self.lbl_motion_data.text = f"6-AXIS IMU METRICS:\nACC: X:{latest_a[0]:+.2f}g | Y:{latest_a[1]:+.2f}g | Z:{latest_a[2]:+.2f}g\nGYR: X:{latest_g[0]:+05.0f} | Y:{latest_g[1]:+05.0f} | Z:{latest_g[2]:+05.0f}"
            self.lbl_motor_data.text = f"CUBEMARS POSITIONS:\nM1: {latest_m1[0]:.1f}° | M2: {latest_m2[0]:.1f}°\nM11: {latest_m11[0]:.1f}° | M12: {latest_m12[0]:.1f}°"
            self.redraw_grid_waveforms()

    def redraw_grid_waveforms(self):
        for i in range(13):
            widget = self.graph_areas[i]
            line = self.plot_lines[i]
            buf = self.buffers[i]
            
            self.plot_bgs[i].pos, self.plot_bgs[i].size = widget.pos, widget.size
            w_w, w_h = widget.width, widget.height
            w_x, w_y = widget.x, widget.y
            x_step = w_w / (self.points_count - 1)
            
            pts = []
            for idx, val in enumerate(buf):
                x = w_x + (idx * x_step)
                if i < 5:
                    y = w_y + (max(0.0, min(100.0, val)) / 100.0) * w_h
                elif i < 7:
                    y = w_y + (max(0.0, min(1023.0, val)) / 1023.0) * w_h
                elif i < 10:
                    y = w_y + max(0.0, min(1.0, (val + 2.0) / 4.0)) * w_h
                else:
                    y = w_y + max(0.0, min(1.0, (val + 250.0) / 500.0)) * w_h
                pts.extend([x, y])
            line.points = pts

    def update_rect(self, instance, value): self.bg_rect.pos, self.bg_rect.size = instance.pos, instance.size
    def update_sidebar_bg(self, instance, value): self.sidebar_bg.pos, self.sidebar_bg.size = instance.pos, instance.size
    def update_panel_bg(self, instance, value): self.panel_bg.pos, self.panel_bg.size = instance.pos, instance.size

if __name__ == '__main__':
    TelemetryDashboard().run()