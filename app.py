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
from kivy.uix.togglebutton import ToggleButton
from kivy.clock import Clock
from kivy.graphics import Color, Rectangle, Line
from kivy.uix.popup import Popup
from kivy.uix.scrollview import ScrollView

import threading
import queue
import os    
import time  
from collections import deque

from backmatter import SensorAcquisitionWorker

class TelemetryDashboard(App):
    def build(self):
        self.telemetry_queue = queue.Queue()
        self.points_count = 100 
        
        self.buffers = [deque([0.0] * self.points_count, maxlen=self.points_count) for _ in range(13)]
        
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

        sidebar.add_widget(Label(text="SYSTEM CONTROL", font_size='14sp', bold=True, size_hint_y=None, height=25))

        # SMART LSTM CONTROL TOGGLE BUTTON
        self.btn_auto_lstm = ToggleButton(
            text="ENABLE SMART LSTM CONTROL", 
            font_size='11sp', bold=True, size_hint_y=None, height=45,
            background_color=(0.2, 0.7, 0.4, 1)
        )
        self.btn_auto_lstm.bind(on_press=self.toggle_smart_control)
        sidebar.add_widget(self.btn_auto_lstm)

        # SENSOR TEST STREAM CONTROLS
        self.btn_test_start = Button(text="Start Test Stream", font_size='12sp', size_hint_y=None, height=34, background_color=(0.2, 0.7, 0.9, 1))
        self.btn_test_stop = Button(text="Stop Test Stream", font_size='12sp', size_hint_y=None, height=34, disabled=True, background_color=(0.9, 0.4, 0.3, 1))
        
        self.btn_test_start.bind(on_press=self.trigger_test_start)
        self.btn_test_stop.bind(on_press=self.trigger_test_stop)
        
        sidebar.add_widget(self.btn_test_start)
        sidebar.add_widget(self.btn_test_stop)

        # HARDWARE, ZERO ORIGIN & DASHBOARD RESET CONTROLS
        self.btn_set_origin = Button(text="Set Motor Origin (0°)", font_size='12sp', size_hint_y=None, height=34, background_color=(0.9, 0.7, 0.2, 1))
        self.btn_check = Button(text="Reinit Hardware", font_size='12sp', size_hint_y=None, height=34, background_color=(0.3, 0.5, 0.7, 1))
        self.btn_reset = Button(text="Reset Dashboard", font_size='12sp', size_hint_y=None, height=34, background_color=(0.7, 0.3, 0.3, 1))
        
        self.btn_set_origin.bind(on_press=self.trigger_set_origin)
        self.btn_check.bind(on_press=self.trigger_reinit_hardware)
        self.btn_reset.bind(on_press=self.reset_entire_dashboard)
        
        sidebar.add_widget(self.btn_set_origin)
        sidebar.add_widget(self.btn_check)
        sidebar.add_widget(self.btn_reset)

        self.lbl_status_indicator = Label(text="STATUS: IDLE", font_size='11sp', bold=True, size_hint_y=None, height=25, color=(0.6, 0.6, 0.6, 1))
        sidebar.add_widget(self.lbl_status_indicator)

        self.lbl_movement_indicator = Label(text="ACTION: IDLE", font_size='11sp', bold=True, size_hint_y=None, height=30, color=(0.2, 1.0, 0.4, 1))
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
        
        # LOWER TELEMETRY DISPLAY PANEL
        data_panel = GridLayout(cols=4, size_hint=(1, 0.27), padding=6, spacing=6)
        with data_panel.canvas.before:
            Color(0.12, 0.13, 0.15, 1)
            self.panel_bg = Rectangle()
        data_panel.bind(size=self.update_panel_bg, pos=self.update_panel_bg)
        
        self.lbl_hand_data = Label(text="FLEX TRACKING:\nF1: --% | F2: --%\nF3: --% | F4: --%\nF5: --%", font_size='10sp', bold=True, halign='left', valign='middle')
        self.lbl_emg_data = Label(text="EMG PROFILES:\nFLEXOR: --\nEXTENSOR: --", font_size='10sp', bold=True, halign='left', valign='middle', color=(1.0, 0.4, 1.0, 1))
        self.lbl_motion_data = Label(text="6-AXIS IMU METRICS:\nACC: X:--g | Y:--g | Z:--g\nGYR: X:--dps | Y:--dps | Z:--dps", font_size='10sp', bold=True, halign='left', valign='middle', color=(0.0, 0.8, 1.0, 1))
        
        self.lbl_motor_data = Label(
            text="CUBEMARS MOTOR TELEMETRY:\nM1: --° / Tgt: --° (-- eRPM) | M2: --° / Tgt: --° (-- eRPM)\nM11: --° / Tgt: --° (-- eRPM) | M12: --° / Tgt: --° (-- eRPM)", 
            font_size='10sp', bold=True, halign='left', valign='middle', color=(1.0, 0.8, 0.2, 1)
        )

        data_panel.add_widget(self.lbl_hand_data)
        data_panel.add_widget(self.lbl_emg_data)
        data_panel.add_widget(self.lbl_motion_data)
        data_panel.add_widget(self.lbl_motor_data)
        right_workspace.add_widget(data_panel)
        
        root_layout.add_widget(right_workspace)
        return root_layout

    def on_start(self):
        self.worker = SensorAcquisitionWorker(self.telemetry_queue, model_path="multiLSTM-LOSO.keras")
        self.worker_thread = threading.Thread(target=self.worker.main_loop)
        self.worker_thread.daemon = True
        self.worker_thread.start()
        Clock.schedule_interval(self.update_telemetry_ui, 0.016)

    def toggle_smart_control(self, instance):
        if instance.state == 'down':
            instance.text = "STOP SMART LSTM CONTROL"
            instance.background_color = (0.9, 0.3, 0.3, 1)
            self.lbl_status_indicator.text = "STATUS: LSTM ACTIVE"
            self.btn_test_start.disabled = True
            self.btn_test_stop.disabled = True
            self.worker.toggle_auto_lstm_control(True)
        else:
            instance.text = "ENABLE SMART LSTM CONTROL"
            instance.background_color = (0.2, 0.7, 0.4, 1)
            self.lbl_status_indicator.text = "STATUS: IDLE"
            self.btn_test_start.disabled = False
            self.btn_test_stop.disabled = True
            self.worker.toggle_auto_lstm_control(False)

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
        if self.worker.can_mgr:
            self.worker.can_mgr.set_origin_all()
            content = BoxLayout(orientation='vertical', padding=12, spacing=12)
            close_btn = Button(text="Dismiss", size_hint=(1, 0.3))
            content.add_widget(Label(text="Zero origin calibrated for all motors!", font_size='13sp', halign='center'))
            content.add_widget(close_btn)
            popup = Popup(title="⚙️ Origin Calibration", content=content, size_hint=(0.55, 0.35))
            close_btn.bind(on_press=popup.dismiss)
            popup.open()

    def trigger_reinit_hardware(self, instance):
        _, status_msg = self.worker.reinitialize_hardware()
        content = BoxLayout(orientation='vertical', padding=12, spacing=12)
        close_btn = Button(text="Dismiss", size_hint=(1, 0.3))
        content.add_widget(Label(text=status_msg, font_size='13sp', halign='center'))
        content.add_widget(close_btn)
        popup = Popup(title="🎛️ Hardware Re-initialization", content=content, size_hint=(0.55, 0.42))
        close_btn.bind(on_press=popup.dismiss)
        popup.open()

    def reset_entire_dashboard(self, instance=None):
        self.buffers = [deque([0.0] * self.points_count, maxlen=self.points_count) for _ in range(13)]
        
        self.lbl_hand_data.text = (
            "FLEX TRACKING:\n"
            "F1: 0.0% | F2: 0.0%\n"
            "F3: 0.0% | F4: 0.0%\n"
            "F5: 0.0%"
        )
        self.lbl_emg_data.text = (
            "EMG PROFILES:\n"
            "FLEXOR: 0\n"
            "EXTENSOR: 0"
        )
        self.lbl_motion_data.text = (
            "6-AXIS IMU METRICS:\n"
            "ACC: X:+0.00g | Y:+0.00g | Z:+0.00g\n"
            "GYR: X:+0000 | Y:+0000 | Z:+0000"
        )
        self.lbl_motor_data.text = (
            "CUBEMARS MOTOR TELEMETRY:\n"
            "M1: 0.0° / Tgt: 0.0° (0 eRPM) | M2: 0.0° / Tgt: 0.0° (0 eRPM)\n"
            "M11: 0.0° / Tgt: 0.0° (0 eRPM) | M12: 0.0° / Tgt: 0.0° (0 eRPM)"
        )
        self.redraw_grid_waveforms()

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

                for i in range(5): self.buffers[i].append(latest_f[i])
                for i in range(2): self.buffers[5 + i].append(latest_emg[i])
                for i in range(3): self.buffers[7 + i].append(latest_a[i])
                for i in range(3): self.buffers[10 + i].append(latest_g[i])
                has_new_data = True
            except queue.Empty: break 
            
        if has_new_data:
            if "SAFETY_CUTOFF" in current_label or "OVERSHOOT" in current_label:
                self.lbl_movement_indicator.text = f"🚨 {current_label}"
                self.lbl_movement_indicator.color = (1.0, 0.2, 0.2, 1)
            else:
                self.lbl_movement_indicator.text = f"ACTION: {current_label}"
                self.lbl_movement_indicator.color = (0.2, 1.0, 0.4, 1)

            self.lbl_hand_data.text = f"FLEX SENSORS:\nF1: {latest_f[0]:.1f}% | F2: {latest_f[1]:.1f}%\nF3: {latest_f[2]:.1f}% | F4: {latest_f[3]:.1f}%\nF5: {latest_f[4]:.1f}%"
            self.lbl_emg_data.text = f"EMG PROFILES:\nFLEXOR: {latest_emg[0]}\nEXTENSOR: {latest_emg[1]}"
            self.lbl_motion_data.text = f"6-AXIS IMU METRICS:\nACC: X:{latest_a[0]:+.2f}g | Y:{latest_a[1]:+.2f}g | Z:{latest_a[2]:+.2f}g\nGYR: X:{latest_g[0]:+05.0f} | Y:{latest_g[1]:+05.0f} | Z:{latest_g[2]:+05.0f}"
            
            self.lbl_motor_data.text = (
                f"CUBEMARS MOTOR TELEMETRY:\n"
                f"M1: {latest_m1[0]:.1f}° / Tgt: {latest_m1[2]:.1f}° ({latest_m1[1]:.0f} eRPM) | M2: {latest_m2[0]:.1f}° / Tgt: {latest_m2[2]:.1f}° ({latest_m2[1]:.0f} eRPM)\n"
                f"M11: {latest_m11[0]:.1f}° / Tgt: {latest_m11[2]:.1f}° ({latest_m11[1]:.0f} eRPM) | M12: {latest_m12[0]:.1f}° / Tgt: {latest_m12[2]:.1f}° ({latest_m12[1]:.0f} eRPM)"
            )
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
                if i < 5: y = w_y + (max(0.0, min(100.0, val)) / 100.0) * w_h
                elif i < 7: y = w_y + (max(0.0, min(1023.0, val)) / 1023.0) * w_h
                elif i < 10: y = w_y + max(0.0, min(1.0, (val + 2.0) / 4.0)) * w_h
                else: y = w_y + max(0.0, min(1.0, (val + 250.0) / 500.0)) * w_h
                pts.extend([x, y])
            line.points = pts

    def update_rect(self, instance, value): self.bg_rect.pos, self.bg_rect.size = instance.pos, instance.size
    def update_sidebar_bg(self, instance, value): self.sidebar_bg.pos, self.sidebar_bg.size = instance.pos, instance.size
    def update_panel_bg(self, instance, value): self.panel_bg.pos, self.panel_bg.size = instance.pos, instance.size

if __name__ == '__main__':
    TelemetryDashboard().run()