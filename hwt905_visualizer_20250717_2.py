#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import struct
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtOpenGL import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import math


import os                                       # 운영체제 관련 기능 (파일/디렉토리 조작, 환경변수 등)
import platform                                 # OS 종류 감지용
IS_WINDOWS = platform.system() == 'Windows'     # Windows OS 여부
IS_LINUX = platform.system() == 'Linux'         # Linux OS 여부

# Windows에서 PyQt5 플러그인 경로 수동 설정 (한글 경로 포함)
# - 경로에 사용자가 PC마다 경로가 달라질 것으로 보임
if IS_WINDOWS:
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = r'C:/Users/박두진/AppData/Local/Programs/Python/Python313/Lib/site-packages/PyQt5/Qt5/plugins'


class SerialThread(QThread):
    """시리얼 통신을 처리하는 스레드"""
    data_received = pyqtSignal(float, float, float)
    error_occurred = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.running = False
        self.port_name = None
        self.baud_rate = 9600
        
    def set_port(self, port_name, baud_rate=9600):
        self.port_name = port_name
        self.baud_rate = baud_rate
        
    def run(self):
        if not self.port_name:
            return
            
        try:
            self.serial_port = serial.Serial(self.port_name, self.baud_rate, timeout=1)
            self.running = True
            buffer = bytearray()
            
            while self.running:
                # 데이터가 있으면 읽기
                if self.serial_port.in_waiting > 0:
                    # 버퍼에 새 데이터 추가
                    buffer.extend(self.serial_port.read(self.serial_port.in_waiting))
                    
                    # 버퍼에서 유효한 패킷 찾기
                    while len(buffer) >= 11:
                        # 헤더 찾기 (0x55, 0x53)
                        if buffer[0] == 0x55 and buffer[1] == 0x53:
                            # 11바이트 패킷 추출
                            packet = buffer[:11]
                            
                            # 체크섬 검증 (선택사항)
                            checksum = sum(packet[0:10]) & 0xFF
                            if checksum == packet[10]:
                                # 데이터 파싱
                                roll = struct.unpack('<h', packet[2:4])[0] / 32768.0 * 180.0
                                pitch = struct.unpack('<h', packet[4:6])[0] / 32768.0 * 180.0
                                yaw = struct.unpack('<h', packet[6:8])[0] / 32768.0 * 180.0
                                
                                self.data_received.emit(roll, pitch, yaw)
                            
                            # 처리한 패킷 제거
                            buffer = buffer[11:]
                        else:
                            # 첫 바이트가 0x55가 아니면 제거하고 계속 찾기
                            buffer.pop(0)
                    
                    # 버퍼가 너무 크면 정리 (메모리 보호)
                    if len(buffer) > 1000:
                        buffer = buffer[-100:]
                            
        except serial.SerialException as e:
            self.error_occurred.emit(f"시리얼 포트 오류: {str(e)}")
        except Exception as e:
            self.error_occurred.emit(f"알 수 없는 오류: {str(e)}")
        finally:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                
    def stop(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.wait()

class GLWidget(QWidget):
    """3D 큐브를 표시하는 위젯 (QPainter 사용)"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.angle_x = 0.0
        self.angle_y = 0.0
        self.angle_z = 0.0
        self.setMinimumSize(300, 300)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(50)  # 20 FPS로 제한
        
    def project_3d_to_2d(self, x, y, z):
        """3D 좌표를 2D로 투영"""
        
        # 초기 좌표
        _x, _y, _z = x, y, z

        # X축 회전
        rad_x = math.radians(self.angle_x)
        _y_rotated_x = _y * math.cos(rad_x) - _z * math.sin(rad_x)
        _z_rotated_x = _y * math.sin(rad_x) + _z * math.cos(rad_x)
        _y, _z = _y_rotated_x, _z_rotated_x # X축 회전 결과로 _y, _z 업데이트

        # Y축 회전
        rad_y = math.radians(self.angle_y)
        _x_rotated_y = _x * math.cos(rad_y) + _z * math.sin(rad_y) # X축 회전된 _z 사용
        _z_rotated_y = -_x * math.sin(rad_y) + _z * math.cos(rad_y) # X축 회전된 _z 사용
        _x, _z = _x_rotated_y, _z_rotated_y # Y축 회전 결과로 _x, _z 업데이트

        # Z축 회전
        rad_z = math.radians(self.angle_z)
        x_final = _x * math.cos(rad_z) - _y * math.sin(rad_z) # Y축 회전된 _x, _y 사용
        y_final = _x * math.sin(rad_z) + _y * math.cos(rad_z) # Y축 회전된 _x, _y 사용
        z_final = _z # Z축 회전에서 z는 변하지 않음 (이전 회전 결과)
        
        # 원근 투영
        distance = 5
        scale = 300 
        if z_final + distance != 0:
            px = x_final * scale / (z_final + distance) + self.width() / 2
            py = y_final * scale / (z_final + distance) + self.height() / 2
        else:
            px = x_final * scale + self.width() / 2
            py = y_final * scale + self.height() / 2
                
        return int(px), int(py)
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 배경
        painter.fillRect(self.rect(), QColor(26, 38, 51))
        
        # 큐브 정점
        vertices = [
            [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
            [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]
        ]
        
        # 모서리
        edges = [
            [0, 1], [1, 2], [2, 3], [3, 0],
            [4, 5], [5, 6], [6, 7], [7, 4],
            [0, 4], [1, 5], [2, 6], [3, 7]
        ]
        
        # 2D 투영
        projected = [self.project_3d_to_2d(v[0], v[1], v[2]) for v in vertices]
        
        # 큐브 그리기
        painter.setPen(QPen(QColor(0, 127, 255), 2))
        for edge in edges:
            painter.drawLine(projected[edge[0]][0], projected[edge[0]][1],
                           projected[edge[1]][0], projected[edge[1]][1])
        
        # 축 그리기
        origin = self.project_3d_to_2d(0, 0, 0)
        
        # X축 (빨강)
        x_end = self.project_3d_to_2d(2, 0, 0)
        painter.setPen(QPen(Qt.red, 3))
        painter.drawLine(origin[0], origin[1], x_end[0], x_end[1])
        painter.drawText(x_end[0] + 5, x_end[1], "X")
        
        # Y축 (초록)
        y_end = self.project_3d_to_2d(0, 2, 0)
        painter.setPen(QPen(Qt.green, 3))
        painter.drawLine(origin[0], origin[1], y_end[0], y_end[1])
        painter.drawText(y_end[0] + 5, y_end[1], "Y")
        
        # Z축 (파랑)
        z_end = self.project_3d_to_2d(0, 0, 2)
        painter.setPen(QPen(Qt.blue, 3))
        painter.drawLine(origin[0], origin[1], z_end[0], z_end[1])
        painter.drawText(z_end[0] + 5, z_end[1], "Z")
        
        # 각도 표시
        painter.setFont(QFont("Arial", 12, QFont.Bold))
        painter.setPen(QPen(Qt.red, 2))
        painter.drawText(10, 25, f"X: {self.angle_x:.1f}°")
        painter.setPen(QPen(Qt.green, 2))
        painter.drawText(10, 45, f"Y: {self.angle_y:.1f}°")
        painter.setPen(QPen(Qt.blue, 2))
        painter.drawText(10, 65, f"Z: {self.angle_z:.1f}°")
        
    def update_angles(self, x, y, z):
        """각도 업데이트"""
        self.angle_x = x
        self.angle_y = y
        self.angle_z = z

class CompassWidget(QWidget):
    """나침반 위젯"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.angle = 0.0
        # self.setMinimumSize(200, 300) # 이 줄을 제거하여 동적 사이즈 조절을 허용합니다.
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 배경
        painter.fillRect(self.rect(), QColor(26, 38, 51))
        
        # 나침반 그리기
        center = self.rect().center()
        # 위젯 크기에 따라 나침반의 반지름을 동적으로 조정합니다.
        # min(self.width(), self.height()) * 0.45는 위젯의 짧은 변의 45%를 반지름으로 사용하고,
        # - 20은 여백을 위한 값입니다.
        radius = min(self.width(), self.height()) * 0.45 - 20 
        
        # 외곽 원
        painter.setPen(QPen(QColor(100, 100, 100), 3))
        painter.setBrush(QBrush(QColor(40, 40, 40)))
        painter.drawEllipse(center, int(radius), int(radius)) # radius를 int로 형변환
        
        # 나침반 눈금
        painter.setPen(QPen(Qt.white, 2))
        # 폰트 크기도 위젯 크기에 비례하여 조정할 수 있습니다. (예: radius * 0.1)
        font = QFont("Arial", max(8, int(radius * 0.1)), QFont.Bold)
        painter.setFont(font)
        
        directions = ['N', 'E', 'S', 'W']
        angles = [0, 90, 180, 270]
        
        for i, (dir, ang) in enumerate(zip(directions, angles)):
            angle_rad = math.radians(ang - 90)
            # 텍스트 위치를 나침반 원 바깥쪽으로 충분히 확보 (radius의 1.1배 위치에 배치)
            x = center.x() + radius * 1.1 * math.cos(angle_rad)
            y = center.y() + radius * 1.1 * math.sin(angle_rad)
            
            # 텍스트 사각형 크기도 동적으로 조정
            text_size = max(20, int(radius * 0.15))
            text_rect = QRect(int(x - text_size / 2), int(y - text_size / 2), text_size, text_size)
            painter.drawText(text_rect, Qt.AlignCenter, dir)
        
        # 나침반 바늘
        painter.save()
        painter.translate(center)
        painter.rotate(-self.angle)
        
        # 북쪽 화살표 (빨강)
        painter.setPen(QPen(Qt.red, 3))
        painter.setBrush(QBrush(Qt.red))
        north_arrow = QPolygon([
            QPoint(0, int(-radius * 0.7)),
            QPoint(int(-radius * 0.1), int(-radius * 0.3)),
            QPoint(0, int(-radius * 0.4)),
            QPoint(int(radius * 0.1), int(-radius * 0.3))
        ])
        painter.drawPolygon(north_arrow)
        
        # 남쪽 화살표 (흰색)
        painter.setPen(QPen(Qt.white, 3))
        painter.setBrush(QBrush(Qt.white))
        south_arrow = QPolygon([
            QPoint(0, int(radius * 0.7)),
            QPoint(int(-radius * 0.1), int(radius * 0.3)),
            QPoint(0, int(radius * 0.4)),
            QPoint(int(radius * 0.1), int(radius * 0.3))
        ])
        painter.drawPolygon(south_arrow)
        
        painter.restore()
        
        # 중심점
        painter.setPen(QPen(Qt.white, 3))
        painter.setBrush(QBrush(Qt.white))
        painter.drawEllipse(center, int(radius * 0.05), int(radius * 0.05)) # 중심점 크기도 동적 조정
        
        # Z축 각도 표시 (나침반 아래)
        painter.setPen(QPen(QColor(100, 150, 255), 2))
        # 폰트 크기 동적 조정
        painter.setFont(QFont("Arial", max(10, int(radius * 0.07)), QFont.Bold))
        angle_text = f"Z: {self.angle:.1f}°"
        text_rect = painter.fontMetrics().boundingRect(angle_text)
        # Y 위치를 radius에 비례하여 조정하고 int로 형변환
        painter.drawText(int(center.x() - text_rect.width() / 2), 
                         int(center.y() + radius + max(30, radius * 0.3)), angle_text)
        
        # 현재 방향 표시 (나침반 위)
        painter.setPen(QPen(Qt.white, 1))
        # 폰트 크기 동적 조정
        painter.setFont(QFont("Arial", max(8, int(radius * 0.08))))
        direction_text = self.get_direction_text(self.angle)
        text_rect = painter.fontMetrics().boundingRect(direction_text)
        # Y 위치를 radius에 비례하여 조정하고 int로 형변환
        painter.drawText(int(center.x() - text_rect.width() / 2), 
                         int(center.y() - radius - max(20, radius * 0.2)), direction_text)
        
    def get_direction_text(self, angle):
        """각도에 따른 방향 텍스트 반환"""
        directions = [
            (0, "N"), (45, "NE"), (90, "E"), (135, "SE"),
            (180, "S"), (225, "SW"), (270, "W"), (315, "NW")
        ]
        
        # 각도를 0-360 범위로 정규화
        normalized_angle = angle % 360
        if normalized_angle < 0:
            normalized_angle += 360
            
        # 가장 가까운 방향 찾기
        min_diff = float('inf')
        direction = "N"
        
        for dir_angle, dir_name in directions:
            diff = abs(normalized_angle - dir_angle)
            if diff > 180:
                diff = 360 - diff
            if diff < min_diff:
                min_diff = diff
                direction = dir_name
                
        return f"{direction} ({normalized_angle:.0f}°)"
        
    def update_angle(self, angle):
        """나침반 각도 업데이트"""
        self.angle = angle
        self.update()

class RadarWidget(QWidget):
    """레이더 형태의 시각화 위젯"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.x_angle = 0.0
        self.y_angle = 0.0
        self.setMinimumSize(300, 300)
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 배경
        painter.fillRect(self.rect(), QColor(26, 38, 51))
        
        # 레이더 그리기
        center = self.rect().center()
        radius = min(self.width(), self.height()) // 2 - 40
        
        # 동심원
        painter.setPen(QPen(QColor(80, 80, 80), 1))
        for i in range(1, 4):
            r = radius * i / 3
            painter.drawEllipse(center, int(r), int(r))
            
        # 십자선 (축)
        painter.setPen(QPen(QColor(100, 100, 100), 2))
        painter.drawLine(center.x() - radius, center.y(), 
                        center.x() + radius, center.y())
        painter.drawLine(center.x(), center.y() - radius, 
                        center.x(), center.y() + radius)
        
        # 축 레이블
        painter.setFont(QFont("Arial", 10, QFont.Bold))
        
        # X축 레이블 (빨강)
        painter.setPen(QPen(QColor(255, 100, 100), 2))
        painter.drawText(center.x() + radius + 5, center.y() + 5, "X+")
        painter.drawText(center.x() - radius - 25, center.y() + 5, "X-")
        painter.drawText(center.x() - 50, center.y() + radius + 20, f"X: {self.x_angle:.1f}°")
        
        # Y축 레이블 (초록)
        painter.setPen(QPen(QColor(100, 255, 100), 2))
        painter.drawText(center.x() - 10, center.y() - radius - 5, "Y+")
        painter.drawText(center.x() - 10, center.y() + radius + 15, "Y-")
        painter.drawText(center.x() + 20, center.y() + radius + 20, f"Y: {self.y_angle:.1f}°")
        
        # 각도 범위 표시
        painter.setPen(QPen(QColor(60, 60, 60), 1))
        painter.setFont(QFont("Arial", 8))
        for angle in [-90, -45, 0, 45, 90]:
            # X축 각도
            x_pos = center.x() + (angle / 90.0) * radius
            painter.drawText(int(x_pos - 15), center.y() - 5, f"{angle}°")
            
            # Y축 각도
            y_pos = center.y() - (angle / 90.0) * radius
            painter.drawText(center.x() + 5, int(y_pos + 3), f"{angle}°")
        
        # 위치 점
        x_pos = center.x() + (self.x_angle / 90.0) * radius
        y_pos = center.y() - (self.y_angle / 90.0) * radius
        
        # 범위 제한
        x_pos = max(center.x() - radius, min(center.x() + radius, x_pos))
        y_pos = max(center.y() - radius, min(center.y() + radius, y_pos))
        
        # 궤적 효과
        for i in range(5):
            alpha = 255 - i * 40
            painter.setPen(QPen(QColor(255, 0, 0, alpha), 2))
            painter.setBrush(QBrush(QColor(255, 0, 0, alpha)))
            size = 10 - i * 1.5
            painter.drawEllipse(QPointF(x_pos, y_pos), int(size), int(size))
            
        # 현재 위치 표시
        painter.setPen(QPen(Qt.white, 2))
        painter.setFont(QFont("Arial", 10, QFont.Bold))
        position_text = f"({self.x_angle:.1f}°, {self.y_angle:.1f}°)"
        text_rect = painter.fontMetrics().boundingRect(position_text)
        painter.drawText(center.x() - text_rect.width() // 2, 
                        center.y() - radius - 20, position_text)
            
    def update_position(self, x, y):
        """위치 업데이트"""
        self.x_angle = x
        self.y_angle = y
        self.update()


import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.font_manager as fm

# 한글 폰트 설정
plt.rcParams['font.family'] = 'Malgun Gothic'  # Windows
# plt.rcParams['font.family'] = 'AppleGothic'  # macOS
# plt.rcParams['font.family'] = 'NanumGothic'  # Linux 또는 기타

# 마이너스 기호 깨짐 방지
plt.rcParams['axes.unicode_minus'] = False


class DriftPlot(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.x_data = []
        self.y_data = []
        self.z_data = []
        self.time_data = []

        self.time_counter = 0

        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.fig)
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)

        self.line_x, = self.ax.plot([], [], label='X 기준값')
        self.line_y, = self.ax.plot([], [], label='Y 기준값')
        self.line_z, = self.ax.plot([], [], label='Z 기준값')

        self.ax.legend()
        self.ax.set_title("실시간 기준값 변화")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Offset")

        # self.timer = QTimer()
        # self.timer.timeout.connect(self.update_plot)
        # self.timer.start(1000)  # 1초마다 업데이트

    def update_plot(self, x, y, z):
        # # 예시 데이터 (실제 보정된 기준값으로 교체)
        # x = random.uniform(-0.05, 0.05)
        # y = random.uniform(-0.05, 0.05)
        # z = random.uniform(-0.05, 0.05)

        self.time_counter += 1
        self.time_data.append(self.time_counter)
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)

        max_len = 60  # 최근 60초만 표시
        self.time_data = self.time_data[-max_len:]
        self.x_data = self.x_data[-max_len:]
        self.y_data = self.y_data[-max_len:]
        self.z_data = self.z_data[-max_len:]

        self.line_x.set_data(self.time_data, self.x_data)
        self.line_y.set_data(self.time_data, self.y_data)
        self.line_z.set_data(self.time_data, self.z_data)

        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw()


class InclinoMeter:
    def __init__(self):
        self.calibration_offset = {'roll': 0, 'pitch': 0, 'yaw': 0}
    
    def set_calibration(self, roll_offset=0, pitch_offset=0, yaw_offset=0):
        """센서 캘리브레이션 값 설정"""
        self.calibration_offset = {
            'roll': roll_offset,
            'pitch': pitch_offset, 
            'yaw': yaw_offset
        }
    
    def apply_calibration(self, roll, pitch, yaw):
        """캘리브레이션 적용"""
        calibrated_roll = roll - self.calibration_offset['roll']
        calibrated_pitch = pitch - self.calibration_offset['pitch']
        calibrated_yaw = yaw - self.calibration_offset['yaw']
        
        return calibrated_roll, calibrated_pitch, calibrated_yaw
    
    def calculate_basic_inclination(self, roll, pitch, yaw):
        """기본 기울기 계산"""
        # 캘리브레이션 적용
        roll_cal, pitch_cal, yaw_cal = self.apply_calibration(roll, pitch, yaw)
        
        # 라디안 변환
        roll_rad = math.radians(roll_cal)
        pitch_rad = math.radians(pitch_cal)
        
        # 각 축별 기울기 계산
        roll_inclination = math.tan(roll_rad)
        pitch_inclination = math.tan(pitch_rad)
        
        return {
            'roll_inclination': roll_inclination,
            'pitch_inclination': pitch_inclination,
            'roll_angle': roll_cal,
            'pitch_angle': pitch_cal,
            'roll_radian': roll_rad,
            'pitch_radian': pitch_rad
        }
    
    def calculate_total_inclination(self, roll, pitch, yaw):
        """전체 기울기 계산"""
        # 캘리브레이션 적용
        roll_cal, pitch_cal, yaw_cal = self.apply_calibration(roll, pitch, yaw)
        
        # 라디안 변환
        roll_rad = math.radians(roll_cal)
        pitch_rad = math.radians(pitch_cal)
        
        # 방법 1: 탄젠트 값을 이용한 계산
        roll_tan = math.tan(roll_rad)
        pitch_tan = math.tan(pitch_rad)
        total_inclination_tan = math.sqrt(roll_tan**2 + pitch_tan**2)
        
        # 방법 2: 중력 벡터를 이용한 계산
        gx = math.sin(roll_rad)
        gy = math.sin(pitch_rad)
        gz = math.cos(roll_rad) * math.cos(pitch_rad)
        
        # 수직축과의 각도
        inclination_angle = math.acos(abs(gz))
        inclination_slope = math.tan(inclination_angle)
        
        return {
            'total_inclination_tan': total_inclination_tan,
            'total_inclination_gravity': inclination_slope,
            'inclination_angle_deg': math.degrees(inclination_angle),
            'inclination_angle_rad': inclination_angle
        }
    
    def calculate_inclination_direction(self, roll, pitch, yaw):
        """기울기 방향 계산"""
        # 캘리브레이션 적용
        roll_cal, pitch_cal, yaw_cal = self.apply_calibration(roll, pitch, yaw)
        
        # 라디안 변환
        roll_rad = math.radians(roll_cal)
        pitch_rad = math.radians(pitch_cal)
        
        # 기울기 방향 계산 (북쪽 기준)
        direction_angle = math.atan2(math.sin(roll_rad), math.sin(pitch_rad))
        direction_deg = math.degrees(direction_angle)
        
        # 0~360도 범위로 변환
        if direction_deg < 0:
            direction_deg += 360
        
        return {
            'direction_angle_deg': direction_deg,
            'direction_angle_rad': direction_angle
        }
    
    def convert_to_korean_standard(self, inclination_value):
        """한국 시설안전공단 기준으로 변환"""
        # 기울기 값을 분수 형태로 변환 (1/x 형태)
        if inclination_value == 0:
            return float('inf'), 'a'  # 완전 수평
        
        fraction_denominator = 1 / abs(inclination_value)
        
        # 등급 판정
        if fraction_denominator >= 750:
            grade = 'a'
        elif fraction_denominator >= 500:
            grade = 'b'
        elif fraction_denominator >= 250:
            grade = 'c'
        elif fraction_denominator >= 150:
            grade = 'd'
        else:
            grade = 'e'
        
        return fraction_denominator, grade
    
    def analyze_sensor_data(self, roll, pitch, yaw):
        """센서 데이터 종합 분석"""
        basic = self.calculate_basic_inclination(roll, pitch, yaw)
        total = self.calculate_total_inclination(roll, pitch, yaw)
        direction = self.calculate_inclination_direction(roll, pitch, yaw)
        
        # 한국 기준 변환
        roll_fraction, roll_grade = self.convert_to_korean_standard(basic['roll_inclination'])
        pitch_fraction, pitch_grade = self.convert_to_korean_standard(basic['pitch_inclination'])
        total_fraction, total_grade = self.convert_to_korean_standard(total['total_inclination_gravity'])
        
        return {
            'raw_data': {
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw
            },
            'basic_inclination': basic,
            'total_inclination': total,
            'direction': direction,
            'korean_standard': {
                'roll': {'fraction': roll_fraction, 'grade': roll_grade},
                'pitch': {'fraction': pitch_fraction, 'grade': pitch_grade},
                'total': {'fraction': total_fraction, 'grade': total_grade}
            }
        }
    
    def print_analysis_report(self, analysis_result):
        """분석 결과 리포트 출력"""
        print("=" * 60)
        print("HWT905-TTL 센서 기울기 분석 리포트")
        print("=" * 60)
        
        raw = analysis_result['raw_data']
        basic = analysis_result['basic_inclination']
        total = analysis_result['total_inclination']
        direction = analysis_result['direction']
        korean = analysis_result['korean_standard']
        
        print(f"\n[원본 데이터]")
        print(f"Roll (X): {raw['roll']:.2f}°")
        print(f"Pitch (Y): {raw['pitch']:.2f}°")
        print(f"Yaw (Z): {raw['yaw']:.2f}°")
        
        print(f"\n[기본 기울기]")
        print(f"Roll 기울기: {basic['roll_inclination']:.6f} (각도: {basic['roll_angle']:.2f}°)")
        print(f"Pitch 기울기: {basic['pitch_inclination']:.6f} (각도: {basic['pitch_angle']:.2f}°)")
        
        print(f"\n[전체 기울기]")
        print(f"전체 기울기: {total['total_inclination_gravity']:.6f}")
        print(f"기울기 각도: {total['inclination_angle_deg']:.2f}°")
        print(f"기울기 방향: {direction['direction_angle_deg']:.1f}°")
        
        print(f"\n[한국 시설안전공단 기준]")
        print(f"Roll 기울기: 1/{korean['roll']['fraction']:.0f} (등급: {korean['roll']['grade']})")
        print(f"Pitch 기울기: 1/{korean['pitch']['fraction']:.0f} (등급: {korean['pitch']['grade']})")
        print(f"전체 기울기: 1/{korean['total']['fraction']:.0f} (등급: {korean['total']['grade']})")
        
        print(f"\n[안전성 평가]")
        worst_grade = max(korean['roll']['grade'], korean['pitch']['grade'], korean['total']['grade'])
        grade_descriptions = {
            'a': '매우 안전 (정밀장비 설치 가능)',
            'b': '안전 (구조물 균열 발생 한계)',
            'c': '주의 (육안으로 기울기 감지 가능)',
            'd': '위험 (구조적 손상 예상)',
            'e': '매우 위험 (즉시 조치 필요)'
        }
        print(f"최종 등급: {worst_grade} - {grade_descriptions[worst_grade]}")

import sqlite3
from typing import List, Dict, Any, Optional
from datetime import datetime
import time

class InclinoDBHandler:
    """INCLINO SQLite 데이터베이스 핸들러 클래스"""
    
    def __init__(self, db_name: str = "INCLINO.db"):
        """
        데이터베이스 초기화
        
        Args:
            db_name: 데이터베이스 파일명 (기본값: "INCLINO.db")
        """
        self.db_name = db_name
        self.db_path = os.path.join(os.getcwd(), db_name)
        self._initialize_database()
    
    def _initialize_database(self):
        """데이터베이스 및 테이블 초기화"""
        try:
            # 데이터베이스 연결 (없으면 생성)
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            # tblInitVal 테이블 생성
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS tblInitVal (
                    val_roll TEXT,
                    val_pitch TEXT,
                    check_time TEXT
                )
            """)
            
            # tblSensorData 테이블 생성
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS tblSensorData (
                    Val1 TEXT,
                    Val2 TEXT,
                    Val3 TEXT,
                    Val4 TEXT,
                    Val5 TEXT,
                    Val6 TEXT,
                    Val7 TEXT,
                    Val8 TEXT,
                    Val9 TEXT,
                    Val10 TEXT,
                    Val11 TEXT,
                    Val12 TEXT,
                    Val13 TEXT,
                    Val14 TEXT,
                    Val15 TEXT,
                    check_time TEXT
                )
            """)
            
            conn.commit()
            conn.close()
            print(f"데이터베이스 '{self.db_name}' 초기화 완료")
            
        except sqlite3.Error as e:
            print(f"데이터베이스 초기화 중 오류 발생: {e}")
    
    def _get_connection(self) -> sqlite3.Connection:
        """데이터베이스 연결 반환"""
        return sqlite3.connect(self.db_path)
    
    # ==================== tblInitVal CRUD ====================
    
    def insert_init_val(self, val_roll: str, val_pitch: str, check_time: str = None) -> bool:
        """
        tblInitVal에 데이터 삽입
        
        Args:
            val_roll: 롤 값
            val_pitch: 피치 값
            check_time: 체크 시간 (None이면 현재 시간)
            
        Returns:
            bool: 삽입 성공 여부
        """
        try:
            if check_time is None:
                check_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            conn = self._get_connection()
            cursor = conn.cursor()
            
            cursor.execute("""
                INSERT INTO tblInitVal (val_roll, val_pitch, check_time)
                VALUES (?, ?, ?)
            """, (val_roll, val_pitch, check_time))
            
            conn.commit()
            conn.close()
            return True
            
        except sqlite3.Error as e:
            print(f"tblInitVal 삽입 중 오류 발생: {e}")
            return False
    
    def select_init_val(self, limit: int = None) -> List[Dict[str, Any]]:
        """
        tblInitVal 데이터 조회
        
        Args:
            limit: 조회할 레코드 수 제한
            
        Returns:
            List[Dict]: 조회된 데이터 리스트
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            query = "SELECT val_roll, val_pitch, check_time FROM tblInitVal"
            query += " order by check_time desc"

            if limit:
                query += f" LIMIT {limit}"
                
            cursor.execute(query)
            rows = cursor.fetchall()
            
            result = []
            for row in rows:
                result.append({
                    'val_roll': row[0],
                    'val_pitch': row[1],
                    'check_time': row[2]
                })
            
            conn.close()
            return result
            
        except sqlite3.Error as e:
            print(f"tblInitVal 조회 중 오류 발생: {e}")
            return []
    
    def update_init_val(self, val_roll: str, val_pitch: str, check_time: str) -> bool:
        """
        tblInitVal 데이터 업데이트 (check_time 기준)
        
        Args:
            val_roll: 새로운 롤 값
            val_pitch: 새로운 피치 값
            check_time: 업데이트할 레코드의 check_time
            
        Returns:
            bool: 업데이트 성공 여부
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            cursor.execute("""
                UPDATE tblInitVal 
                SET val_roll = ?, val_pitch = ?
                WHERE check_time = ?
            """, (val_roll, val_pitch, check_time))
            
            conn.commit()
            affected_rows = cursor.rowcount
            conn.close()
            
            return affected_rows > 0
            
        except sqlite3.Error as e:
            print(f"tblInitVal 업데이트 중 오류 발생: {e}")
            return False
    
    def delete_init_val(self, check_time: str = None) -> bool:
        """
        tblInitVal 데이터 삭제
        
        Args:
            check_time: 삭제할 레코드의 check_time (None이면 모든 레코드 삭제)
            
        Returns:
            bool: 삭제 성공 여부
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            if check_time:
                cursor.execute("DELETE FROM tblInitVal WHERE check_time = ?", (check_time,))
            else:
                cursor.execute("DELETE FROM tblInitVal")
            
            conn.commit()
            affected_rows = cursor.rowcount
            conn.close()
            
            return affected_rows > 0
            
        except sqlite3.Error as e:
            print(f"tblInitVal 삭제 중 오류 발생: {e}")
            return False
    
    # ==================== tblSensorData CRUD ====================
    
    def insert_sensor_data(self, check_time: str = None, **kwargs) -> bool:
        """
        tblSensorData에 데이터 삽입
        
        Args:
            check_time: 체크 시간 (None이면 현재 시간)
            **kwargs: Val1~Val15 컬럼 값들
            
        Returns:
            bool: 삽입 성공 여부
        """
        try:
            if check_time is None:
                check_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            conn = self._get_connection()
            cursor = conn.cursor()
            
            # 컬럼명과 값 준비
            columns = [f"Val{i}" for i in range(1, 16)]
            columns.append("check_time")
            
            values = [kwargs.get(f"Val{i}", '') for i in range(1, 16)]
            values.append(check_time)
            
            placeholders = ', '.join(['?' for _ in columns])
            column_names = ', '.join(columns)
            
            cursor.execute(f"""
                INSERT INTO tblSensorData ({column_names})
                VALUES ({placeholders})
            """, values)
            
            conn.commit()
            conn.close()
            return True
            
        except sqlite3.Error as e:
            print(f"tblSensorData 삽입 중 오류 발생: {e}")
            return False
    
    def select_sensor_data(self, limit: int = None) -> List[Dict[str, Any]]:
        """
        tblSensorData 데이터 조회
        
        Args:
            limit: 조회할 레코드 수 제한
            
        Returns:
            List[Dict]: 조회된 데이터 리스트
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            query = "SELECT * FROM tblSensorData"
            if limit:
                query += f" LIMIT {limit}"
            
            cursor.execute(query)
            rows = cursor.fetchall()
            
            result = []
            for row in rows:
                data = {}
                for i in range(15):
                    data[f'Val{i+1}'] = row[i]
                data['check_time'] = row[15]  # check_time 컬럼 추가
                result.append(data)
            
            conn.close()
            return result
            
        except sqlite3.Error as e:
            print(f"tblSensorData 조회 중 오류 발생: {e}")
            return []
    
    def update_sensor_data(self, check_time: str = None, row_id: int = None, **kwargs) -> bool:
        """
        tblSensorData 데이터 업데이트 (check_time 또는 ROWID 기준)
        
        Args:
            check_time: 업데이트할 레코드의 check_time (우선순위)
            row_id: 업데이트할 레코드의 ROWID (check_time이 None일 때 사용)
            **kwargs: 업데이트할 컬럼 값들
            
        Returns:
            bool: 업데이트 성공 여부
        """
        try:
            if not check_time and not row_id:
                print("check_time 또는 row_id 중 하나는 필수입니다.")
                return False
            
            conn = self._get_connection()
            cursor = conn.cursor()
            
            # 업데이트할 컬럼들 준비
            set_clauses = []
            values = []
            
            for key, value in kwargs.items():
                if key.startswith('Val') and key[3:].isdigit():
                    set_clauses.append(f"{key} = ?")
                    values.append(value)
                elif key == 'check_time':
                    set_clauses.append(f"{key} = ?")
                    values.append(value)
            
            if not set_clauses:
                return False
            
            # WHERE 조건 설정
            if check_time:
                where_clause = "check_time = ?"
                values.append(check_time)
            else:
                where_clause = "ROWID = ?"
                values.append(row_id)
            
            cursor.execute(f"""
                UPDATE tblSensorData 
                SET {', '.join(set_clauses)}
                WHERE {where_clause}
            """, values)
            
            conn.commit()
            affected_rows = cursor.rowcount
            conn.close()
            
            return affected_rows > 0
            
        except sqlite3.Error as e:
            print(f"tblSensorData 업데이트 중 오류 발생: {e}")
            return False
    
    def delete_sensor_data(self, check_time: str = None, row_id: int = None) -> bool:
        """
        tblSensorData 데이터 삭제
        
        Args:
            check_time: 삭제할 레코드의 check_time (우선순위)
            row_id: 삭제할 레코드의 ROWID (check_time이 None일 때 사용)
            둘 다 None이면 모든 레코드 삭제
            
        Returns:
            bool: 삭제 성공 여부
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            if check_time:
                cursor.execute("DELETE FROM tblSensorData WHERE check_time = ?", (check_time,))
            elif row_id:
                cursor.execute("DELETE FROM tblSensorData WHERE ROWID = ?", (row_id,))
            else:
                cursor.execute("DELETE FROM tblSensorData")
            
            conn.commit()
            affected_rows = cursor.rowcount
            conn.close()
            
            return affected_rows > 0
            
        except sqlite3.Error as e:
            print(f"tblSensorData 삭제 중 오류 발생: {e}")
            return False
    
    # ==================== 유틸리티 메서드 ====================
    
    def get_table_info(self, table_name: str) -> List[Dict[str, Any]]:
        """
        테이블 정보 조회
        
        Args:
            table_name: 테이블명
            
        Returns:
            List[Dict]: 테이블 구조 정보
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            cursor.execute(f"PRAGMA table_info({table_name})")
            rows = cursor.fetchall()
            
            result = []
            for row in rows:
                result.append({
                    'cid': row[0],
                    'name': row[1],
                    'type': row[2],
                    'notnull': row[3],
                    'dflt_value': row[4],
                    'pk': row[5]
                })
            
            conn.close()
            return result
            
        except sqlite3.Error as e:
            print(f"테이블 정보 조회 중 오류 발생: {e}")
            return []
    
    def get_row_count(self, table_name: str) -> int:
        """
        테이블의 레코드 수 조회
        
        Args:
            table_name: 테이블명
            
        Returns:
            int: 레코드 수
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            cursor.execute(f"SELECT COUNT(*) FROM {table_name}")
            count = cursor.fetchone()[0]
            
            conn.close()
            return count
            
        except sqlite3.Error as e:
            print(f"레코드 수 조회 중 오류 발생: {e}")
            return 0
    
    def close(self):
        """데이터베이스 연결 종료 (현재 구현에서는 각 메서드마다 연결을 닫으므로 placeholder)"""
        pass


# 데이터베이스 핸들러 생성
db = InclinoDBHandler()

class MainWindow(QMainWindow):
    """HWT905 센서 데이터를 시각화하기 위한 메인 윈도우 클래스"""

    def __init__(self):
        super().__init__()
        self.initUI()  # UI 초기화
        
        
        # hwt905_visualizer_20250717_2.py
        # 데이터 조회
        init_data = db.select_init_val(1)
        print(f"tblInitVal 레코드 수: {len(init_data)}")
        if len(init_data) > 0:
            print(f"Roll: {init_data[0]['val_roll']}, Pitch: {init_data[0]['val_pitch']}, Time: {init_data[0]['check_time']}")
            self.cal_roll_editor.setText(f"{float(init_data[0]['val_roll']):.4f}")
            self.cal_pitch_editor.setText(f"{float(init_data[0]['val_pitch']):.4f}")
            # elf.cal_yaw_editor.setText(f"{self.current_raw_yaw:.4f}")

        # 시리얼 통신을 위한 스레드 생성
        self.serial_thread = SerialThread()
        # 데이터 수신 시그널 연결
        self.serial_thread.data_received.connect(self.update_display)
        # 에러 발생 시그널 연결
        self.serial_thread.error_occurred.connect(self.handle_serial_error)

        # 기울기 계산기 초기화
        self.inclinoMeter = InclinoMeter()
        
        # 현재 raw 센서 데이터를 저장할 변수 초기화
        self.current_raw_roll = 0.0
        self.current_raw_pitch = 0.0
        self.current_raw_yaw = 0.0

        # 1. 마지막으로 데이터를 저장한 시간을 저장할 변수 초기화
        self.last_save_time = time.time() # 프로그램 시작 시 현재 시간을 기록
        
    def initUI(self):
        """UI 초기화 메서드"""
        # 윈도우 제목과 크기 설정
        self.setWindowTitle('HWT905 센서 데이터 시각화')

        # hwt905_visualizer_20250717_2.py - 주석처리
        # self.setGeometry(100, 100, 1200, 1000)

        # 다크 테마 스타일 시트 적용
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1a2633;  /* 어두운 배경색 */
            }
            QLabel {
                color: #ffffff; /* 기본 텍스트 색상 흰색 */
                font-size: 10pt;
            }
            QLineEdit {
                background-color: #333333;
                color: #ffffff;
                border: 1px solid #555555;
                padding: 2px;
            }
            QPushButton {
                background-color: #5a5a5a;
                color: #ffffff;
                border: 1px solid #777777;
                padding: 5px 10px;
            }
            QPushButton:hover {
                background-color: #6a6a6a;
            }
            .data_label {
                color: #4a9eff; /* 데이터 값 색상 */
                font-family: monospace;
            }
            .inclination_label {
                color: #ff9e4a; /* 기울기 값 색상 */
                font-family: monospace;
            }
            .report_text_edit {
                background-color: #2b2b2b; /* 보고서 배경색 */
                color: #cccccc; /* 보고서 텍스트 색상 */
                font-family: Consolas, monospace;
                font-size: 9pt;
                border: 1px solid #555555;
            }
            .analysis_frame {
                border: 1px solid #444444; /* 프레임 테두리 색상 */
                border-radius: 5px; /* 모서리 둥글게 */
                background-color: #2b2b2b; /* 프레임 배경색 */
            }
        """)

        # 중앙 위젯 설정
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # ===================================================
        # 상단 컨트롤 패널 생성
        control_panel = QWidget()
        control_panel.setStyleSheet("background-color: #81ca8a;")
        control_layout = QHBoxLayout(control_panel)

        # COM 포트 선택을 위한 콤보박스
        control_layout.addWidget(QLabel('COM 포트:'))
        self.port_combo = QComboBox()
        self.refresh_ports()  # 사용 가능한 포트 목록 새로고침
        control_layout.addWidget(self.port_combo)

        # 새로고침 버튼
        refresh_btn = QPushButton('새로고침')
        refresh_btn.clicked.connect(self.refresh_ports)
        control_layout.addWidget(refresh_btn)

        # 연결/해제 버튼
        self.connect_btn = QPushButton('연결')
        self.connect_btn.clicked.connect(self.toggle_connection)
        control_layout.addWidget(self.connect_btn)

        control_layout.addStretch()
        main_layout.addWidget(control_panel)
        # ===================================================

        # ===================================================
        # 캘리브레이션 설정 패널 (control_layout 아래에 추가)
        set_panel = QWidget()
        set_layout = QHBoxLayout(set_panel)
        # set_panel.setStyleSheet("background-color: #2c3e50; padding: 5px; border-radius: 5px;")

        # Cal_Roll(X)
        set_layout.addWidget(QLabel('Cal_Roll(X):'))
        self.cal_roll_editor = QLineEdit('0.00')
        self.cal_roll_editor.setFixedWidth(80)
        set_layout.addWidget(self.cal_roll_editor)

        # Cal_Pitch(Y)
        set_layout.addWidget(QLabel('Cal_Pitch(Y):'))
        self.cal_pitch_editor = QLineEdit('0.00')
        self.cal_pitch_editor.setFixedWidth(80)
        set_layout.addWidget(self.cal_pitch_editor)

        # Cal_Yaw(Z)
        set_layout.addWidget(QLabel('Cal_Yaw(Z):'))
        self.cal_yaw_editor = QLineEdit('0.00')
        self.cal_yaw_editor.setFixedWidth(80)
        set_layout.addWidget(self.cal_yaw_editor)

        # Get_Cal 버튼
        self.get_cal_btn = QPushButton('Get_Cal')
        self.get_cal_btn.clicked.connect(self.get_calibration_values)
        set_layout.addWidget(self.get_cal_btn)

        # Set_Cal 버튼
        self.set_cal_btn = QPushButton('Set_Cal')
        self.set_cal_btn.clicked.connect(self.set_calibration_values)
        set_layout.addWidget(self.set_cal_btn)

        set_layout.addStretch()
        main_layout.addWidget(set_panel)
        # ===================================================

        data_txt_panel = QWidget()
        data_txt_panel.setStyleSheet("background-color: #1a2633;")
        data_txt_layout = QHBoxLayout(data_txt_panel)

        # InclinoMeter 분석 결과 표시를 위한 레이아웃
        self.analysis_labels = {}
        # ==========================================================
        # analysis_data1_container
        analysis_data1_frame = QFrame()  # QFrame으로 감싸서 구분
        analysis_data1_frame.setFrameShape(QFrame.Box)
        analysis_data1_frame.setFrameShadow(QFrame.Plain)
        analysis_data1_frame.setObjectName("analysis_frame") # 스타일시트 적용
        analysis_data1_frame.setFixedSize(300, 300) # 여기에 고정할 너비와 높이를 설정합니다. (예: 350, 400)
        analysis_data1_layout = QVBoxLayout(analysis_data1_frame)
        analysis_data1_layout.setContentsMargins(10, 10, 10, 10) # 적절한 마진 설정
        analysis_data1_layout.setAlignment(Qt.AlignTop) # 라벨들을 위쪽 정렬

        self.add_analysis_label(analysis_data1_layout, "원본 데이터:", "raw_data_header", "#ffffff", "#4a9eff", is_header=True)
        self.add_analysis_label(analysis_data1_layout, "Roll (X):", "raw_roll_val", "#bbbbbb", "#ab53b5")
        self.add_analysis_label(analysis_data1_layout, "Pitch (Y):", "raw_pitch_val", "#bbbbbb", "#ab53b5")

        self.add_analysis_div(analysis_data1_layout)

        self.add_analysis_label(analysis_data1_layout, "Yaw (Z):", "raw_yaw_val", "#bbbbbb", "#bbbbbb")
        analysis_data1_layout.addStretch() # 라벨들을 위로 정렬

        data_txt_layout.addWidget(analysis_data1_frame) # 프레임을 레이아웃에 추가

        # ==========================================================
        # analysis_data2_container
        analysis_data2_frame = QFrame() # QFrame으로 감싸서 구분
        analysis_data2_frame.setFrameShape(QFrame.Box)
        analysis_data2_frame.setFrameShadow(QFrame.Plain)
        analysis_data2_frame.setObjectName("analysis_frame") # 스타일시트 적용
        analysis_data2_frame.setFixedSize(300, 300) # 여기에 고정할 너비와 높이를 설정합니다. (예: 350, 400)
        analysis_data2_layout = QVBoxLayout(analysis_data2_frame)
        analysis_data2_layout.setContentsMargins(10, 10, 10, 10) # 적절한 마진 설정
        analysis_data2_layout.setAlignment(Qt.AlignTop) # 라벨들을 위쪽 정렬

        self.add_analysis_label(analysis_data2_layout, "기본 기울기:", "basic_inclination_header", "#ffffff", "#4a9eff", is_header=True)
        self.add_analysis_label(analysis_data2_layout, "Roll 각도(degree):", "basic_roll_angle", "#bbbbbb", "#4a9eff")
        self.add_analysis_label(analysis_data2_layout, "Roll 라디안(radian):", "basic_roll_radian", "#bbbbbb", "#4a9eff")
        self.add_analysis_label(analysis_data2_layout, "Roll 기울기 (tan):", "basic_roll_inclination", "#bbbbbb", "#dd3333")

        self.add_analysis_div(analysis_data2_layout)

        self.add_analysis_label(analysis_data2_layout, "Pitch 각도(degree):", "basic_pitch_angle", "#bbbbbb", "#4a9eff")
        self.add_analysis_label(analysis_data2_layout, "Pitch 라디안(radian):", "basic_pitch_radian", "#bbbbbb", "#4a9eff")
        self.add_analysis_label(analysis_data2_layout, "Pitch 기울기 (tan):", "basic_pitch_inclination", "#bbbbbb", "#dd3333")

        self.add_analysis_div(analysis_data2_layout)        

        self.add_analysis_label(analysis_data2_layout, "전체 기울기:", "total_inclination_header", "#ffffff", "#4a9eff", is_header=True)
        self.add_analysis_label(analysis_data2_layout, "전체 기울기 (tan):", "total_inclination_tan", "#bbbbbb", "#dd3333")
        self.add_analysis_label(analysis_data2_layout, "전체 기울기 (중력):", "total_inclination_gravity", "#bbbbbb", "#ec991c")
        self.add_analysis_label(analysis_data2_layout, "기울기 각도(degree):", "total_inclination_angle_deg", "#bbbbbb", "#4a9eff")
        self.add_analysis_label(analysis_data2_layout, "기울기 방향:", "direction_angle_deg", "#bbbbbb", "#4a9eff")
        analysis_data2_layout.addStretch() # 라벨들을 위로 정렬

        data_txt_layout.addWidget(analysis_data2_frame) # 프레임을 레이아웃에 추가

        # ==========================================================
        # analysis_data3_container
        analysis_data3_frame = QFrame() # QFrame으로 감싸서 구분
        analysis_data3_frame.setFrameShape(QFrame.Box)
        analysis_data3_frame.setFrameShadow(QFrame.Plain)
        analysis_data3_frame.setObjectName("analysis_frame") # 스타일시트 적용
        analysis_data3_frame.setFixedSize(400, 300) # 여기에 고정할 너비와 높이를 설정합니다. (예: 350, 400)
        analysis_data3_layout = QVBoxLayout(analysis_data3_frame)
        analysis_data3_layout.setContentsMargins(10, 10, 10, 10) # 적절한 마진 설정
        analysis_data3_layout.setAlignment(Qt.AlignTop) # 라벨들을 위쪽 정렬

        self.add_analysis_label(analysis_data3_layout, "한국 시설안전공단 기준:", "korean_standard_header", "#ffffff", "#4a9eff", is_header=True)
        self.add_analysis_label(analysis_data3_layout, "Roll (1/):", "korean_roll", "#bbbbbb", "#4a9eff")
        self.add_analysis_label(analysis_data3_layout, "Pitch (1/):", "korean_pitch", "#bbbbbb", "#4a9eff")
        self.add_analysis_label(analysis_data3_layout, "Total (1/):", "korean_total", "#bbbbbb", "#4a9eff")

        self.add_analysis_div(analysis_data3_layout)

        self.add_analysis_label(analysis_data3_layout, "최종 등급:", "final_grade", "#bbbbbb", "#4a9eff")
        analysis_data3_layout.addStretch() # 라벨들을 위로 정렬

        data_txt_layout.addWidget(analysis_data3_frame) # 프레임을 레이아웃에 추가

        main_layout.addWidget(data_txt_panel)

        # ===================================================
        # hwt905_visualizer_20250717_2.py - 주석 처리
        # # 2025.07.17 duzin
        # self.plot_widget = DriftPlot(self)
        # main_layout.addWidget(self.plot_widget)

        # ===================================================

        # 이미지 데이터 표시 영역
        data_widget = QWidget()
        data_widget.setStyleSheet("background-color: #1a2633;")
        data_layout = QHBoxLayout(data_widget)

        # 3D 큐브 위젯
        self.gl_widget = GLWidget()
        self.gl_widget.setMinimumSize(300, 300)

        data_layout.addWidget(self.gl_widget)

        # 2. 중앙 패널: 나침반 위젯
        self.compass_widget = CompassWidget()
        data_layout.addWidget(self.compass_widget)

        # 3. 오른쪽 패널: 레이더 뷰
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        self.radar_widget = RadarWidget()
        right_layout.addWidget(self.radar_widget)
        data_layout.addWidget(right_panel)

        main_layout.addWidget(data_widget)

        
    def add_analysis_label(self, layout, text, key, color_lbl1, color_lbl2, is_header=False):
        """분석 결과 레이블을 추가하는 헬퍼 함수"""
        row_widget = QWidget()
        row_layout = QHBoxLayout(row_widget)
        row_layout.setContentsMargins(0, 0, 0, 0) # 마진 제거

        # label = QLabel(text)
        if is_header:
            label = QLabel(text)
        else:
            label = QLabel(' - ' + text)
        
        label.setFixedWidth(150) # 레이블 너비 고정
        if is_header:
            # label.setStyleSheet("color: #ffffff; font-weight: bold;")
            label.setStyleSheet(f"color: {color_lbl1}; font-weight: bold;")
        else:
            # label.setStyleSheet("color: #bbbbbb;") # 일반 항목 색상
            label.setStyleSheet(f"color: {color_lbl1};") # 일반 항목 색상
        row_layout.addWidget(label)
        
        value_label = QLabel('')
        value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        # value_label.setStyleSheet("color: #4a9eff; font-family: monospace;")
        value_label.setStyleSheet(f"color: {color_lbl2}; font-family: monospace;")
        self.analysis_labels[key] = value_label
        row_layout.addWidget(value_label)
        
        layout.addWidget(row_widget)

    def add_analysis_div(self, layout):
        """분석 결과 레이블을 추가하는 헬퍼 함수"""
        row_widget = QWidget()
        row_layout = QHBoxLayout(row_widget)
        row_layout.setContentsMargins(0, 0, 0, 0) # 마진 제거
        row_layout.addWidget(QLabel(''))       
        
        layout.addWidget(row_widget)

    def refresh_ports(self):
        """사용 가능한 시리얼 포트 목록을 새로고침하는 메서드"""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)
            
    def handle_serial_error(self, error_msg):
        """시리얼 통신 오류를 처리하는 메서드"""
        QMessageBox.critical(self, "연결 오류", error_msg)
        self.connect_btn.setText('연결')
        self.port_combo.setEnabled(True)
        
    def toggle_connection(self):
        """시리얼 연결을 토글하는 메서드"""
        if self.connect_btn.text() == '연결':
            port = self.port_combo.currentText()
            if port:
                try:
                    # 포트 사용 가능 여부 확인
                    test_port = serial.Serial()
                    test_port.port = port
                    test_port.baudrate = 19200
                    test_port.timeout = 0.1
                    test_port.open()
                    test_port.close()
                    
                    # 연결 시작
                    self.serial_thread.set_port(port, 19200)
                    self.serial_thread.start()
                    self.connect_btn.setText('연결 해제')
                    self.port_combo.setEnabled(False)
                except serial.SerialException as e:
                    QMessageBox.critical(self, "연결 오류", 
                        f"포트 {port}에 연결할 수 없습니다.\n\n"
                        f"오류: {str(e)}\n\n"
                        f"가능한 원인:\n"
                        f"- 다른 프로그램에서 포트 사용 중\n"
                        f"- 장치가 연결되지 않음\n"
                        f"- 권한 부족 (관리자 권한 필요)")
        else:
            # 연결 해제
            self.serial_thread.stop()
            self.connect_btn.setText('연결')
            self.port_combo.setEnabled(True)
            
    def update_display(self, x, y, z):
        """센서 데이터를 화면에 업데이트하는 메서드"""
        # 현재 raw 데이터를 저장 (캘리브레이션 버튼에서 사용)
        self.current_raw_roll = x
        self.current_raw_pitch = y
        self.current_raw_yaw = z

        # 3D 큐브 업데이트
        self.gl_widget.update_angles(x, y, z)
        
        # 나침반 업데이트 (Z축 사용)
        self.compass_widget.update_angle(z)
        
        # 레이더 뷰 업데이트 (X, Y 좌표 사용)
        self.radar_widget.update_position(x, y)
        
        # InclinoMeter를 사용하여 센서 데이터 분석
        analysis = self.inclinoMeter.analyze_sensor_data(x, y, z)
        
        # data_txt_layout에 analyze_sensor_data의 리턴값 모두 표시
        self.analysis_labels['raw_roll_val'].setText(f"{analysis['raw_data']['roll']:.4f}°")
        self.analysis_labels['raw_pitch_val'].setText(f"{analysis['raw_data']['pitch']:.4f}°")
        self.analysis_labels['raw_yaw_val'].setText(f"{analysis['raw_data']['yaw']:.4f}°")

        self.analysis_labels['basic_roll_angle'].setText(f"{analysis['basic_inclination']['roll_angle']:.4f}°")
        self.analysis_labels['basic_roll_radian'].setText(f"{analysis['basic_inclination']['roll_radian']:.4f}")
        self.analysis_labels['basic_roll_inclination'].setText(f"{analysis['basic_inclination']['roll_inclination']:.6f}")
        self.analysis_labels['basic_pitch_angle'].setText(f"{analysis['basic_inclination']['pitch_angle']:.4f}°")
        self.analysis_labels['basic_pitch_radian'].setText(f"{analysis['basic_inclination']['pitch_radian']:.4f}")
        self.analysis_labels['basic_pitch_inclination'].setText(f"{analysis['basic_inclination']['pitch_inclination']:.6f}")
        
        self.analysis_labels['total_inclination_tan'].setText(f"{analysis['total_inclination']['total_inclination_tan']:.6f}")
        self.analysis_labels['total_inclination_gravity'].setText(f"{analysis['total_inclination']['total_inclination_gravity']:.6f}")
        self.analysis_labels['total_inclination_angle_deg'].setText(f"{analysis['total_inclination']['inclination_angle_deg']:.4f}°")
        self.analysis_labels['direction_angle_deg'].setText(f"{analysis['direction']['direction_angle_deg']:.4f}°")

        korean_std = analysis['korean_standard']
        self.analysis_labels['korean_roll'].setText(f"1/{korean_std['roll']['fraction']:.0f} (등급: {korean_std['roll']['grade'].upper()})")
        self.analysis_labels['korean_pitch'].setText(f"1/{korean_std['pitch']['fraction']:.0f} (등급: {korean_std['pitch']['grade'].upper()})")
        self.analysis_labels['korean_total'].setText(f"1/{korean_std['total']['fraction']:.0f} (등급: {korean_std['total']['grade'].upper()})")

        worst_grade = max(korean_std['roll']['grade'], korean_std['pitch']['grade'], korean_std['total']['grade'])
        grade_descriptions = {
            'a': '매우 안전 (정밀장비 설치 가능)',
            'b': '안전 (구조물 균열 발생 한계)',
            'c': '주의 (육안으로 기울기 감지 가능)',
            'd': '위험 (구조적 손상 예상)',
            'e': '매우 위험 (즉시 조치 필요)'
        }
        self.analysis_labels['final_grade'].setText(f"{worst_grade.upper()} - {grade_descriptions[worst_grade]}")


        # hwt905_visualizer_20250717_2.py
        current_time = time.time()
        # 2. 현재 시간과 마지막 저장 시간 비교
        if current_time - self.last_save_time >= 60: # 60초 (1분) 이상 지났으면            
            _val1 = f"{analysis['raw_data']['roll']:.4f}"
            _val2 = f"{analysis['raw_data']['pitch']:.4f}"
            _val3 = f"{analysis['raw_data']['yaw']:.4f}"
            _val4 = f"{analysis['basic_inclination']['roll_angle']:.4f}"
            _val5 = f"{analysis['basic_inclination']['roll_radian']:.4f}"
            _val6 = f"{analysis['basic_inclination']['roll_inclination']:.4f}"
            _val7 = f"{analysis['basic_inclination']['pitch_angle']:.4f}"
            _val8 = f"{analysis['basic_inclination']['pitch_radian']:.4f}"
            _val9 = f"{analysis['basic_inclination']['pitch_inclination']:.4f}"
            _val10 = f"{analysis['total_inclination']['total_inclination_tan']:.4f}"
            _val11 = f"{analysis['total_inclination']['total_inclination_gravity']:.4f}"
            _val12 = f"{analysis['total_inclination']['inclination_angle_deg']:.4f}"
            _val13 = f"{analysis['direction']['direction_angle_deg']:.4f}"
            _val14 = ""
            _val15 = ""
            db.insert_sensor_data(Val1=_val1, Val2=_val2, Val3=_val3, Val4=_val4, Val5=_val5, 
                                Val6=_val6, Val7=_val7, Val8=_val8, Val9=_val9, Val10=_val10,
                                Val11=_val11, Val12=_val12, Val13=_val13, Val14=_val14, Val15=_val15)

            self.last_save_time = current_time # 마지막 저장 시간 업데이트




        # hwt905_visualizer_20250717_2.py - 주석처리
        # # 2025.07.17 duzin
        # self.plot_widget.update_plot(x, y, z)

        # # print_analysis_report의 데이터도 QTextEdit에 표시
        # report_output = self.get_analysis_report_string(analysis)
        # self.report_text_edit.setText(report_output)

    def get_calibration_values(self):
        """현재 raw 센서 값을 캘리브레이션 에디터에 설정합니다."""
        self.cal_roll_editor.setText(f"{self.current_raw_roll:.4f}")
        self.cal_pitch_editor.setText(f"{self.current_raw_pitch:.4f}")
        self.cal_yaw_editor.setText(f"{self.current_raw_yaw:.4f}")
        QMessageBox.information(self, "캘리브레이션", "현재 센서 값이 캘리브레이션 오프셋으로 설정되었습니다.")

    def set_calibration_values(self):
        """캘리브레이션 에디터의 값을 InclinoMeter에 설정합니다."""
        try:
            roll_offset = float(self.cal_roll_editor.text())
            pitch_offset = float(self.cal_pitch_editor.text())
            yaw_offset = float(self.cal_yaw_editor.text())
            self.inclinoMeter.set_calibration(roll_offset, pitch_offset, yaw_offset)

            # hwt905_visualizer_20250717_2.py
            # 데이터 저장 - 초기 값
            db.insert_init_val(self.cal_roll_editor.text(), self.cal_pitch_editor.text())
            print(f"데이터 저장 : Roll: {self.cal_roll_editor.text()}, Pitch: {self.cal_pitch_editor.text()}")


            QMessageBox.information(self, "캘리브레이션", 
                                    f"캘리브레이션 오프셋 설정 완료:\n"
                                    f"Roll: {roll_offset:.2f}, Pitch: {pitch_offset:.2f}, Yaw: {yaw_offset:.2f}")
        except ValueError:
            QMessageBox.warning(self, "입력 오류", "유효한 숫자 값을 입력해주세요.")
        
    def get_analysis_report_string(self, analysis_result):
        """분석 결과를 문자열로 반환 (print_analysis_report와 유사)"""
        report_str = "=" * 60 + "\n"
        report_str += "HWT905-TTL 센서 기울기 분석 리포트\n"
        report_str += "=" * 60 + "\n"
        
        raw = analysis_result['raw_data']
        basic = analysis_result['basic_inclination']
        total = analysis_result['total_inclination']
        direction = analysis_result['direction']
        korean = analysis_result['korean_standard']
        
        report_str += f"\n[원본 데이터]\n"
        report_str += f"Roll (X): {raw['roll']:.2f}°\n"
        report_str += f"Pitch (Y): {raw['pitch']:.2f}°\n"
        report_str += f"Yaw (Z): {raw['yaw']:.2f}°\n"
        
        report_str += f"\n[기본 기울기]\n"
        report_str += f"Roll 기울기: {basic['roll_inclination']:.6f} (각도: {basic['roll_angle']:.2f}°)\n"
        report_str += f"Pitch 기울기: {basic['pitch_inclination']:.6f} (각도: {basic['pitch_angle']:.2f}°)\n"
        
        report_str += f"\n[전체 기울기]\n"
        report_str += f"전체 기울기: {total['total_inclination_gravity']:.6f}\n"
        report_str += f"기울기 각도: {total['inclination_angle_deg']:.2f}°\n"
        report_str += f"기울기 방향: {direction['direction_angle_deg']:.1f}°\n"
        
        report_str += f"\n[한국 시설안전공단 기준]\n"
        report_str += f"Roll 기울기: 1/{korean['roll']['fraction']:.0f} (등급: {korean['roll']['grade']})\n"
        report_str += f"Pitch 기울기: 1/{korean['pitch']['fraction']:.0f} (등급: {korean['pitch']['grade']})\n"
        report_str += f"전체 기울기: 1/{korean['total']['fraction']:.0f} (등급: {korean['total']['grade']})\n"
        
        report_str += f"\n[안전성 평가]\n"
        worst_grade = max(korean['roll']['grade'], korean['pitch']['grade'], korean['total']['grade'])
        grade_descriptions = {
            'a': '매우 안전 (정밀장비 설치 가능)',
            'b': '안전 (구조물 균열 발생 한계)',
            'c': '주의 (육안으로 기울기 감지 가능)',
            'd': '위험 (구조적 손상 예상)',
            'e': '매우 위험 (즉시 조치 필요)'
        }
        report_str += f"최종 등급: {worst_grade} - {grade_descriptions[worst_grade]}\n"
        return report_str

    def closeEvent(self, event):
        """프로그램 종료 시 리소스를 정리하는 메서드"""
        self.serial_thread.stop()  # 시리얼 스레드 정지
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())