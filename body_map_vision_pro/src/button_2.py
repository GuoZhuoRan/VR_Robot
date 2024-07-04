import os
import sys
from time import sleep
from PyQt5.QtWidgets import QMessageBox,QApplication, QWidget, QLabel,QListWidget, QLineEdit, QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout
import subprocess
from PyQt5.QtCore import QThread, pyqtSignal
import threading


import  other_file.avp_stream.streamer_dev  as streamer_dev
import numpy as np
import vr_test



running=True

def prin():
    print('start listening ')

def calibrate():

    vr_test.calibrate()
    pass


class ScriptRunner(QThread):

    error_occurred = pyqtSignal(str)

    def __init__(self, script_path):
        super().__init__()
        self.script_path = script_path

    def run(self):
        try:
            subprocess.run(['python', self.script_path], check=True)
        except subprocess.CalledProcessError as e:
            self.error_occurred.emit(f'Failed to run script: {e}')
        except Exception as e:
            self.error_occurred.emit(f'An error occurred: {e}')

class RobotControllerUI(QWidget):
    def __init__(self):
        super().__init__()

        self.initUI()
        

    def show_error_message(self, message):
        QMessageBox.critical(self, 'Error', message)    
    
    def run_script(self):
        pass
    
    def start_vp_server(self):
        # self.script_runner = ScriptRunner(script_path)
        # self.script_runner.error_occurred.connect(self.show_error_message)
        # self.script_runner.start()
        self.start_pv_btn.setEnabled(False)
        streamer_dev.running=True
        listener_thread = threading.Thread(target=streamer_dev.start_vision_pro_server)
        listener_thread.start()
    

    def start_calc_server(self):
        self.start_processing_btn.setEnabled(False)
        listener_thread = threading.Thread(target=vr_test.main)
        listener_thread.start()
        pass

    def on_selection_change(self):
        selected_items = self.listWidget.selectedItems()
        if selected_items:
            selected_text = selected_items[0].text()
            self.label_type.setText(f'Selected: {selected_text}')
            vr_test.robot_type = selected_text
        else:
            self.label_type.setText('No selection') 


    def initUI(self):        

        # Grid Layout
        grid = QGridLayout()
        self.setLayout(grid)
        
        # Calibration Controls
        grid.addWidget(QLabel('Calibration Controls'), 0, 0)
        self.start_pv_btn = QPushButton('Start PV')
        # self.end_pv_btn = QPushButton('End PV')
        self.start_processing_btn = QPushButton('Start processing')
        # self.end_processing_btn = QPushButton('End processing')
        self.calibration_headpose_btn = QPushButton('Calibration headpose')
        
        #Guozi: Add click functions
        self.start_pv_btn.clicked.connect(self.start_vp_server)
        # self.end_pv_btn.clicked.connect(stop_vp_server)
        self.start_processing_btn.clicked.connect(self.start_calc_server)
        # self.end_processing_btn.clicked.connect(self.run_script)
        self.calibration_headpose_btn.clicked.connect(calibrate)
        
        grid.addWidget(self.start_pv_btn, 1, 0)
        # grid.addWidget(self.end_pv_btn, 1, 1)
        grid.addWidget(self.start_processing_btn, 2, 0)
        # grid.addWidget(self.end_processing_btn, 2, 1)
        grid.addWidget(self.calibration_headpose_btn, 3, 0, 1, 2)
        
        # Calibrated Head Pose
        grid.addWidget(QLabel('Calibrated head pose: r, p, y, x, y, z'), 4, 0, 1, 2)
        
        # Network Configuration
        grid.addWidget(QLabel(f'VsionPro IP {streamer_dev.vision_pro_ip} \r\nLocal IP: {vr_test.server_ip}:{vr_test.server_port}'), 5, 0)
        # self.local_ip_edit = QLineEdit()
        # grid.addWidget(self.local_ip_edit, 5, 1)
        
        # grid.addWidget(QLabel('Local PORT:'), 6, 0)
        # self.local_port_edit = QLineEdit()
        # grid.addWidget(self.local_port_edit, 6, 1)
        
        grid.addWidget(QLabel(f'Destination IP: {vr_test.robot_ip}:{vr_test.robot_port}'), 7, 0)
        # self.destination_ip_edit = QLineEdit()
        # grid.addWidget(self.destination_ip_edit, 7, 1)
        self.label_type =QLabel(f'Type:{vr_test.robot_type}')
        grid.addWidget(self.label_type, 8, 0)

        # Create a QListWidget with single selection mode
        self.listWidget = QListWidget()
        self.listWidget.addItems(['mujoco', 'robot'])
        self.listWidget.setSelectionMode(QListWidget.SingleSelection)

        grid.addWidget(self.listWidget)

        # Connect the selection change signal to the slot
        self.listWidget.itemSelectionChanged.connect(self.on_selection_change)
        
        # self.type_edit = QLineEdit('robot')
        # grid.addWidget(self.type_edit, 8, 1)
        
        self.label_left_hand=QLabel('L hand Pose: r, p, y, x, y, z')
        self.label_right_hand=QLabel('R hand Pose: r, p, y, x, y, z')
        self.label_left_fingers=QLabel('L Finger : f1, f2, f3, f4, f5, f6')
        self.label_right_fingers=QLabel('R Finger : f1, f2, f3, f4, f5, f6')

        # Pose Data
        grid.addWidget(self.label_left_hand, 9, 0, 1, 2)
        grid.addWidget(self.label_right_hand, 10, 0, 1, 2)
        grid.addWidget(self.label_left_fingers, 11, 0, 1, 2)
        grid.addWidget(self.label_right_fingers, 12, 0, 1, 2)
        
        # Set window properties
        self.setWindowTitle('Robot Controller')
        self.setGeometry(100, 100, 400, 300)

        listener_thread = threading.Thread(target=self.update_val)
        listener_thread.start()

    
    def update_val(self):
        while running:
            formatted_elements = [f'{x:.2f}' for x in vr_test.left_hand[:6]]

            self.label_left_hand.setText(f'L h:{formatted_elements}')

            formatted_elements = [f'{x:.2f}' for x in vr_test.right_hand[:6]]
            self.label_right_hand.setText(f'R h:{formatted_elements}')

            formatted_elements = [f'{x:.2f}' for x in vr_test.left_hand[6:]]
            self.label_left_fingers.setText(f'L f:{formatted_elements}')

            formatted_elements = [f'{x:.2f}' for x in vr_test.left_hand[6:]]
            self.label_right_fingers.setText(f'R f:{formatted_elements}')
            sleep(0.1)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = RobotControllerUI()
    ex.show()
    app.exec_()
    print("exit")
    # QApplication.instance().quit()
    os._exit(0)

