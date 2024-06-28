import os
import sys
from time import sleep
from PyQt5.QtWidgets import QMessageBox,QApplication, QWidget, QLabel,QListWidget, QLineEdit, QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout
import subprocess
from PyQt5.QtCore import QThread, pyqtSignal
import threading


import  other_file.avp_stream.streamer_dev  as streamer_dev
import vr_test

import numpy as np

from vr_test import head_to_left_S,head_to_right_S



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
   
    

    def create_shoulder_layout(self, shoulder_name, coordinates):
        
        layout = QVBoxLayout()
        layout.addWidget(QLabel(shoulder_name))

        for coord in ['x', 'y', 'z']:
            coord_layout = QHBoxLayout()
            coord_label = QLabel(f'{coord.upper()}: {coordinates[coord]:.2f}')
            coord_layout.addWidget(coord_label)

            inc_btn = QPushButton('+')
            dec_btn = QPushButton('--')
            inc_btn.clicked.connect(lambda _, c=coord, l=coord_label: self.update_coordinate(coordinates['name'],coordinates, c, l, 0.1))

            dec_btn.clicked.connect(lambda _, c=coord, l=coord_label: self.update_coordinate(coordinates['name'],coordinates, c, l, -0.1))

            coord_layout.addWidget(inc_btn)
            coord_layout.addWidget(dec_btn)
            layout.addLayout(coord_layout)

        return layout
    
    


    def initUI(self):   

        global head_to_left_S
        global head_to_right_S     

        # Grid Layout
        grid = QGridLayout()
        self.setLayout(grid)
        
        # Calibration Controls
        grid.addWidget(QLabel('Calibration Controls'), 0, 0)
        
        #Guozi: add shoulder martrix parameters
        self.left_shoulder_name = 'Left Shoulder'
        self.left_shoulder = {'name':'left','x': head_to_left_S[0,-1], 'y': head_to_left_S[1,-1], 'z': head_to_left_S[2,-1]}
        
        self.right_shoulder_name = 'Right Shoulder'
        self.right_shoulder = {'name':'right','x': head_to_right_S[0,-1], 'y':head_to_right_S[1,-1], 'z':head_to_right_S[2,-1]}
        
        #Guozi: Add click functions
      
     
        
        
        
        

        #guozi:add left & right shoulder controller
        grid.addLayout(self.create_shoulder_layout(self.left_shoulder_name, self.left_shoulder), 13, 0)
        
        grid.addLayout(self.create_shoulder_layout(self.right_shoulder_name, self.right_shoulder), 13, 1)

        
        # Set window properties
        self.setGeometry(100, 100, 400, 300)


    def update_coordinate(self, hand_tag,coordinates, coord, label, increment):
        global head_to_left_S, head_to_right_S
        coordinates[coord] += increment
        if hand_tag=='left':
            head_to_left_S[0,-1]=coordinates['x']
            head_to_left_S[1,-1]=coordinates['y']
            head_to_left_S[2,-1]=coordinates['z']
        if hand_tag =='right':
            head_to_left_S[0,-1]=coordinates['x']
            head_to_left_S[1,-1]=coordinates['y']
            head_to_left_S[2,-1]=coordinates['z']

            

        label.setText(f'{coord.upper()}: {coordinates[coord]:.1f}')

    
   


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = RobotControllerUI()
    ex.show()
    app.exec_()
    print("exit")
    # QApplication.instance().quit()
    os._exit(0)

