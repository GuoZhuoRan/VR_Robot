import sys
import subprocess
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QMessageBox

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

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle('Run External Python Scripts Example')
        self.setGeometry(100, 100, 400, 200)

        # Button 1 - Run script1.py
        self.button1 = QPushButton('Run script1', self)
        self.button1.setGeometry(50, 50, 150, 40)
        self.button1.clicked.connect(self.run_script1)

        # Button 2 - Run script2.py
        self.button2 = QPushButton('Run script2', self)
        self.button2.setGeometry(220, 50, 150, 40)
        self.button2.clicked.connect(self.run_script2)

    def run_script1(self):
        self.run_script('/home/guozhuoran/catkin_ws/src/body_map_vision_pro/src/other_file/avp_stream/streamer_dev.py')

    def run_script2(self):
        self.run_script('/home/guozhuoran/catkin_ws/src/body_map_vision_pro/src/vr_test.py')

    def run_script(self, script_path):
        self.script_runner = ScriptRunner(script_path)
        self.script_runner.error_occurred.connect(self.show_error_message)
        self.script_runner.start()

    def show_error_message(self, message):
        QMessageBox.critical(self, 'Error', message)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
