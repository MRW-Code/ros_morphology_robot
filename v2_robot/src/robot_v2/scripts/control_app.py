from src.control import ControlWindow
import sys
from PyQt5.QtWidgets import QApplication
import os
import rospy

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ControlWindow()
    sys.exit(app.exec_())