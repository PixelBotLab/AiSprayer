#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys

# Fix PyQt5 conflict with OpenCV's built-in Qt plugins
try:
    import cv2  # Must import cv2 first because it overwrites QT_QPA_PLATFORM_PLUGIN_PATH
    import PyQt5
    pyqt5_dir = os.path.dirname(PyQt5.__file__)
    plugins_dir = os.path.join(pyqt5_dir, "Qt5", "plugins")
    if os.path.exists(plugins_dir):
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = plugins_dir
    else:
        plugins_dir_alt = os.path.join(pyqt5_dir, "Qt", "plugins")
        if os.path.exists(plugins_dir_alt):
            os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = plugins_dir_alt
except Exception as e:
    pass

# Setup paths
if getattr(sys, 'frozen', False):
    PROJECT_ROOT = os.path.dirname(sys.executable)
else:
    PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
    sys.path.append(os.path.join(PROJECT_ROOT, "src"))

from PyQt5.QtWidgets import QApplication
from aisprayer.tools.calib.calib_ui.ui.main_window import CalibMainWindow

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CalibMainWindow()
    window.show()
    sys.exit(app.exec_())
