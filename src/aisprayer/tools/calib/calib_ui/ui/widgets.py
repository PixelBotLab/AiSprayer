# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import QFrame, QLabel, QSizePolicy
from PyQt5.QtCore import Qt, pyqtSignal, QPoint

class ClickableFrame(QFrame):
    clicked = pyqtSignal()
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit()

class AspectLabel(QLabel):
    """A QLabel that maintains a fixed height-for-width aspect ratio."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.aspect_ratio = 800 / 1280  # Default 16:10 aspect ratio (height / width)
        policy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        policy.setHeightForWidth(True)
        self.setSizePolicy(policy)

    def heightForWidth(self, width):
        return int(width * self.aspect_ratio)

class ClickableLabel(AspectLabel):
    """A QLabel that emits custom clicked signal with relative coordinates, right click, and hover/move coordinates."""
    clicked_pos = pyqtSignal(QPoint)
    right_clicked = pyqtSignal()
    mouse_moved = pyqtSignal(QPoint)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(480, 360)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background-color: #111; border: 1px solid #333;")
        self.setMouseTracking(True)  # 开启鼠标跟踪，使 mouseMoveEvent 在不按下按键时也能触发

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked_pos.emit(event.pos())
        elif event.button() == Qt.RightButton:
            self.right_clicked.emit()

    def mouseMoveEvent(self, event):
        self.mouse_moved.emit(event.pos())
