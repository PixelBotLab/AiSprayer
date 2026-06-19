# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import QFrame, QLabel
from PyQt5.QtCore import Qt, pyqtSignal, QPoint

class ClickableFrame(QFrame):
    clicked = pyqtSignal()
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit()

class ClickableLabel(QLabel):
    """A QLabel that emits custom clicked signal with relative coordinates."""
    clicked_pos = pyqtSignal(QPoint)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(640, 480)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background-color: #111; border: 1px solid #333;")

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked_pos.emit(event.pos())
