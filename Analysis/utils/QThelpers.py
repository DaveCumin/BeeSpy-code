## taken from https://www.fundza.com/pyqt_pyside2/pyqt5_int_lineedit/index.html

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

class LabelledIntField(QWidget):
    def __init__(self, title, initial_value=None):
        QWidget.__init__(self)
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        self.label = QLabel()
        self.label.setText(title)
        self.label.setFont(QFont("Arial",weight=QFont.Bold))
        layout.addWidget(self.label)
        
        self.lineEdit = QLineEdit(self)
        self.lineEdit.setValidator(QIntValidator())
        if initial_value != None:
            self.lineEdit.setText(str(initial_value))
        layout.addWidget(self.lineEdit)
        layout.addStretch()
        
    def setLabelWidth(self, width):
        self.label.setFixedWidth(width)
        
    def setInputWidth(self, width):
        self.lineEdit.setFixedWidth(width)
    
    def on_value_changed(self, function):
        self.lineEdit.textChanged.connect(function)
        
    def getValue(self):
        return int(self.lineEdit.text())
    

class LabelledDoubleField(QWidget):
    def __init__(self, title, initial_value=None):
        QWidget.__init__(self)
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        self.label = QLabel()
        self.label.setText(title)
        self.label.setFont(QFont("Arial",weight=QFont.Bold))
        layout.addWidget(self.label)
        
        self.lineEdit = QLineEdit(self)
        self.lineEdit.setValidator(QDoubleValidator())
        if initial_value != None:
            self.lineEdit.setText(str(initial_value))
        layout.addWidget(self.lineEdit)
        layout.addStretch()
        
    def setLabelWidth(self, width):
        self.label.setFixedWidth(width)
        
    def setInputWidth(self, width):
        self.lineEdit.setFixedWidth(width)

    def on_value_changed(self, function):
        self.lineEdit.textChanged.connect(function)
        
    def getValue(self):
        return float(self.lineEdit.text())