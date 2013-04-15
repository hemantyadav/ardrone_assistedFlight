# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'dronetools.ui'
#
# Created: Sun Mar 31 23:36:55 2013
#      by: PyQt4 UI code generator 4.9.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(800, 600)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.pushButtonSwapCamera = QtGui.QPushButton(self.centralwidget)
        self.pushButtonSwapCamera.setGeometry(QtCore.QRect(690, 10, 101, 27))
        self.pushButtonSwapCamera.setObjectName(_fromUtf8("pushButtonSwapCamera"))
        self.groupBox = QtGui.QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QtCore.QRect(10, 10, 371, 301))
        self.groupBox.setObjectName(_fromUtf8("groupBox"))
        self.label = QtGui.QLabel(self.groupBox)
        self.label.setGeometry(QtCore.QRect(10, 220, 41, 17))
        self.label.setObjectName(_fromUtf8("label"))
        self.framePad = QtGui.QFrame(self.groupBox)
        self.framePad.setGeometry(QtCore.QRect(10, 30, 320, 180))
        self.framePad.setFrameShape(QtGui.QFrame.StyledPanel)
        self.framePad.setFrameShadow(QtGui.QFrame.Raised)
        self.framePad.setObjectName(_fromUtf8("framePad"))
        self.labelX = QtGui.QLabel(self.framePad)
        self.labelX.setGeometry(QtCore.QRect(145, 75, 30, 30))
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelX.setFont(font)
        self.labelX.setScaledContents(False)
        self.labelX.setAlignment(QtCore.Qt.AlignCenter)
        self.labelX.setObjectName(_fromUtf8("labelX"))
        self.labelDroneMarker = QtGui.QLabel(self.framePad)
        self.labelDroneMarker.setGeometry(QtCore.QRect(150, 110, 30, 30))
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelDroneMarker.setFont(font)
        self.labelDroneMarker.setScaledContents(False)
        self.labelDroneMarker.setAlignment(QtCore.Qt.AlignCenter)
        self.labelDroneMarker.setObjectName(_fromUtf8("labelDroneMarker"))
        self.labelAutoLandStatus = QtGui.QLabel(self.groupBox)
        self.labelAutoLandStatus.setGeometry(QtCore.QRect(60, 220, 261, 17))
        self.labelAutoLandStatus.setObjectName(_fromUtf8("labelAutoLandStatus"))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QtGui.QApplication.translate("MainWindow", "MainWindow", None, QtGui.QApplication.UnicodeUTF8))
        self.pushButtonSwapCamera.setText(QtGui.QApplication.translate("MainWindow", "Swap Camera", None, QtGui.QApplication.UnicodeUTF8))
        self.groupBox.setTitle(QtGui.QApplication.translate("MainWindow", "Auto Landing", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("MainWindow", "Status: ", None, QtGui.QApplication.UnicodeUTF8))
        self.labelX.setText(QtGui.QApplication.translate("MainWindow", "X", None, QtGui.QApplication.UnicodeUTF8))
        self.labelDroneMarker.setText(QtGui.QApplication.translate("MainWindow", "O", None, QtGui.QApplication.UnicodeUTF8))
        self.labelAutoLandStatus.setText(QtGui.QApplication.translate("MainWindow", "Off", None, QtGui.QApplication.UnicodeUTF8))

