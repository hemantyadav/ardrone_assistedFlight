# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'dronetools.ui'
#
# Created: Thu Apr 11 14:23:57 2013
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
        MainWindow.resize(891, 600)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.pushButtonSwapCamera = QtGui.QPushButton(self.centralwidget)
        self.pushButtonSwapCamera.setGeometry(QtCore.QRect(780, 10, 101, 27))
        self.pushButtonSwapCamera.setObjectName(_fromUtf8("pushButtonSwapCamera"))
        self.groupBox = QtGui.QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QtCore.QRect(10, 10, 501, 401))
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
        self.labelSetCommand = QtGui.QLabel(self.groupBox)
        self.labelSetCommand.setGeometry(QtCore.QRect(10, 240, 461, 17))
        self.labelSetCommand.setObjectName(_fromUtf8("labelSetCommand"))
        self.labelLandCond = QtGui.QLabel(self.groupBox)
        self.labelLandCond.setGeometry(QtCore.QRect(10, 260, 431, 17))
        self.labelLandCond.setObjectName(_fromUtf8("labelLandCond"))
        self.labelLandCondVals = QtGui.QLabel(self.groupBox)
        self.labelLandCondVals.setGeometry(QtCore.QRect(10, 280, 481, 17))
        self.labelLandCondVals.setObjectName(_fromUtf8("labelLandCondVals"))
        self.labelAcceptableVelocity = QtGui.QLabel(self.groupBox)
        self.labelAcceptableVelocity.setGeometry(QtCore.QRect(10, 300, 311, 17))
        self.labelAcceptableVelocity.setObjectName(_fromUtf8("labelAcceptableVelocity"))
        self.labelAcceptableOffset = QtGui.QLabel(self.groupBox)
        self.labelAcceptableOffset.setGeometry(QtCore.QRect(10, 320, 311, 17))
        self.labelAcceptableOffset.setObjectName(_fromUtf8("labelAcceptableOffset"))
        self.pushButtonTrim = QtGui.QPushButton(self.centralwidget)
        self.pushButtonTrim.setGeometry(QtCore.QRect(780, 50, 101, 27))
        self.pushButtonTrim.setObjectName(_fromUtf8("pushButtonTrim"))
        self.groupBoxNavData = QtGui.QGroupBox(self.centralwidget)
        self.groupBoxNavData.setGeometry(QtCore.QRect(530, 0, 231, 291))
        self.groupBoxNavData.setObjectName(_fromUtf8("groupBoxNavData"))
        self.labelNavDataAltd = QtGui.QLabel(self.groupBoxNavData)
        self.labelNavDataAltd.setGeometry(QtCore.QRect(0, 20, 131, 17))
        self.labelNavDataAltd.setObjectName(_fromUtf8("labelNavDataAltd"))
        self.labelNavDataVelocity = QtGui.QLabel(self.groupBoxNavData)
        self.labelNavDataVelocity.setGeometry(QtCore.QRect(0, 40, 221, 17))
        self.labelNavDataVelocity.setObjectName(_fromUtf8("labelNavDataVelocity"))
        self.labelNavDataRot = QtGui.QLabel(self.groupBoxNavData)
        self.labelNavDataRot.setGeometry(QtCore.QRect(0, 80, 231, 17))
        self.labelNavDataRot.setObjectName(_fromUtf8("labelNavDataRot"))
        self.labelNavDataBattery = QtGui.QLabel(self.groupBoxNavData)
        self.labelNavDataBattery.setGeometry(QtCore.QRect(0, 100, 111, 17))
        self.labelNavDataBattery.setObjectName(_fromUtf8("labelNavDataBattery"))
        self.labelNavDataAccel = QtGui.QLabel(self.groupBoxNavData)
        self.labelNavDataAccel.setGeometry(QtCore.QRect(0, 60, 221, 17))
        self.labelNavDataAccel.setObjectName(_fromUtf8("labelNavDataAccel"))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 891, 25))
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
        self.labelSetCommand.setText(QtGui.QApplication.translate("MainWindow", "pitch: 0, roll: 0, yaw: 0, zvel: 0", None, QtGui.QApplication.UnicodeUTF8))
        self.labelLandCond.setText(QtGui.QApplication.translate("MainWindow", "Landing Conditions: Not Met", None, QtGui.QApplication.UnicodeUTF8))
        self.labelLandCondVals.setText(QtGui.QApplication.translate("MainWindow", "ALTD: 0, Veldrone: 0, OffsetMag: 0, AOM: 0", None, QtGui.QApplication.UnicodeUTF8))
        self.labelAcceptableVelocity.setText(QtGui.QApplication.translate("MainWindow", "Acceptable Velocity: No", None, QtGui.QApplication.UnicodeUTF8))
        self.labelAcceptableOffset.setText(QtGui.QApplication.translate("MainWindow", "Acceptable Offset: No", None, QtGui.QApplication.UnicodeUTF8))
        self.pushButtonTrim.setText(QtGui.QApplication.translate("MainWindow", "Trim", None, QtGui.QApplication.UnicodeUTF8))
        self.groupBoxNavData.setTitle(QtGui.QApplication.translate("MainWindow", "NavData", None, QtGui.QApplication.UnicodeUTF8))
        self.labelNavDataAltd.setText(QtGui.QApplication.translate("MainWindow", "altd: 0000", None, QtGui.QApplication.UnicodeUTF8))
        self.labelNavDataVelocity.setText(QtGui.QApplication.translate("MainWindow", "vx: 0.0000 vy:0.0000 vz: 0.0000", None, QtGui.QApplication.UnicodeUTF8))
        self.labelNavDataRot.setText(QtGui.QApplication.translate("MainWindow", "rotX: 0.0000 rotY: 0.0000 rotZ: 0.0000", None, QtGui.QApplication.UnicodeUTF8))
        self.labelNavDataBattery.setText(QtGui.QApplication.translate("MainWindow", "battery: 100%", None, QtGui.QApplication.UnicodeUTF8))
        self.labelNavDataAccel.setText(QtGui.QApplication.translate("MainWindow", "ax: 0.0000 ay:0.0000 az: 0.0000", None, QtGui.QApplication.UnicodeUTF8))

