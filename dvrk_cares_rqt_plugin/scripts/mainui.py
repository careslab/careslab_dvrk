# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainwindow.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.setEnabled(True)
        MainWindow.resize(619, 548)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QtCore.QRect(10, 10, 91, 111))
        self.groupBox.setObjectName("groupBox")
        self.gridLayoutWidget = QtWidgets.QWidget(self.groupBox)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(0, 20, 91, 91))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.powerOnBtn = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.powerOnBtn.setObjectName("powerOnBtn")
        self.gridLayout.addWidget(self.powerOnBtn, 0, 0, 1, 1)
        self.homeBtn = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.homeBtn.setEnabled(False)
        self.homeBtn.setObjectName("homeBtn")
        self.gridLayout.addWidget(self.homeBtn, 2, 0, 1, 1)
        self.powerOffBtn = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.powerOffBtn.setEnabled(False)
        self.powerOffBtn.setObjectName("powerOffBtn")
        self.gridLayout.addWidget(self.powerOffBtn, 1, 0, 1, 1)
        self.groupBox_2 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_2.setGeometry(QtCore.QRect(10, 130, 121, 81))
        self.groupBox_2.setObjectName("groupBox_2")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.groupBox_2)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(0, 20, 119, 61))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.SimulationRadioBtn = QtWidgets.QRadioButton(self.verticalLayoutWidget)
        self.SimulationRadioBtn.setEnabled(False)
        self.SimulationRadioBtn.setObjectName("SimulationRadioBtn")
        self.verticalLayout.addWidget(self.SimulationRadioBtn)
        self.HardwareRadioBtn = QtWidgets.QRadioButton(self.verticalLayoutWidget)
        self.HardwareRadioBtn.setEnabled(False)
        self.HardwareRadioBtn.setObjectName("HardwareRadioBtn")
        self.verticalLayout.addWidget(self.HardwareRadioBtn)
        self.groupBox_3 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_3.setGeometry(QtCore.QRect(10, 220, 171, 191))
        self.groupBox_3.setObjectName("groupBox_3")
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.groupBox_3)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(0, 20, 171, 170))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.TeleopRadioBtn = QtWidgets.QRadioButton(self.verticalLayoutWidget_2)
        self.TeleopRadioBtn.setEnabled(False)
        self.TeleopRadioBtn.setObjectName("TeleopRadioBtn")
        self.verticalLayout_2.addWidget(self.TeleopRadioBtn)
        self.AutocameraRadioBtn = QtWidgets.QRadioButton(self.verticalLayoutWidget_2)
        self.AutocameraRadioBtn.setEnabled(False)
        self.AutocameraRadioBtn.setAcceptDrops(False)
        self.AutocameraRadioBtn.setCheckable(True)
        self.AutocameraRadioBtn.setChecked(False)
        self.AutocameraRadioBtn.setAutoRepeat(False)
        self.AutocameraRadioBtn.setObjectName("AutocameraRadioBtn")
        self.verticalLayout_2.addWidget(self.AutocameraRadioBtn)
        self.ClutchNMoveRadioBtn = QtWidgets.QRadioButton(self.verticalLayoutWidget_2)
        self.ClutchNMoveRadioBtn.setEnabled(False)
        self.ClutchNMoveRadioBtn.setObjectName("ClutchNMoveRadioBtn")
        self.verticalLayout_2.addWidget(self.ClutchNMoveRadioBtn)
        self.JoystickCtrlRadioBtn = QtWidgets.QRadioButton(self.verticalLayoutWidget_2)
        self.JoystickCtrlRadioBtn.setEnabled(False)
        self.JoystickCtrlRadioBtn.setObjectName("JoystickCtrlRadioBtn")
        self.verticalLayout_2.addWidget(self.JoystickCtrlRadioBtn)
        self.OculusRadioBtn = QtWidgets.QRadioButton(self.verticalLayoutWidget_2)
        self.OculusRadioBtn.setEnabled(False)
        self.OculusRadioBtn.setObjectName("OculusRadioBtn")
        self.verticalLayout_2.addWidget(self.OculusRadioBtn)
        self.ClutchlessRadioBtn = QtWidgets.QRadioButton(self.verticalLayoutWidget_2)
        self.ClutchlessRadioBtn.setEnabled(False)
        self.ClutchlessRadioBtn.setObjectName("ClutchlessRadioBtn")
        self.verticalLayout_2.addWidget(self.ClutchlessRadioBtn)
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(190, 270, 186, 31))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.TrackLeft = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.TrackLeft.setObjectName("TrackLeft")
        self.TrackLeft.setVisible(False)
        self.horizontalLayout.addWidget(self.TrackLeft)
        self.TrackRight = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.TrackRight.setEnabled(True)
        self.TrackRight.setVisible(False)
        self.TrackRight.setCheckable(False)
        self.TrackRight.setObjectName("TrackRight")
        self.horizontalLayout.addWidget(self.TrackRight)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionTest = QtWidgets.QAction(MainWindow)
        self.actionTest.setObjectName("actionTest")

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "da Vinci Controller"))
        self.groupBox.setTitle(_translate("MainWindow", "Power"))
        self.powerOnBtn.setText(_translate("MainWindow", "On"))
        self.homeBtn.setText(_translate("MainWindow", "Home"))
        self.powerOffBtn.setText(_translate("MainWindow", "Off"))
        self.groupBox_2.setTitle(_translate("MainWindow", "Operation Mode"))
        self.SimulationRadioBtn.setText(_translate("MainWindow", "Simulation"))
        self.HardwareRadioBtn.setText(_translate("MainWindow", "Hardware"))
        self.groupBox_3.setTitle(_translate("MainWindow", "Camera Control Method"))
        self.TeleopRadioBtn.setText(_translate("MainWindow", "Teleop"))
        self.AutocameraRadioBtn.setText(_translate("MainWindow", "Autocamera"))
        self.ClutchNMoveRadioBtn.setText(_translate("MainWindow", "Clutch and Move"))
        self.JoystickCtrlRadioBtn.setText(_translate("MainWindow", "Joystick Control"))
        self.OculusRadioBtn.setText(_translate("MainWindow", "Oculus"))
        self.ClutchlessRadioBtn.setText(_translate("MainWindow", "Clutchless System"))
        self.TrackLeft.setText(_translate("MainWindow", "Left"))
        self.TrackRight.setText(_translate("MainWindow", "Right"))
        self.actionTest.setText(_translate("MainWindow", "test"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
