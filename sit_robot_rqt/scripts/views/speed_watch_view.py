# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'speed_watch.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(504, 472)
        self.gridLayout_7 = QtWidgets.QGridLayout(Form)
        self.gridLayout_7.setObjectName("gridLayout_7")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtWidgets.QLabel(Form)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.label_2 = QtWidgets.QLabel(Form)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 0, 0, 1, 1)
        self.expVxLabel = QtWidgets.QLabel(Form)
        self.expVxLabel.setText("")
        self.expVxLabel.setObjectName("expVxLabel")
        self.gridLayout.addWidget(self.expVxLabel, 0, 1, 1, 1)
        self.label_3 = QtWidgets.QLabel(Form)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 1, 0, 1, 1)
        self.expVyLabel = QtWidgets.QLabel(Form)
        self.expVyLabel.setText("")
        self.expVyLabel.setObjectName("expVyLabel")
        self.gridLayout.addWidget(self.expVyLabel, 1, 1, 1, 1)
        self.label_4 = QtWidgets.QLabel(Form)
        self.label_4.setObjectName("label_4")
        self.gridLayout.addWidget(self.label_4, 2, 0, 1, 1)
        self.expWLabel = QtWidgets.QLabel(Form)
        self.expWLabel.setText("")
        self.expWLabel.setObjectName("expWLabel")
        self.gridLayout.addWidget(self.expWLabel, 2, 1, 1, 1)
        self.horizontalLayout.addLayout(self.gridLayout)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.gridLayout_7.addLayout(self.horizontalLayout, 0, 0, 1, 1)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_8 = QtWidgets.QLabel(Form)
        self.label_8.setObjectName("label_8")
        self.horizontalLayout_2.addWidget(self.label_8)
        self.gridLayout_2 = QtWidgets.QGridLayout()
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.label_9 = QtWidgets.QLabel(Form)
        self.label_9.setObjectName("label_9")
        self.gridLayout_2.addWidget(self.label_9, 0, 0, 1, 1)
        self.realVxLabel = QtWidgets.QLabel(Form)
        self.realVxLabel.setText("")
        self.realVxLabel.setObjectName("realVxLabel")
        self.gridLayout_2.addWidget(self.realVxLabel, 0, 1, 1, 1)
        self.label_11 = QtWidgets.QLabel(Form)
        self.label_11.setObjectName("label_11")
        self.gridLayout_2.addWidget(self.label_11, 1, 0, 1, 1)
        self.realVyLabel = QtWidgets.QLabel(Form)
        self.realVyLabel.setText("")
        self.realVyLabel.setObjectName("realVyLabel")
        self.gridLayout_2.addWidget(self.realVyLabel, 1, 1, 1, 1)
        self.label_13 = QtWidgets.QLabel(Form)
        self.label_13.setObjectName("label_13")
        self.gridLayout_2.addWidget(self.label_13, 2, 0, 1, 1)
        self.realWLabel = QtWidgets.QLabel(Form)
        self.realWLabel.setText("")
        self.realWLabel.setObjectName("realWLabel")
        self.gridLayout_2.addWidget(self.realWLabel, 2, 1, 1, 1)
        self.horizontalLayout_2.addLayout(self.gridLayout_2)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem1)
        self.gridLayout_7.addLayout(self.horizontalLayout_2, 0, 1, 1, 1)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.gridLayout_3 = QtWidgets.QGridLayout()
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.angVelYLabel = QtWidgets.QLabel(Form)
        self.angVelYLabel.setObjectName("angVelYLabel")
        self.gridLayout_3.addWidget(self.angVelYLabel, 1, 1, 1, 1)
        self.label_7 = QtWidgets.QLabel(Form)
        self.label_7.setObjectName("label_7")
        self.gridLayout_3.addWidget(self.label_7, 2, 0, 1, 1)
        self.angVelXLabel = QtWidgets.QLabel(Form)
        self.angVelXLabel.setObjectName("angVelXLabel")
        self.gridLayout_3.addWidget(self.angVelXLabel, 0, 1, 1, 1)
        self.label_5 = QtWidgets.QLabel(Form)
        self.label_5.setObjectName("label_5")
        self.gridLayout_3.addWidget(self.label_5, 0, 0, 1, 1)
        self.label_6 = QtWidgets.QLabel(Form)
        self.label_6.setObjectName("label_6")
        self.gridLayout_3.addWidget(self.label_6, 1, 0, 1, 1)
        self.angVelZLabel = QtWidgets.QLabel(Form)
        self.angVelZLabel.setObjectName("angVelZLabel")
        self.gridLayout_3.addWidget(self.angVelZLabel, 2, 1, 1, 1)
        spacerItem2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_3.addItem(spacerItem2, 1, 2, 1, 1)
        self.horizontalLayout_3.addLayout(self.gridLayout_3)
        self.gridLayout_7.addLayout(self.horizontalLayout_3, 1, 0, 1, 1)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.gridLayout_4 = QtWidgets.QGridLayout()
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.label_12 = QtWidgets.QLabel(Form)
        self.label_12.setObjectName("label_12")
        self.gridLayout_4.addWidget(self.label_12, 2, 0, 1, 1)
        self.linAccXLabel = QtWidgets.QLabel(Form)
        self.linAccXLabel.setObjectName("linAccXLabel")
        self.gridLayout_4.addWidget(self.linAccXLabel, 0, 1, 1, 1)
        self.linAccZLabel = QtWidgets.QLabel(Form)
        self.linAccZLabel.setObjectName("linAccZLabel")
        self.gridLayout_4.addWidget(self.linAccZLabel, 2, 1, 1, 1)
        self.label_10 = QtWidgets.QLabel(Form)
        self.label_10.setObjectName("label_10")
        self.gridLayout_4.addWidget(self.label_10, 1, 0, 1, 1)
        self.label_14 = QtWidgets.QLabel(Form)
        self.label_14.setObjectName("label_14")
        self.gridLayout_4.addWidget(self.label_14, 0, 0, 1, 1)
        self.linAccYLabel = QtWidgets.QLabel(Form)
        self.linAccYLabel.setObjectName("linAccYLabel")
        self.gridLayout_4.addWidget(self.linAccYLabel, 1, 1, 1, 1)
        self.horizontalLayout_4.addLayout(self.gridLayout_4)
        spacerItem3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem3)
        self.gridLayout_7.addLayout(self.horizontalLayout_4, 1, 1, 1, 1)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.gridLayout_6 = QtWidgets.QGridLayout()
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.label_17 = QtWidgets.QLabel(Form)
        self.label_17.setObjectName("label_17")
        self.gridLayout_6.addWidget(self.label_17, 0, 0, 1, 1)
        self.oriXLabel = QtWidgets.QLabel(Form)
        self.oriXLabel.setObjectName("oriXLabel")
        self.gridLayout_6.addWidget(self.oriXLabel, 0, 1, 1, 1)
        self.label_18 = QtWidgets.QLabel(Form)
        self.label_18.setObjectName("label_18")
        self.gridLayout_6.addWidget(self.label_18, 1, 0, 1, 1)
        self.oriYLabel = QtWidgets.QLabel(Form)
        self.oriYLabel.setObjectName("oriYLabel")
        self.gridLayout_6.addWidget(self.oriYLabel, 1, 1, 1, 1)
        self.label_19 = QtWidgets.QLabel(Form)
        self.label_19.setObjectName("label_19")
        self.gridLayout_6.addWidget(self.label_19, 2, 0, 1, 1)
        self.oriZLabel = QtWidgets.QLabel(Form)
        self.oriZLabel.setObjectName("oriZLabel")
        self.gridLayout_6.addWidget(self.oriZLabel, 2, 1, 1, 1)
        self.label_20 = QtWidgets.QLabel(Form)
        self.label_20.setObjectName("label_20")
        self.gridLayout_6.addWidget(self.label_20, 3, 0, 1, 1)
        self.oriWLabel = QtWidgets.QLabel(Form)
        self.oriWLabel.setObjectName("oriWLabel")
        self.gridLayout_6.addWidget(self.oriWLabel, 3, 1, 1, 1)
        self.horizontalLayout_5.addLayout(self.gridLayout_6)
        spacerItem4 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_5.addItem(spacerItem4)
        self.gridLayout_7.addLayout(self.horizontalLayout_5, 2, 0, 1, 1)
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.gridLayout_5 = QtWidgets.QGridLayout()
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.eulerPitchLabel = QtWidgets.QLabel(Form)
        self.eulerPitchLabel.setObjectName("eulerPitchLabel")
        self.gridLayout_5.addWidget(self.eulerPitchLabel, 1, 1, 1, 1)
        self.eulerRowLabel = QtWidgets.QLabel(Form)
        self.eulerRowLabel.setObjectName("eulerRowLabel")
        self.gridLayout_5.addWidget(self.eulerRowLabel, 2, 1, 1, 1)
        self.label_15 = QtWidgets.QLabel(Form)
        self.label_15.setObjectName("label_15")
        self.gridLayout_5.addWidget(self.label_15, 0, 0, 1, 1)
        self.label_16 = QtWidgets.QLabel(Form)
        self.label_16.setObjectName("label_16")
        self.gridLayout_5.addWidget(self.label_16, 1, 0, 1, 1)
        self.eulerYawLabel = QtWidgets.QLabel(Form)
        self.eulerYawLabel.setObjectName("eulerYawLabel")
        self.gridLayout_5.addWidget(self.eulerYawLabel, 0, 1, 1, 1)
        self.label_21 = QtWidgets.QLabel(Form)
        self.label_21.setObjectName("label_21")
        self.gridLayout_5.addWidget(self.label_21, 2, 0, 1, 1)
        spacerItem5 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_5.addItem(spacerItem5, 1, 2, 1, 1)
        self.horizontalLayout_6.addLayout(self.gridLayout_5)
        self.gridLayout_7.addLayout(self.horizontalLayout_6, 2, 1, 1, 1)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.label.setText(_translate("Form", "期望\n"
"速度"))
        self.label_2.setText(_translate("Form", "vx:"))
        self.label_3.setText(_translate("Form", "vy:"))
        self.label_4.setText(_translate("Form", "w:"))
        self.label_8.setText(_translate("Form", "真实\n"
"速度"))
        self.label_9.setText(_translate("Form", "vx:"))
        self.label_11.setText(_translate("Form", "vy:"))
        self.label_13.setText(_translate("Form", "w:"))
        self.angVelYLabel.setText(_translate("Form", "TextLabel"))
        self.label_7.setText(_translate("Form", "角速度z:"))
        self.angVelXLabel.setText(_translate("Form", "TextLabel"))
        self.label_5.setText(_translate("Form", "角速度x:"))
        self.label_6.setText(_translate("Form", "角速度y:"))
        self.angVelZLabel.setText(_translate("Form", "TextLabel"))
        self.label_12.setText(_translate("Form", "线加速度z:"))
        self.linAccXLabel.setText(_translate("Form", "TextLabel"))
        self.linAccZLabel.setText(_translate("Form", "TextLabel"))
        self.label_10.setText(_translate("Form", "线加速度y:"))
        self.label_14.setText(_translate("Form", "线加速度x:"))
        self.linAccYLabel.setText(_translate("Form", "TextLabel"))
        self.label_17.setText(_translate("Form", "四元数x:"))
        self.oriXLabel.setText(_translate("Form", "TextLabel"))
        self.label_18.setText(_translate("Form", "四元数y:"))
        self.oriYLabel.setText(_translate("Form", "TextLabel"))
        self.label_19.setText(_translate("Form", "四元数z:"))
        self.oriZLabel.setText(_translate("Form", "TextLabel"))
        self.label_20.setText(_translate("Form", "四元数w:"))
        self.oriWLabel.setText(_translate("Form", "TextLabel"))
        self.eulerPitchLabel.setText(_translate("Form", "TextLabel"))
        self.eulerRowLabel.setText(_translate("Form", "TextLabel"))
        self.label_15.setText(_translate("Form", "欧拉角yaw:"))
        self.label_16.setText(_translate("Form", "欧拉角pitch:"))
        self.eulerYawLabel.setText(_translate("Form", "TextLabel"))
        self.label_21.setText(_translate("Form", "欧拉角row:"))