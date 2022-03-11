# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'shovel.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(107, 86)
        self.verticalLayout = QtWidgets.QVBoxLayout(Form)
        self.verticalLayout.setObjectName("verticalLayout")
        self.up_button = QtWidgets.QPushButton(Form)
        self.up_button.setObjectName("up_button")
        self.verticalLayout.addWidget(self.up_button)
        self.down_button = QtWidgets.QPushButton(Form)
        self.down_button.setObjectName("down_button")
        self.verticalLayout.addWidget(self.down_button)
        self.forward_button = QtWidgets.QPushButton(Form)
        self.forward_button.setObjectName("forward_button")
        self.verticalLayout.addWidget(self.forward_button)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.up_button.setText(_translate("Form", "上升起落架"))
        self.down_button.setText(_translate("Form", "下降起落架"))
        self.forward_button.setText(_translate("Form", "前进按钮"))
