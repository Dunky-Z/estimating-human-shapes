# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'test.ui'
#
# Created by: PyQt5 UI code generator 5.15.0
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_HumanEstimate(object):
    def setupUi(self, HumanEstimate):
        HumanEstimate.setObjectName("HumanEstimate")
        HumanEstimate.setEnabled(True)
        HumanEstimate.resize(1014, 801)
        self.centralwidget = QtWidgets.QWidget(HumanEstimate)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.measure = QtWidgets.QWidget(self.centralwidget)
        self.measure.setEnabled(True)
        self.measure.setMinimumSize(QtCore.QSize(250, 0))
        self.measure.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.measure.setAutoFillBackground(False)
        self.measure.setStyleSheet("border: 1px solid red")
        self.measure.setObjectName("measure")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.measure)
        self.verticalLayout.setObjectName("verticalLayout")
        self.widget_5 = QtWidgets.QWidget(self.measure)
        self.widget_5.setObjectName("widget_5")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget_5)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtWidgets.QLabel(self.widget_5)
        self.label.setMinimumSize(QtCore.QSize(56, 16))
        self.label.setLineWidth(1)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.doubleSpinBox = QtWidgets.QDoubleSpinBox(self.widget_5)
        self.doubleSpinBox.setObjectName("doubleSpinBox")
        self.horizontalLayout.addWidget(self.doubleSpinBox)
        self.verticalLayout.addWidget(self.widget_5)
        self.widget_6 = QtWidgets.QWidget(self.measure)
        self.widget_6.setObjectName("widget_6")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.widget_6)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_2 = QtWidgets.QLabel(self.widget_6)
        self.label_2.setMinimumSize(QtCore.QSize(56, 16))
        self.label_2.setLineWidth(1)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_2.addWidget(self.label_2)
        self.doubleSpinBox_2 = QtWidgets.QDoubleSpinBox(self.widget_6)
        self.doubleSpinBox_2.setObjectName("doubleSpinBox_2")
        self.horizontalLayout_2.addWidget(self.doubleSpinBox_2)
        self.verticalLayout.addWidget(self.widget_6)
        self.widget_9 = QtWidgets.QWidget(self.measure)
        self.widget_9.setObjectName("widget_9")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.widget_9)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label_5 = QtWidgets.QLabel(self.widget_9)
        self.label_5.setMinimumSize(QtCore.QSize(56, 16))
        self.label_5.setLineWidth(1)
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.horizontalLayout_5.addWidget(self.label_5)
        self.doubleSpinBox_5 = QtWidgets.QDoubleSpinBox(self.widget_9)
        self.doubleSpinBox_5.setObjectName("doubleSpinBox_5")
        self.horizontalLayout_5.addWidget(self.doubleSpinBox_5)
        self.verticalLayout.addWidget(self.widget_9)
        self.widget_10 = QtWidgets.QWidget(self.measure)
        self.widget_10.setObjectName("widget_10")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(self.widget_10)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.label_6 = QtWidgets.QLabel(self.widget_10)
        self.label_6.setMinimumSize(QtCore.QSize(56, 16))
        self.label_6.setLineWidth(1)
        self.label_6.setAlignment(QtCore.Qt.AlignCenter)
        self.label_6.setObjectName("label_6")
        self.horizontalLayout_6.addWidget(self.label_6)
        self.doubleSpinBox_6 = QtWidgets.QDoubleSpinBox(self.widget_10)
        self.doubleSpinBox_6.setObjectName("doubleSpinBox_6")
        self.horizontalLayout_6.addWidget(self.doubleSpinBox_6)
        self.verticalLayout.addWidget(self.widget_10)
        self.widget_11 = QtWidgets.QWidget(self.measure)
        self.widget_11.setObjectName("widget_11")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout(self.widget_11)
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.label_7 = QtWidgets.QLabel(self.widget_11)
        self.label_7.setMinimumSize(QtCore.QSize(56, 16))
        self.label_7.setLineWidth(1)
        self.label_7.setAlignment(QtCore.Qt.AlignCenter)
        self.label_7.setObjectName("label_7")
        self.horizontalLayout_7.addWidget(self.label_7)
        self.doubleSpinBox_7 = QtWidgets.QDoubleSpinBox(self.widget_11)
        self.doubleSpinBox_7.setObjectName("doubleSpinBox_7")
        self.horizontalLayout_7.addWidget(self.doubleSpinBox_7)
        self.verticalLayout.addWidget(self.widget_11)
        self.widget_12 = QtWidgets.QWidget(self.measure)
        self.widget_12.setObjectName("widget_12")
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout(self.widget_12)
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.label_8 = QtWidgets.QLabel(self.widget_12)
        self.label_8.setMinimumSize(QtCore.QSize(56, 16))
        self.label_8.setLineWidth(1)
        self.label_8.setAlignment(QtCore.Qt.AlignCenter)
        self.label_8.setObjectName("label_8")
        self.horizontalLayout_8.addWidget(self.label_8)
        self.doubleSpinBox_8 = QtWidgets.QDoubleSpinBox(self.widget_12)
        self.doubleSpinBox_8.setObjectName("doubleSpinBox_8")
        self.horizontalLayout_8.addWidget(self.doubleSpinBox_8)
        self.verticalLayout.addWidget(self.widget_12)
        self.widget_13 = QtWidgets.QWidget(self.measure)
        self.widget_13.setObjectName("widget_13")
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout(self.widget_13)
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.label_9 = QtWidgets.QLabel(self.widget_13)
        self.label_9.setMinimumSize(QtCore.QSize(56, 16))
        self.label_9.setLineWidth(1)
        self.label_9.setAlignment(QtCore.Qt.AlignCenter)
        self.label_9.setObjectName("label_9")
        self.horizontalLayout_9.addWidget(self.label_9)
        self.doubleSpinBox_9 = QtWidgets.QDoubleSpinBox(self.widget_13)
        self.doubleSpinBox_9.setObjectName("doubleSpinBox_9")
        self.horizontalLayout_9.addWidget(self.doubleSpinBox_9)
        self.verticalLayout.addWidget(self.widget_13)
        self.widget_20 = QtWidgets.QWidget(self.measure)
        self.widget_20.setObjectName("widget_20")
        self.horizontalLayout_16 = QtWidgets.QHBoxLayout(self.widget_20)
        self.horizontalLayout_16.setObjectName("horizontalLayout_16")
        self.label_16 = QtWidgets.QLabel(self.widget_20)
        self.label_16.setMinimumSize(QtCore.QSize(56, 16))
        self.label_16.setLineWidth(1)
        self.label_16.setAlignment(QtCore.Qt.AlignCenter)
        self.label_16.setObjectName("label_16")
        self.horizontalLayout_16.addWidget(self.label_16)
        self.doubleSpinBox_16 = QtWidgets.QDoubleSpinBox(self.widget_20)
        self.doubleSpinBox_16.setObjectName("doubleSpinBox_16")
        self.horizontalLayout_16.addWidget(self.doubleSpinBox_16)
        self.verticalLayout.addWidget(self.widget_20)
        self.widget_21 = QtWidgets.QWidget(self.measure)
        self.widget_21.setObjectName("widget_21")
        self.horizontalLayout_17 = QtWidgets.QHBoxLayout(self.widget_21)
        self.horizontalLayout_17.setObjectName("horizontalLayout_17")
        self.label_17 = QtWidgets.QLabel(self.widget_21)
        self.label_17.setMinimumSize(QtCore.QSize(56, 16))
        self.label_17.setLineWidth(1)
        self.label_17.setAlignment(QtCore.Qt.AlignCenter)
        self.label_17.setObjectName("label_17")
        self.horizontalLayout_17.addWidget(self.label_17)
        self.doubleSpinBox_17 = QtWidgets.QDoubleSpinBox(self.widget_21)
        self.doubleSpinBox_17.setObjectName("doubleSpinBox_17")
        self.horizontalLayout_17.addWidget(self.doubleSpinBox_17)
        self.verticalLayout.addWidget(self.widget_21)
        self.widget_14 = QtWidgets.QWidget(self.measure)
        self.widget_14.setObjectName("widget_14")
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout(self.widget_14)
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.label_10 = QtWidgets.QLabel(self.widget_14)
        self.label_10.setMinimumSize(QtCore.QSize(56, 16))
        self.label_10.setLineWidth(1)
        self.label_10.setAlignment(QtCore.Qt.AlignCenter)
        self.label_10.setObjectName("label_10")
        self.horizontalLayout_10.addWidget(self.label_10)
        self.doubleSpinBox_10 = QtWidgets.QDoubleSpinBox(self.widget_14)
        self.doubleSpinBox_10.setObjectName("doubleSpinBox_10")
        self.horizontalLayout_10.addWidget(self.doubleSpinBox_10)
        self.verticalLayout.addWidget(self.widget_14)
        self.widget_15 = QtWidgets.QWidget(self.measure)
        self.widget_15.setObjectName("widget_15")
        self.horizontalLayout_11 = QtWidgets.QHBoxLayout(self.widget_15)
        self.horizontalLayout_11.setObjectName("horizontalLayout_11")
        self.label_11 = QtWidgets.QLabel(self.widget_15)
        self.label_11.setMinimumSize(QtCore.QSize(56, 16))
        self.label_11.setLineWidth(1)
        self.label_11.setAlignment(QtCore.Qt.AlignCenter)
        self.label_11.setObjectName("label_11")
        self.horizontalLayout_11.addWidget(self.label_11)
        self.doubleSpinBox_11 = QtWidgets.QDoubleSpinBox(self.widget_15)
        self.doubleSpinBox_11.setObjectName("doubleSpinBox_11")
        self.horizontalLayout_11.addWidget(self.doubleSpinBox_11)
        self.verticalLayout.addWidget(self.widget_15)
        self.widget_16 = QtWidgets.QWidget(self.measure)
        self.widget_16.setObjectName("widget_16")
        self.horizontalLayout_12 = QtWidgets.QHBoxLayout(self.widget_16)
        self.horizontalLayout_12.setObjectName("horizontalLayout_12")
        self.label_12 = QtWidgets.QLabel(self.widget_16)
        self.label_12.setMinimumSize(QtCore.QSize(56, 16))
        self.label_12.setLineWidth(1)
        self.label_12.setAlignment(QtCore.Qt.AlignCenter)
        self.label_12.setObjectName("label_12")
        self.horizontalLayout_12.addWidget(self.label_12)
        self.doubleSpinBox_12 = QtWidgets.QDoubleSpinBox(self.widget_16)
        self.doubleSpinBox_12.setObjectName("doubleSpinBox_12")
        self.horizontalLayout_12.addWidget(self.doubleSpinBox_12)
        self.verticalLayout.addWidget(self.widget_16)
        self.widget_22 = QtWidgets.QWidget(self.measure)
        self.widget_22.setObjectName("widget_22")
        self.horizontalLayout_21 = QtWidgets.QHBoxLayout(self.widget_22)
        self.horizontalLayout_21.setObjectName("horizontalLayout_21")
        self.label_40 = QtWidgets.QLabel(self.widget_22)
        self.label_40.setMinimumSize(QtCore.QSize(56, 16))
        self.label_40.setLineWidth(1)
        self.label_40.setAlignment(QtCore.Qt.AlignCenter)
        self.label_40.setObjectName("label_40")
        self.horizontalLayout_21.addWidget(self.label_40)
        self.doubleSpinBox_21 = QtWidgets.QDoubleSpinBox(self.widget_22)
        self.doubleSpinBox_21.setObjectName("doubleSpinBox_21")
        self.horizontalLayout_21.addWidget(self.doubleSpinBox_21)
        self.verticalLayout.addWidget(self.widget_22)
        self.widget_23 = QtWidgets.QWidget(self.measure)
        self.widget_23.setObjectName("widget_23")
        self.horizontalLayout_24 = QtWidgets.QHBoxLayout(self.widget_23)
        self.horizontalLayout_24.setObjectName("horizontalLayout_24")
        self.label_43 = QtWidgets.QLabel(self.widget_23)
        self.label_43.setMinimumSize(QtCore.QSize(56, 16))
        self.label_43.setLineWidth(1)
        self.label_43.setAlignment(QtCore.Qt.AlignCenter)
        self.label_43.setObjectName("label_43")
        self.horizontalLayout_24.addWidget(self.label_43)
        self.doubleSpinBox_24 = QtWidgets.QDoubleSpinBox(self.widget_23)
        self.doubleSpinBox_24.setObjectName("doubleSpinBox_24")
        self.horizontalLayout_24.addWidget(self.doubleSpinBox_24)
        self.verticalLayout.addWidget(self.widget_23)
        self.widget_17 = QtWidgets.QWidget(self.measure)
        self.widget_17.setObjectName("widget_17")
        self.horizontalLayout_13 = QtWidgets.QHBoxLayout(self.widget_17)
        self.horizontalLayout_13.setObjectName("horizontalLayout_13")
        self.label_13 = QtWidgets.QLabel(self.widget_17)
        self.label_13.setMinimumSize(QtCore.QSize(56, 16))
        self.label_13.setLineWidth(1)
        self.label_13.setAlignment(QtCore.Qt.AlignCenter)
        self.label_13.setObjectName("label_13")
        self.horizontalLayout_13.addWidget(self.label_13)
        self.doubleSpinBox_13 = QtWidgets.QDoubleSpinBox(self.widget_17)
        self.doubleSpinBox_13.setObjectName("doubleSpinBox_13")
        self.horizontalLayout_13.addWidget(self.doubleSpinBox_13)
        self.verticalLayout.addWidget(self.widget_17)
        self.widget_18 = QtWidgets.QWidget(self.measure)
        self.widget_18.setObjectName("widget_18")
        self.horizontalLayout_14 = QtWidgets.QHBoxLayout(self.widget_18)
        self.horizontalLayout_14.setObjectName("horizontalLayout_14")
        self.label_14 = QtWidgets.QLabel(self.widget_18)
        self.label_14.setMinimumSize(QtCore.QSize(56, 16))
        self.label_14.setLineWidth(1)
        self.label_14.setAlignment(QtCore.Qt.AlignCenter)
        self.label_14.setObjectName("label_14")
        self.horizontalLayout_14.addWidget(self.label_14)
        self.doubleSpinBox_14 = QtWidgets.QDoubleSpinBox(self.widget_18)
        self.doubleSpinBox_14.setObjectName("doubleSpinBox_14")
        self.horizontalLayout_14.addWidget(self.doubleSpinBox_14)
        self.verticalLayout.addWidget(self.widget_18)
        self.widget_39 = QtWidgets.QWidget(self.measure)
        self.widget_39.setObjectName("widget_39")
        self.horizontalLayout_45 = QtWidgets.QHBoxLayout(self.widget_39)
        self.horizontalLayout_45.setObjectName("horizontalLayout_45")
        self.label_45 = QtWidgets.QLabel(self.widget_39)
        self.label_45.setMinimumSize(QtCore.QSize(56, 16))
        self.label_45.setLineWidth(1)
        self.label_45.setAlignment(QtCore.Qt.AlignCenter)
        self.label_45.setObjectName("label_45")
        self.horizontalLayout_45.addWidget(self.label_45)
        self.doubleSpinBox_26 = QtWidgets.QDoubleSpinBox(self.widget_39)
        self.doubleSpinBox_26.setObjectName("doubleSpinBox_26")
        self.horizontalLayout_45.addWidget(self.doubleSpinBox_26)
        self.verticalLayout.addWidget(self.widget_39)
        self.widget_19 = QtWidgets.QWidget(self.measure)
        self.widget_19.setObjectName("widget_19")
        self.horizontalLayout_15 = QtWidgets.QHBoxLayout(self.widget_19)
        self.horizontalLayout_15.setObjectName("horizontalLayout_15")
        self.label_15 = QtWidgets.QLabel(self.widget_19)
        self.label_15.setMinimumSize(QtCore.QSize(56, 16))
        self.label_15.setLineWidth(1)
        self.label_15.setAlignment(QtCore.Qt.AlignCenter)
        self.label_15.setObjectName("label_15")
        self.horizontalLayout_15.addWidget(self.label_15)
        self.doubleSpinBox_15 = QtWidgets.QDoubleSpinBox(self.widget_19)
        self.doubleSpinBox_15.setObjectName("doubleSpinBox_15")
        self.horizontalLayout_15.addWidget(self.doubleSpinBox_15)
        self.verticalLayout.addWidget(self.widget_19)
        self.widget_40 = QtWidgets.QWidget(self.measure)
        self.widget_40.setObjectName("widget_40")
        self.horizontalLayout_46 = QtWidgets.QHBoxLayout(self.widget_40)
        self.horizontalLayout_46.setObjectName("horizontalLayout_46")
        self.label_46 = QtWidgets.QLabel(self.widget_40)
        self.label_46.setMinimumSize(QtCore.QSize(56, 16))
        self.label_46.setLineWidth(1)
        self.label_46.setAlignment(QtCore.Qt.AlignCenter)
        self.label_46.setObjectName("label_46")
        self.horizontalLayout_46.addWidget(self.label_46)
        self.doubleSpinBox_27 = QtWidgets.QDoubleSpinBox(self.widget_40)
        self.doubleSpinBox_27.setObjectName("doubleSpinBox_27")
        self.horizontalLayout_46.addWidget(self.doubleSpinBox_27)
        self.verticalLayout.addWidget(self.widget_40)
        self.gridLayout_3.addWidget(self.measure, 0, 1, 4, 1)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_3.addItem(spacerItem, 0, 2, 1, 1)
        self.maya = QtWidgets.QWidget(self.centralwidget)
        self.maya.setMinimumSize(QtCore.QSize(300, 400))
        self.maya.setStyleSheet("border: 1px solid red")
        self.maya.setObjectName("maya")
        self.gridLayout_3.addWidget(self.maya, 0, 3, 3, 1)
        self.predict = QtWidgets.QWidget(self.centralwidget)
        self.predict.setMinimumSize(QtCore.QSize(250, 0))
        self.predict.setStyleSheet("border: 1px solid red")
        self.predict.setObjectName("predict")
        self.gridLayout = QtWidgets.QGridLayout(self.predict)
        self.gridLayout.setObjectName("gridLayout")
        self.widget = QtWidgets.QWidget(self.predict)
        self.widget.setObjectName("widget")
        self.horizontalLayout_25 = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout_25.setObjectName("horizontalLayout_25")
        self.label_20 = QtWidgets.QLabel(self.widget)
        self.label_20.setObjectName("label_20")
        self.horizontalLayout_25.addWidget(self.label_20)
        self.lineEdit = QtWidgets.QLineEdit(self.widget)
        self.lineEdit.setObjectName("lineEdit")
        self.horizontalLayout_25.addWidget(self.lineEdit)
        self.gridLayout.addWidget(self.widget, 0, 0, 1, 1)
        self.widget_2 = QtWidgets.QWidget(self.predict)
        self.widget_2.setObjectName("widget_2")
        self.horizontalLayout_26 = QtWidgets.QHBoxLayout(self.widget_2)
        self.horizontalLayout_26.setObjectName("horizontalLayout_26")
        self.label_21 = QtWidgets.QLabel(self.widget_2)
        self.label_21.setObjectName("label_21")
        self.horizontalLayout_26.addWidget(self.label_21)
        self.lineEdit_2 = QtWidgets.QLineEdit(self.widget_2)
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.horizontalLayout_26.addWidget(self.lineEdit_2)
        self.gridLayout.addWidget(self.widget_2, 1, 0, 1, 1)
        self.widget_3 = QtWidgets.QWidget(self.predict)
        self.widget_3.setObjectName("widget_3")
        self.horizontalLayout_27 = QtWidgets.QHBoxLayout(self.widget_3)
        self.horizontalLayout_27.setObjectName("horizontalLayout_27")
        self.label_22 = QtWidgets.QLabel(self.widget_3)
        self.label_22.setObjectName("label_22")
        self.horizontalLayout_27.addWidget(self.label_22)
        self.lineEdit_3 = QtWidgets.QLineEdit(self.widget_3)
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.horizontalLayout_27.addWidget(self.lineEdit_3)
        self.gridLayout.addWidget(self.widget_3, 2, 0, 1, 1)
        self.widget_4 = QtWidgets.QWidget(self.predict)
        self.widget_4.setObjectName("widget_4")
        self.horizontalLayout_28 = QtWidgets.QHBoxLayout(self.widget_4)
        self.horizontalLayout_28.setObjectName("horizontalLayout_28")
        self.label_23 = QtWidgets.QLabel(self.widget_4)
        self.label_23.setObjectName("label_23")
        self.horizontalLayout_28.addWidget(self.label_23)
        self.lineEdit_4 = QtWidgets.QLineEdit(self.widget_4)
        self.lineEdit_4.setObjectName("lineEdit_4")
        self.horizontalLayout_28.addWidget(self.lineEdit_4)
        self.gridLayout.addWidget(self.widget_4, 3, 0, 1, 1)
        self.widget_24 = QtWidgets.QWidget(self.predict)
        self.widget_24.setObjectName("widget_24")
        self.horizontalLayout_29 = QtWidgets.QHBoxLayout(self.widget_24)
        self.horizontalLayout_29.setObjectName("horizontalLayout_29")
        self.label_24 = QtWidgets.QLabel(self.widget_24)
        self.label_24.setObjectName("label_24")
        self.horizontalLayout_29.addWidget(self.label_24)
        self.lineEdit_5 = QtWidgets.QLineEdit(self.widget_24)
        self.lineEdit_5.setObjectName("lineEdit_5")
        self.horizontalLayout_29.addWidget(self.lineEdit_5)
        self.gridLayout.addWidget(self.widget_24, 4, 0, 1, 1)
        self.widget_25 = QtWidgets.QWidget(self.predict)
        self.widget_25.setObjectName("widget_25")
        self.horizontalLayout_30 = QtWidgets.QHBoxLayout(self.widget_25)
        self.horizontalLayout_30.setObjectName("horizontalLayout_30")
        self.label_25 = QtWidgets.QLabel(self.widget_25)
        self.label_25.setObjectName("label_25")
        self.horizontalLayout_30.addWidget(self.label_25)
        self.lineEdit_6 = QtWidgets.QLineEdit(self.widget_25)
        self.lineEdit_6.setObjectName("lineEdit_6")
        self.horizontalLayout_30.addWidget(self.lineEdit_6)
        self.gridLayout.addWidget(self.widget_25, 5, 0, 1, 1)
        self.widget_26 = QtWidgets.QWidget(self.predict)
        self.widget_26.setObjectName("widget_26")
        self.horizontalLayout_31 = QtWidgets.QHBoxLayout(self.widget_26)
        self.horizontalLayout_31.setObjectName("horizontalLayout_31")
        self.label_26 = QtWidgets.QLabel(self.widget_26)
        self.label_26.setObjectName("label_26")
        self.horizontalLayout_31.addWidget(self.label_26)
        self.lineEdit_7 = QtWidgets.QLineEdit(self.widget_26)
        self.lineEdit_7.setObjectName("lineEdit_7")
        self.horizontalLayout_31.addWidget(self.lineEdit_7)
        self.gridLayout.addWidget(self.widget_26, 6, 0, 1, 1)
        self.widget_27 = QtWidgets.QWidget(self.predict)
        self.widget_27.setObjectName("widget_27")
        self.horizontalLayout_32 = QtWidgets.QHBoxLayout(self.widget_27)
        self.horizontalLayout_32.setObjectName("horizontalLayout_32")
        self.label_27 = QtWidgets.QLabel(self.widget_27)
        self.label_27.setObjectName("label_27")
        self.horizontalLayout_32.addWidget(self.label_27)
        self.lineEdit_8 = QtWidgets.QLineEdit(self.widget_27)
        self.lineEdit_8.setObjectName("lineEdit_8")
        self.horizontalLayout_32.addWidget(self.lineEdit_8)
        self.gridLayout.addWidget(self.widget_27, 7, 0, 1, 1)
        self.widget_28 = QtWidgets.QWidget(self.predict)
        self.widget_28.setObjectName("widget_28")
        self.horizontalLayout_33 = QtWidgets.QHBoxLayout(self.widget_28)
        self.horizontalLayout_33.setObjectName("horizontalLayout_33")
        self.label_28 = QtWidgets.QLabel(self.widget_28)
        self.label_28.setObjectName("label_28")
        self.horizontalLayout_33.addWidget(self.label_28)
        self.lineEdit_9 = QtWidgets.QLineEdit(self.widget_28)
        self.lineEdit_9.setObjectName("lineEdit_9")
        self.horizontalLayout_33.addWidget(self.lineEdit_9)
        self.gridLayout.addWidget(self.widget_28, 8, 0, 1, 1)
        self.widget_29 = QtWidgets.QWidget(self.predict)
        self.widget_29.setObjectName("widget_29")
        self.horizontalLayout_34 = QtWidgets.QHBoxLayout(self.widget_29)
        self.horizontalLayout_34.setObjectName("horizontalLayout_34")
        self.label_29 = QtWidgets.QLabel(self.widget_29)
        self.label_29.setObjectName("label_29")
        self.horizontalLayout_34.addWidget(self.label_29)
        self.lineEdit_10 = QtWidgets.QLineEdit(self.widget_29)
        self.lineEdit_10.setObjectName("lineEdit_10")
        self.horizontalLayout_34.addWidget(self.lineEdit_10)
        self.gridLayout.addWidget(self.widget_29, 9, 0, 1, 1)
        self.widget_30 = QtWidgets.QWidget(self.predict)
        self.widget_30.setObjectName("widget_30")
        self.horizontalLayout_35 = QtWidgets.QHBoxLayout(self.widget_30)
        self.horizontalLayout_35.setObjectName("horizontalLayout_35")
        self.label_30 = QtWidgets.QLabel(self.widget_30)
        self.label_30.setObjectName("label_30")
        self.horizontalLayout_35.addWidget(self.label_30)
        self.lineEdit_11 = QtWidgets.QLineEdit(self.widget_30)
        self.lineEdit_11.setObjectName("lineEdit_11")
        self.horizontalLayout_35.addWidget(self.lineEdit_11)
        self.gridLayout.addWidget(self.widget_30, 10, 0, 1, 1)
        self.widget_31 = QtWidgets.QWidget(self.predict)
        self.widget_31.setObjectName("widget_31")
        self.horizontalLayout_36 = QtWidgets.QHBoxLayout(self.widget_31)
        self.horizontalLayout_36.setObjectName("horizontalLayout_36")
        self.label_31 = QtWidgets.QLabel(self.widget_31)
        self.label_31.setObjectName("label_31")
        self.horizontalLayout_36.addWidget(self.label_31)
        self.lineEdit_12 = QtWidgets.QLineEdit(self.widget_31)
        self.lineEdit_12.setObjectName("lineEdit_12")
        self.horizontalLayout_36.addWidget(self.lineEdit_12)
        self.gridLayout.addWidget(self.widget_31, 11, 0, 1, 1)
        self.widget_32 = QtWidgets.QWidget(self.predict)
        self.widget_32.setObjectName("widget_32")
        self.horizontalLayout_37 = QtWidgets.QHBoxLayout(self.widget_32)
        self.horizontalLayout_37.setObjectName("horizontalLayout_37")
        self.label_32 = QtWidgets.QLabel(self.widget_32)
        self.label_32.setObjectName("label_32")
        self.horizontalLayout_37.addWidget(self.label_32)
        self.lineEdit_13 = QtWidgets.QLineEdit(self.widget_32)
        self.lineEdit_13.setObjectName("lineEdit_13")
        self.horizontalLayout_37.addWidget(self.lineEdit_13)
        self.gridLayout.addWidget(self.widget_32, 12, 0, 1, 1)
        self.widget_33 = QtWidgets.QWidget(self.predict)
        self.widget_33.setObjectName("widget_33")
        self.horizontalLayout_38 = QtWidgets.QHBoxLayout(self.widget_33)
        self.horizontalLayout_38.setObjectName("horizontalLayout_38")
        self.label_33 = QtWidgets.QLabel(self.widget_33)
        self.label_33.setObjectName("label_33")
        self.horizontalLayout_38.addWidget(self.label_33)
        self.lineEdit_14 = QtWidgets.QLineEdit(self.widget_33)
        self.lineEdit_14.setObjectName("lineEdit_14")
        self.horizontalLayout_38.addWidget(self.lineEdit_14)
        self.gridLayout.addWidget(self.widget_33, 13, 0, 1, 1)
        self.widget_34 = QtWidgets.QWidget(self.predict)
        self.widget_34.setObjectName("widget_34")
        self.horizontalLayout_39 = QtWidgets.QHBoxLayout(self.widget_34)
        self.horizontalLayout_39.setObjectName("horizontalLayout_39")
        self.label_34 = QtWidgets.QLabel(self.widget_34)
        self.label_34.setObjectName("label_34")
        self.horizontalLayout_39.addWidget(self.label_34)
        self.lineEdit_15 = QtWidgets.QLineEdit(self.widget_34)
        self.lineEdit_15.setObjectName("lineEdit_15")
        self.horizontalLayout_39.addWidget(self.lineEdit_15)
        self.gridLayout.addWidget(self.widget_34, 14, 0, 1, 1)
        self.widget_35 = QtWidgets.QWidget(self.predict)
        self.widget_35.setObjectName("widget_35")
        self.horizontalLayout_40 = QtWidgets.QHBoxLayout(self.widget_35)
        self.horizontalLayout_40.setObjectName("horizontalLayout_40")
        self.label_35 = QtWidgets.QLabel(self.widget_35)
        self.label_35.setObjectName("label_35")
        self.horizontalLayout_40.addWidget(self.label_35)
        self.lineEdit_16 = QtWidgets.QLineEdit(self.widget_35)
        self.lineEdit_16.setObjectName("lineEdit_16")
        self.horizontalLayout_40.addWidget(self.lineEdit_16)
        self.gridLayout.addWidget(self.widget_35, 15, 0, 1, 1)
        self.widget_36 = QtWidgets.QWidget(self.predict)
        self.widget_36.setObjectName("widget_36")
        self.horizontalLayout_41 = QtWidgets.QHBoxLayout(self.widget_36)
        self.horizontalLayout_41.setObjectName("horizontalLayout_41")
        self.label_36 = QtWidgets.QLabel(self.widget_36)
        self.label_36.setObjectName("label_36")
        self.horizontalLayout_41.addWidget(self.label_36)
        self.lineEdit_17 = QtWidgets.QLineEdit(self.widget_36)
        self.lineEdit_17.setObjectName("lineEdit_17")
        self.horizontalLayout_41.addWidget(self.lineEdit_17)
        self.gridLayout.addWidget(self.widget_36, 16, 0, 1, 1)
        self.widget_37 = QtWidgets.QWidget(self.predict)
        self.widget_37.setObjectName("widget_37")
        self.horizontalLayout_42 = QtWidgets.QHBoxLayout(self.widget_37)
        self.horizontalLayout_42.setObjectName("horizontalLayout_42")
        self.label_37 = QtWidgets.QLabel(self.widget_37)
        self.label_37.setObjectName("label_37")
        self.horizontalLayout_42.addWidget(self.label_37)
        self.lineEdit_18 = QtWidgets.QLineEdit(self.widget_37)
        self.lineEdit_18.setObjectName("lineEdit_18")
        self.horizontalLayout_42.addWidget(self.lineEdit_18)
        self.gridLayout.addWidget(self.widget_37, 17, 0, 1, 1)
        self.widget_38 = QtWidgets.QWidget(self.predict)
        self.widget_38.setObjectName("widget_38")
        self.horizontalLayout_43 = QtWidgets.QHBoxLayout(self.widget_38)
        self.horizontalLayout_43.setObjectName("horizontalLayout_43")
        self.label_38 = QtWidgets.QLabel(self.widget_38)
        self.label_38.setObjectName("label_38")
        self.horizontalLayout_43.addWidget(self.label_38)
        self.lineEdit_19 = QtWidgets.QLineEdit(self.widget_38)
        self.lineEdit_19.setObjectName("lineEdit_19")
        self.horizontalLayout_43.addWidget(self.lineEdit_19)
        self.gridLayout.addWidget(self.widget_38, 18, 0, 1, 1)
        self.gridLayout_3.addWidget(self.predict, 0, 5, 4, 1)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_3.addItem(spacerItem1, 1, 0, 1, 1)
        spacerItem2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_3.addItem(spacerItem2, 2, 6, 1, 1)
        self.gender = QtWidgets.QWidget(self.centralwidget)
        self.gender.setObjectName("gender")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.gender)
        self.gridLayout_2.setObjectName("gridLayout_2")
        spacerItem3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_2.addItem(spacerItem3, 0, 0, 1, 1)
        spacerItem4 = QtWidgets.QSpacerItem(43, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_2.addItem(spacerItem4, 0, 5, 1, 1)
        self.radioButton = QtWidgets.QRadioButton(self.gender)
        self.radioButton.setObjectName("radioButton")
        self.gridLayout_2.addWidget(self.radioButton, 0, 1, 1, 1)
        self.radioButton_2 = QtWidgets.QRadioButton(self.gender)
        self.radioButton_2.setObjectName("radioButton_2")
        self.gridLayout_2.addWidget(self.radioButton_2, 0, 4, 1, 1)
        spacerItem5 = QtWidgets.QSpacerItem(95, 20, QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_2.addItem(spacerItem5, 0, 3, 1, 1)
        self.pushButton = QtWidgets.QPushButton(self.gender)
        self.pushButton.setObjectName("pushButton")
        self.gridLayout_2.addWidget(self.pushButton, 1, 1, 1, 1)
        self.pushButton_2 = QtWidgets.QPushButton(self.gender)
        self.pushButton_2.setObjectName("pushButton_2")
        self.gridLayout_2.addWidget(self.pushButton_2, 1, 4, 1, 1)
        self.gridLayout_3.addWidget(self.gender, 3, 3, 1, 1)
        spacerItem6 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_3.addItem(spacerItem6, 3, 4, 1, 1)
        self.maya.raise_()
        self.predict.raise_()
        self.measure.raise_()
        self.gender.raise_()
        HumanEstimate.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(HumanEstimate)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1014, 23))
        self.menubar.setObjectName("menubar")
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        self.menuEdit = QtWidgets.QMenu(self.menubar)
        self.menuEdit.setObjectName("menuEdit")
        HumanEstimate.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(HumanEstimate)
        self.statusbar.setObjectName("statusbar")
        HumanEstimate.setStatusBar(self.statusbar)
        self.actionOpen = QtWidgets.QAction(HumanEstimate)
        self.actionOpen.setObjectName("actionOpen")
        self.actionClose = QtWidgets.QAction(HumanEstimate)
        self.actionClose.setObjectName("actionClose")
        self.actionSave = QtWidgets.QAction(HumanEstimate)
        self.actionSave.setObjectName("actionSave")
        self.menuFile.addAction(self.actionOpen)
        self.menuFile.addAction(self.actionSave)
        self.menuFile.addAction(self.actionClose)
        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuEdit.menuAction())

        self.retranslateUi(HumanEstimate)
        QtCore.QMetaObject.connectSlotsByName(HumanEstimate)

    def retranslateUi(self, HumanEstimate):
        _translate = QtCore.QCoreApplication.translate
        HumanEstimate.setWindowTitle(_translate("HumanEstimate", "HumanEstimate"))
        self.label.setText(_translate("HumanEstimate", "Measure01"))
        self.label_2.setText(_translate("HumanEstimate", "Measure01"))
        self.label_5.setText(_translate("HumanEstimate", "Measure01"))
        self.label_6.setText(_translate("HumanEstimate", "Measure01"))
        self.label_7.setText(_translate("HumanEstimate", "Measure01"))
        self.label_8.setText(_translate("HumanEstimate", "Measure01"))
        self.label_9.setText(_translate("HumanEstimate", "Measure01"))
        self.label_16.setText(_translate("HumanEstimate", "Measure01"))
        self.label_17.setText(_translate("HumanEstimate", "Measure01"))
        self.label_10.setText(_translate("HumanEstimate", "Measure01"))
        self.label_11.setText(_translate("HumanEstimate", "Measure01"))
        self.label_12.setText(_translate("HumanEstimate", "Measure01"))
        self.label_40.setText(_translate("HumanEstimate", "Measure01"))
        self.label_43.setText(_translate("HumanEstimate", "Measure01"))
        self.label_13.setText(_translate("HumanEstimate", "Measure01"))
        self.label_14.setText(_translate("HumanEstimate", "Measure01"))
        self.label_45.setText(_translate("HumanEstimate", "Measure01"))
        self.label_15.setText(_translate("HumanEstimate", "Measure01"))
        self.label_46.setText(_translate("HumanEstimate", "Measure01"))
        self.label_20.setText(_translate("HumanEstimate", "Measure01"))
        self.label_21.setText(_translate("HumanEstimate", "Measure01"))
        self.label_22.setText(_translate("HumanEstimate", "Measure01"))
        self.label_23.setText(_translate("HumanEstimate", "Measure01"))
        self.label_24.setText(_translate("HumanEstimate", "Measure01"))
        self.label_25.setText(_translate("HumanEstimate", "Measure01"))
        self.label_26.setText(_translate("HumanEstimate", "Measure01"))
        self.label_27.setText(_translate("HumanEstimate", "Measure01"))
        self.label_28.setText(_translate("HumanEstimate", "Measure01"))
        self.label_29.setText(_translate("HumanEstimate", "Measure01"))
        self.label_30.setText(_translate("HumanEstimate", "Measure01"))
        self.label_31.setText(_translate("HumanEstimate", "Measure01"))
        self.label_32.setText(_translate("HumanEstimate", "Measure01"))
        self.label_33.setText(_translate("HumanEstimate", "Measure01"))
        self.label_34.setText(_translate("HumanEstimate", "Measure01"))
        self.label_35.setText(_translate("HumanEstimate", "Measure01"))
        self.label_36.setText(_translate("HumanEstimate", "Measure01"))
        self.label_37.setText(_translate("HumanEstimate", "Measure01"))
        self.label_38.setText(_translate("HumanEstimate", "Measure01"))
        self.radioButton.setText(_translate("HumanEstimate", "Male"))
        self.radioButton_2.setText(_translate("HumanEstimate", "Female"))
        self.pushButton.setText(_translate("HumanEstimate", "Reset"))
        self.pushButton_2.setText(_translate("HumanEstimate", "Predict"))
        self.menuFile.setTitle(_translate("HumanEstimate", "File"))
        self.menuEdit.setTitle(_translate("HumanEstimate", "Edit"))
        self.actionOpen.setText(_translate("HumanEstimate", "Open"))
        self.actionClose.setText(_translate("HumanEstimate", "Close"))
        self.actionSave.setText(_translate("HumanEstimate", "Save"))
