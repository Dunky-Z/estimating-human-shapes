import sys

from ui import Ui_HumanEstimate
from maya_widget import MayaviQWidget
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication,QMainWindow

class Demo(QMainWindow,Ui_HumanEstimate):
    def __init__(self):
        super(Demo,self).__init__()
        self.setUpMaya(self)
        self.setDoubleSpinBox()
        self.setHorizontalSlider()

    def setUpMaya(self,parent = None):
        super(Demo, self).setupUi(self)
        self.maya_container = QtWidgets.QWidget(self.maya)
        self.maya_container.setWindowTitle("Embedding Mayavi in a PyQt5 Application")
        layout  = QtWidgets.QGridLayout(self.maya_container)
        self.viewer3D = MayaviQWidget(self.maya_container)
        layout.addWidget(self.viewer3D, 1, 1)
        print("setUp running")

    def setDoubleSpinBox(self):
        self.doubleSpinBox01.setSingleStep(0.05)
        self.doubleSpinBox01.valueChanged.connect(self.horizontalSlider01.setValue)
    def setHorizontalSlider(self):
        self.horizontalSlider01.setSingleStep(0.05)
        self.horizontalSlider01.valueChanged.connect(self.doubleSpinBox01.setValue)

class CamShow(QMainWindow,Ui_HumanEstimate):
    def __init__(self,parent = None):
        super(CamShow,self).__init__(parent)
        self.setupUi(self)

if __name__ == '__main__':
 app = QApplication(sys.argv)
 maya = Demo()
 # maya.setUp()
 ui=CamShow()
 maya.show()
 sys.exit(app.exec_())
