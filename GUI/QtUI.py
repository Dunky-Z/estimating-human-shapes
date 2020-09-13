import sys

from test import Ui_HumanEstimate
from maya_widget import MayaviQWidget
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication,QMainWindow

class SetMaYa(QMainWindow,Ui_HumanEstimate):
    def setUp(self,parent = None):
        super(SetMaYa, self).setupUi(self)
        self.maya_container = QtWidgets.QWidget(self.maya)
        self.maya_container.setWindowTitle("Embedding Mayavi in a PyQt5 Application")
        layout  = QtWidgets.QGridLayout(self.maya_container)
        self.viewer3D = MayaviQWidget(self.maya_container)
        layout.addWidget(self.viewer3D, 1, 1)
        # self.maya_container.show()

        print("setUp running")

class CamShow(QMainWindow,Ui_HumanEstimate):
    def __init__(self,parent = None):
        super(CamShow,self).__init__(parent)
        self.setupUi(self)

if __name__ == '__main__':
 app = QApplication(sys.argv)
 maya = SetMaYa()
 maya.setUp()
 ui=CamShow()
 maya.show()
 sys.exit(app.exec_())
