# 逻辑文件

import re
import sys
import binascii
import time
from PyQt5.QtCore import QTimer, QUrl
from PyQt5.QtWidgets import *
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtWebEngineWidgets import *
from test_button_designed import Ui_Form
from test_label_designed import Ui_LabelForm
from PyQt5.QtCore import QDate

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

class MyMainWindow(QMainWindow, Ui_Form):
    def __init__(self, parent=None):
        super(MyMainWindow, self).__init__(parent)
        self.setupUi(self)
        # 设置实例
        # self.CreateItems()
        # 设置信号与槽
        self.CreateSignalSlot()
        # 设置绘图
        self.app = pg.PlotWidget()
        self.app.showGrid(x=True, y=True, alpha=0.5)  # 设置图形网格的形式，我们设置显示横线和竖线，并且透明度惟0.5：
        self.plot_data = self.app.plot([0, 1, 2, 3, 4], [1, 5, 2, 4, 3])
        self.verticalLayout.addWidget(self.app)  # 添加绘图部件到网格布局层


    # 设置信号与槽
    def CreateSignalSlot(self):
        self.pushButton.clicked.connect(self.Com_Open_Button_clicked)

    def Com_Open_Button_clicked(self):
        self.form2 = QWidget()
        self.ui2 = MyLabelWindow()
        self.ui2.setupUi(self.form2)
        self.form2.show()


# class MyLabelWindow(QMainWindow, Ui_LabelForm):
#     def __init__(self, parent=None):
#         super(MyLabelWindow, self).__init__(parent)
#         self.setupUi(self)
#         # 设置实例
#         # self.CreateItems()
#         # 设置信号与槽
#         # self.CreateSignalSlot()
#         # 设置绘图
#         self.plot_widget  = pg.PlotWidget()
#         self.plot_widget.showGrid(x=True, y=True, alpha=0.5)  # 设置图形网格的形式，我们设置显示横线和竖线，并且透明度惟0.5：
#         # Enable antialiasing for prettier plots
#         self.get_vertical_layout().addWidget(self.plot_widget)  # 添加绘图部件到网格布局层
#         self.plot_data = self.plot_widget.plot([0, 1, 2, 3, 4], [1, 5, 2, 4, 3])
#         self.graphicsView.show()
#         print("I am here")

class MyLabelWindow(QWidget, Ui_LabelForm):
    def __init__(self, parent=None):
        super(MyLabelWindow, self).__init__(parent)

        self.setWindowTitle("Label Form")
        self.setGeometry(100, 100, 800, 600)

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.showGrid(x=True, y=True, alpha=0.5)
        self.plot_data = self.plot_widget.plot([0, 1, 2, 3, 4], [1, 5, 2, 4, 3])
        self.layout.addWidget(self.plot_widget)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    myWin = MyMainWindow()
    myWin.show()
    sys.exit(app.exec_())

