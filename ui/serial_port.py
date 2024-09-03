# 逻辑文件

import re
import sys
import binascii
import time
from PyQt5.QtCore import QTimer, QUrl
from PyQt5.QtWidgets import *
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtWebEngineWidgets import *
from motor_window_designed import Ui_MotorWindow
from training_window_designed import Ui_TrainingWindow
from PyQt5.QtCore import QDate


class MyMotorWindow(QMainWindow, Ui_MotorWindow):
    def __init__(self, parent=None):
        super(MyMotorWindow, self).__init__(parent)
        self.setupUi(self)
        # 设置实例
        self.CreateItems()
        # 设置信号与槽
        self.CreateSignalSlot()

    # 设置实例
    def CreateItems(self):
        # Qt 串口类
        self.com = QSerialPort()
        self.initial = self.InitialCom()

    def InitialCom(self):
        self.serialport.clear()
        com = QSerialPort()
        com_list = QSerialPortInfo.availablePorts()
        for info in com_list:
            com.setPort(info)
            if com.open(QSerialPort.ReadWrite):
                self.serialport.addItem(info.portName())
                com.close()

    def CreateSignalSlot(self):
        self.StartButton.clicked.connect(self.StartButton_clicked)
        self.StopButton.clicked.connect(self.StopButton_clicked)
        self.RefreshButton.clicked.connect(self.RefreshButton_clicked)
        self.SendButton.clicked.connect(self.SendButton_clicked)
        self.ShowtrainingwindowButton.clicked.connect(self.ShowtrainingwindowButton_clicked)
        self.com.readyRead.connect(self.Com_Receive_Data)  # 接收数据
        self.Hex.stateChanged.connect(self.HexClicked)


    def Com_Send_Data(self):
        txData = self.textEdit_Send.toPlainText()
        if len(txData) == 0 :
            return
        if self.hexSending_checkBox.isChecked() == False:
            self.com.write(txData.encode('UTF-8'))
        else:
            Data = txData.replace(' ', '')
            # 如果16进制不是偶数个字符, 去掉最后一个, [ ]左闭右开
            if len(Data)%2 == 1:
                Data = Data[0:len(Data)-1]
            # 如果遇到非16进制字符
            if Data.isalnum() is False:
                QMessageBox.critical(self, '错误', '包含非十六进制数')
            try:
                hexData = binascii.a2b_hex(Data)
            except:
                QMessageBox.critical(self, '错误', '转换编码错误')
                return
            # 发送16进制数据, 发送格式如 ‘31 32 33 41 42 43’, 代表'123ABC'
            try:
                self.com.write(hexData)
            except:
                QMessageBox.critical(self, '异常', '十六进制发送错误')
                return

    def Com_Receive_Data(self):
        try:
            rxData = bytes(self.com.readAll())
        except:
            QMessageBox.critical(self, '严重错误', '串口接收数据错误')
        if self.Hex.isChecked() == False:
            try:
                self.Showcontent.insertPlainText(rxData.decode('UTF-8'))
            except:
                pass
        else:
            Data = binascii.b2a_hex(rxData).decode('ascii')
            # re 正则表达式 (.{2}) 匹配两个字母
            hexStr = ' 0x'.join(re.findall('(.{2})', Data))
            # 补齐第一个 0x
            hexStr = '0x' + hexStr
            self.Showcontent.insertPlainText(hexStr)
            self.Showcontent.insertPlainText(' ')

    def HexClicked(self):
        if self.Hex.isChecked() == True:
            self.Showcontent.insertPlainText('\n')


    def SendButton_clicked(self):
        self.Com_Send_Data()

    def StartButton_clicked(self):
        comName = self.serialport.currentText()
        comBaud = int(self.baud.currentText())
        self.com.setPortName(comName)
        try:
            if self.com.open(QSerialPort.ReadWrite) == False:
                QMessageBox.critical(self, '严重错误', '串口打开失败')
                return
        except:
            QMessageBox.critical(self, '严重错误', '串口打开失败')
            return

        self.DoneFlag.setText('已开启')
        self.com.setBaudRate(comBaud)

    def StopButton_clicked(self):
        self.com.close()
        self.DoneFlag.setText('已关闭')

    def RefreshButton_clicked(self):
        self.serialport.clear()
        com = QSerialPort()
        com_list = QSerialPortInfo.availablePorts()
        for info in com_list:
            com.setPort(info)
            if com.open(QSerialPort.ReadWrite):
                self.serialport.addItem(info.portName())
                com.close()

    def ShowtrainingwindowButton_clicked(self):
        self.TrainingWindow = QMainWindow()
        self.ui2 = Ui_TrainingWindow()
        self.ui2.setupUi(self.TrainingWindow)
        self.TrainingWindow.show()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    myWin = MyMotorWindow()
    myWin.show()
    sys.exit(app.exec_())

