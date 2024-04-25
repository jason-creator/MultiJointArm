# 逻辑文件
import os
import re
import sys
import binascii
import time
from datetime import datetime
from PyQt5 import QtGui
from PyQt5.QtCore import QTimer, QDate, QThread, QTime, pyqtSignal, Qt
from PyQt5.QtWidgets import *
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtWidgets import QApplication, QMainWindow, QSlider
# from PyQt5.QtWebEngineWidgets import *
from motor_window_widget_designed import Ui_MotorWindowWidget
from training_window_designed import Ui_TrainingWindow
# 实时绘图
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
from collections import deque
# 图像
from PyQt5.QtGui import QPixmap




# 训练模式窗口
class MyTrainingWindow(QMainWindow, Ui_TrainingWindow):
    def __init__(self, parent=None):
        super(MyTrainingWindow, self).__init__(parent)
        self.setupUi(self)
        # 设置实例
        self.CreateItems()
        # 设置信号与槽
        self.CreateSignalSlot()
        # 绘图部件
        self.DataPlot()

    # 设置实例
    def CreateItems(self):
        # 实例化串口窗口
        self.MotorWindow = MyMotorWindow()
        # 图表实例
        self.torque_plot_widget = pg.PlotWidget()
        self.velocity_plot_widget = pg.PlotWidget()
        # 图像实例
        self.image = QLabel(self)
        pixmap = QPixmap("img/introduce.png")
        self.image.setPixmap(pixmap)
        self.image.setScaledContents(True)  # 如果需要缩放图片
        self.imageshow_layout.addWidget(self.image)  # 将图片添加到布局
        # 创建Qtimer对象,用于训练时间的显示及实时绘图
        self.timer = QTimer(self)
        self.TrainingWindow_StartTrainingFlag = False  # 初始化开始训练按钮为False
        self.elapsed_seconds = 0  # 初始化已经过的秒数为0
        self.ptr = 0.1  # 和绘图时的显示窗口对应
        # 初始化极限速度值
        self.limitiedSpeedValue.setValue(30)
        # 初始化运动角度初值
        self.Angle_Initial = 0
        self.downlimitation = 0
        self.uplimitation = 0
        # 初始化运动范围
        self.MotionRange = 120
        # 初始化重复次数和组数
        self.CountTimes = 0
        self.CountTimesTotal = 2
        self.CountGroup = 0
        self.CountuGroupTotal = 1
        self.Countflag = 0

        # 初始化进度条
        self.progressBar.setMinimum(0)
        self.progressBar.setMaximum(100)

        # 初始化训练结束信息窗
        self.message_box = QMessageBox()
        self.message_shown = False




    def CreateSignalSlot(self):
        self.startbutton.clicked.connect(self.startbutton_clicked)
        self.MotorWindow.start_training_flag.connect(self.starttrainingbutton_click)
        # 设置极限速度
        self.limitedSpeedSendButton.clicked.connect(self.limitedSpeedSendButton_clicked)
        # 设置时钟
        self.timer.timeout.connect(self.CalculateTrainingTime)
        # 设置极限位置
        self.ERSIncButton.clicked.connect(self.ERSIncButton_clicked)
        self.ERSDecButton.clicked.connect(self.ERSDecButton_clicked)
        self.IRSIncButton.clicked.connect(self.IRSIncButton_clicked)
        self.IRSDecButton.clicked.connect(self.IRSDecButton_clicked)


    def DataPlot(self):
        # 力矩窗口数据初始化
        self.data_torque = deque(maxlen=200)
        # self.full_data_torque = []
        self.data_torque.append((0, 0))
        # 速度窗口数据初始化
        self.data_velocity = deque(maxlen=200)
        # self.full_data_velocity = []
        self.data_velocity.append((0, 0))

        # 图1 力矩-时间曲线图
        self.torque_plot_widget_legend = self.torque_plot_widget.addLegend()
        self.torque_plot_widget.setBackground('w')
        self.torque_plot_widget.showGrid(x=True, y=True, alpha=0.5)
        self.torque_plot_widget.setLabel(axis='left', text=u'力矩值(Nm)')
        self.torque_plot_widget.setLabel(axis='bottom', text=u'时间(s)')
        self.torque_curve_1 = self.torque_plot_widget.plot(pen='k', name='肘关节屈伸电机', symbol='o', symbolSize=4, symbolPen=(0, 0, 0),
                                    symbolBrush=(0, 0, 0))
        # self.torque_curve_2 = self.torque_plot_widget.plot(pen='r', name='肘关节内外旋电机', symbol='t', symbolSize=4, symbolPen=(255, 0, 0),
        #                             symbolBrush=(255, 0, 0))
        # self.torque_curve_3 = self.torque_plot_widget.plot(pen='b', name='腕关节电机', symbol='s', symbolSize=4, symbolPen=(0, 0, 255),
        #                             symbolBrush=(0, 0, 255))
        # 图2 速度-时间曲线图
        self.velocity_plot_widget_legend = self.velocity_plot_widget.addLegend()
        self.velocity_plot_widget.setBackground('w')
        self.velocity_plot_widget.showGrid(x=True, y=True, alpha=0.5)
        self.velocity_plot_widget.setLabel(axis='left', text=u'速度值(°/s)')
        self.velocity_plot_widget.setLabel(axis='bottom', text=u'时间(s)')
        self.velocity_curve_1 = self.velocity_plot_widget.plot(pen='k', name='肘关节屈伸电机', symbol='o', symbolSize=4, symbolPen=(0, 0, 0),
                                    symbolBrush=(0, 0, 0))
        # self.velocity_curve_2 = self.velocity_plot_widget.plot([0, 1, 2, 3, 4], [0, 3, 8, 2, 1], pen='r', name='肘关节内外旋电机', symbol='t', symbolSize=4, symbolPen=(255, 0, 0),
        #                             symbolBrush=(255, 0, 0))
        # self.velocity_curve_3 = self.velocity_plot_widget.plot([0, 1, 2, 3, 4], [-5, 0, -5, 2, -2], pen='b', name='腕关节电机', symbol='s', symbolSize=4, symbolPen=(0, 0, 255),
        #                             symbolBrush=(0, 0, 255))


        # 添加绘图部件到网格布局层
        self.torquetime_verticalLayout_plot.addWidget(self.torque_plot_widget)
        self.velocitytime_verticalLayout_plot.addWidget(self.velocity_plot_widget)



    def startbutton_clicked(self):
        # 接收关节数据
        self.MotorWindow.joint_data_received.connect(self.JointInfo_LCD)
        # 接收本次训练文件名
        self.MotorWindow.filename_changed.connect(self.GetFilename)
        # 显示上位机窗口
        self.MotorWindow.show()

    def limitedSpeedSendButton_clicked(self):
        limitedSpeedValue = self.limitiedSpeedValue.value()
        print("limitedSpeedValue:", limitedSpeedValue)
        self.MotorWindow.Com_Send_Data(f'lsv:{limitedSpeedValue}')
        self.sendResult.setText("发送完成")

    def GetFilename(self, filename):
        self.filename = filename
        # print(f"Received name is: {self.filename}\n")

    def JointInfo_LCD(self, velocity, position, torque):
        # print("In the LCD func:", velocity, position, torque)
        # 记录角度初始值，用于极限位置更改，如果是第一次运行，记录下当前的角度值
        if self.Angle_Initial == 0:
            self.Angle_Initial = -position/1456
            print(f"The Angle_Initial = {self.Angle_Initial}")
        # 将关节数据共享到类变量中，便于绘图
        self.Velocity_Current = velocity/1456
        self.Angle_Current = -position/1456
        self.Angle_Current_Show = -position/1456 + self.Angle_Initial
        self.Torque_Current = torque*0.01
        # 数值量显示，保留一位小数
        self.torque.display(round(self.Torque_Current, 1))
        self.angle.display(round(self.Angle_Current_Show, 1))
        # self.angle.display(round(self.Angle_Current, 1))
        self.speed.display(round(self.Velocity_Current, 1))


    def starttrainingbutton_click(self, StartTrainingFlag):
        # 开始计算训练时间
        self.timer.start(100)  # 每 100 毫秒（0.1 秒）发出一次 timeout 信号
        self.TrainingWindow_StartTrainingFlag = StartTrainingFlag  # 开始训练标志位
        # 初始化极限位置
        self.downlimitation = round(self.Angle_Initial, 0)
        self.uplimitation = self.downlimitation - self.MotionRange  # 按照运动范围为90°，计算内收极限位置
        self.ERSDyna.display(round(self.downlimitation, 1))
        self.IRSDyna.display(round(self.uplimitation, 1))
        print("The downlimitation is : ", self.downlimitation)

    def CalculateTrainingTime(self):
        self.update_PlotData()
        # 取小数点后两位，进行四舍五入，避免self.ptr取不到整数的情况
        ptr_rounded = round(self.ptr, 2)
        if ptr_rounded.is_integer():
            self.elapsed_seconds += 1  # 增加 1 秒
            self.time.display(self.elapsed_seconds)

    def update_PlotData(self):
        # 添加新的力矩值
        new_value_motor1_torque = self.Torque_Current
        self.data_torque.append((self.ptr, new_value_motor1_torque))
        # 添加新的速度值
        new_value_motor1_velocity = self.Velocity_Current
        self.data_velocity.append((self.ptr, new_value_motor1_velocity))
        # 解包数据
        times_torque, torque_values = zip(*self.data_torque)
        times_velocity, velocity_values = zip(*self.data_velocity)
        # 注意，zip(*self.data)返回的是元组，需要转换成NumPy数组
        times = np.array(times_torque)
        torque_values = np.array(torque_values)
        velocity_values = np.array(velocity_values)
        # 计算相关数据，用于训练数据显示
        self.IRpeaktorque_data = np.max(torque_values[torque_values >= 0])
        self.ERpeaktorque_data = np.min(torque_values[torque_values <= 0])
        self.IRtotalworkdone_data = self.MotionRange/180 * np.pi * np.sum(torque_values[torque_values >= 0])
        self.ERtotalworkdone_data = self.MotionRange/180 * np.pi * np.sum(torque_values[torque_values <= 0])
        # 计算训练进度
        self.ProcessValue = abs(self.Angle_Current - self.downlimitation) / self.MotionRange * 100
        # print(f"The self.Angle_Current is:{self.Angle_Current}\n")
        # print(f"The self.downlimitation is:{self.downlimitation}\n")
        self.ProcessValue = int(self.ProcessValue)
        # print("The process is:", self.ProcessValue)
        # 根据进度条来计算重复次数
        if self.ProcessValue >= 90 and self.Countflag == 0:
            self.Countflag = 1
        if self.ProcessValue <= 10 and self.Countflag == 1:
            self.CountTimes = self.CountTimes + 1
            self.Countflag = 0
        if self.CountTimes >= self.CountTimesTotal:
            self.CountGroup = self.CountGroup + 1
            self.CountTimes = self.CountTimes - self.CountTimesTotal
        if self.CountGroup >= self.CountuGroupTotal:
            if not self.message_shown:
                self.message_box.setText("训练结束")
                self.message_box.setWindowTitle("通知")
                self.message_box.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)  # 设置标准按钮

                result = self.message_box.exec_()  # 显示弹窗并等待用户响应

                if result == QMessageBox.Ok:
                    txData = 'c';  # 控制关节电机1失能
                    self.MotorWindow.Com_Send_Data(txData)
                    # 停止数据更新
                    self.stopDataUpdates()
                    # 关闭串口
                    QTimer.singleShot(3000, self.MotorWindow.StopButton_clicked)
                    self.message_shown = True

        if not self.message_shown:
            # 训练数据显示更新
            self.TrainingDataShow()
            # 更新曲线
            self.torque_curve_1.setData(times, torque_values)
            self.velocity_curve_1.setData(times, velocity_values)
            # 更新x轴记录点
            self.ptr += 0.1
            # 重新设定 x 相关的坐标原点
            # self.torque_curve_1.setPos(self.ptr, 0)
        else:
            # 显示所有训练数据
            self.ShowAllDataInFig(self.filename)



    def stopDataUpdates(self):
        # 停止数据更新的方法
        if self.timer.isActive():
            self.timer.stop()  # 停止定时器

    def ShowAllDataInFig(self, filename):
        # filename = f"log/joint_data_20240424_171656.txt"
        try:
            with open(filename, 'r') as file:
                lines = file.readlines()

            # 初始化列表来收集每列的数据
            time = []
            velocity = []
            torque = []

            # 遍历每行，收集每列的数据
            for line in lines:
                parts = line.strip().split(',')  # 假设数据由逗号分隔
                if len(parts) >= 4:  # 确保行中有足够的数据
                    time.append(float(parts[0]))  # 第一列数据
                    velocity.append(float(parts[1])) # 第二列数据
                    torque.append(float(parts[3]))  # 第四列数据

            torque = [(t*0.01) for t in torque]
            velocity = [(v/1456) for v in velocity]

            # 更新曲线
            self.torque_curve_1.setData(time, torque)
            self.velocity_curve_1.setData(time, velocity)

            # 强制图表更新
            self.torque_plot_widget.update()
            self.velocity_plot_widget.update()

        except IOError:
            print("Error: File does not exist or cannot be accessed.")
        except IndexError:
            print("Error: Some lines in the file may not have enough columns.")
        except ValueError:
            print("Error: Data conversion error, please check the data format in the file.")

    def TrainingDataShow(self):
        # 训练峰值力矩，总量更新
        self.IRpeaktorque.display(round(self.IRpeaktorque_data, 1))
        self.ERpeaktorque.display(round(self.ERpeaktorque_data, 1))
        self.IRtotalworkdone.display(round(self.IRtotalworkdone_data, 1))
        self.ERtotalworkdone.display(round(self.ERtotalworkdone_data, 1))
        # 训练组数更新
        self.Repsnow.display(self.CountTimes)
        self.Setnow.display(self.CountGroup)
        self.Repstotal.display(self.CountTimesTotal)
        self.Settotal.display(self.CountuGroupTotal)
        # 训练进度更新
        self.progressBar.setValue(self.ProcessValue)

    def ERSIncButton_clicked(self):
        self.downlimitation = self.downlimitation + 5  # 外展运动，每次增加5°
        self.ERSDyna.display(round(self.downlimitation, 1))
        self.MotorWindow.Com_Send_Data(f'ERS:{5}')
        print(f"The downlimitation is {self.downlimitation}")
        print("ERSIncButton is clicked")

    def ERSDecButton_clicked(self):
        self.downlimitation = self.downlimitation - 5  # 外展运动，每次减少5°
        self.ERSDyna.display(round(self.downlimitation, 1))
        self.MotorWindow.Com_Send_Data(f'ERS:{-5}')
        print(f"The downlimitation is {self.downlimitation}")
        print("ERSDecButton is clicked")

    def IRSIncButton_clicked(self):
        self.uplimitation = self.uplimitation + 5  # 内收运动，每次增加5°
        self.IRSDyna.display(round(self.uplimitation, 1))
        self.MotorWindow.Com_Send_Data(f'IRS:{5}')
        print(f"The uplimitation is {self.uplimitation}")
        print("IRSIncButton is clicked")

    def IRSDecButton_clicked(self):
        self.uplimitation = self.uplimitation - 5  # 内收运动，每次减少5°
        self.IRSDyna.display(round(self.uplimitation, 1))
        self.MotorWindow.Com_Send_Data(f'IRS:{-5}')
        print(f"The uplimitation is {self.uplimitation}")
        print("IRSDecButton is clicked")



# 串口上位机窗口
class MyMotorWindow(QMainWindow, Ui_MotorWindowWidget):
    joint_data_received = pyqtSignal(int, int, int)
    start_training_flag = pyqtSignal(bool)
    filename_changed = pyqtSignal(str)


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
        # 添加变量属性
        self.rx_buffer = b''
        # 初始化开始训练标志位
        self.StartTrainingFlag = 0

    def InitialCom(self):
        # 初始化波特率(115200)
        self.baud.setCurrentIndex(6)
        # 初始化串口号
        self.serialport.clear()
        com = QSerialPort()
        com_list = QSerialPortInfo.availablePorts()
        for info in com_list:
            com.setPort(info)
            if com.open(QSerialPort.ReadWrite):
                self.serialport.addItem(info.portName())
                com.close()

    def CreateSignalSlot(self):
        # 串口功能按键
        self.StartButton.clicked.connect(self.StartButton_clicked)
        self.StopButton.clicked.connect(self.StopButton_clicked)
        self.RefreshButton.clicked.connect(self.RefreshButton_clicked)
        self.SendButton.clicked.connect(self.SendButton_clicked)
        self.CleanButton.clicked.connect(self.CleanButton_clicked)
        self.com.readyRead.connect(self.Com_Receive_Data)  # 接收数据
        self.Hex.stateChanged.connect(self.HexClicked)
        self.SaveButton.clicked.connect(self.SaveButton_clicked)
        # 电机功能按键
        # 1 motor
        self.motor1initial.clicked.connect(self.motor1initialButton_clicked)
        self.motor1enable.clicked.connect(self.motor1enableButton_clicked)
        self.motor1disable.clicked.connect(self.motor1disableButton_clicked)
        self.motor1stop.clicked.connect(self.motor1stopButton_clicked)
        # 2 motor
        self.motor2initial.clicked.connect(self.motor2initialButton_clicked)
        self.motor2enable.clicked.connect(self.motor2enableButton_clicked)
        self.motor2disable.clicked.connect(self.motor2disableButton_clicked)
        self.motor2stop.clicked.connect(self.motor2stopButton_clicked)
        # 3 motor
        self.motor3initial.clicked.connect(self.motor3initialButton_clicked)
        self.motor3enable.clicked.connect(self.motor3enableButton_clicked)
        self.motor3disable.clicked.connect(self.motor3disableButton_clicked)
        self.motor3stop.clicked.connect(self.motor3stopButton_clicked)
        # 开始训练按键
        self.StartTrainingButton.clicked.connect(self.StartTrainingButton_clicked)


    def Com_Send_Data(self, txData):
        if len(txData) == 0:
            return
        txData += '\r\n'
        # 如果发送的是纯文本数据（非十六进制）
        if not self.Hex.isChecked():
            try:
                # 尝试将数据编码为 GBK 格式并发送
                self.com.write(txData.encode('GBK'))
                # print("txData:",txData)
            except Exception as e:
                # 如果发送过程中出现问题，显示错误消息
                QMessageBox.critical(self, '错误', f'发送数据失败: {e}')
        else:
            # 移除字符串中的空格，并将其转换为小写
            data = txData.replace(' ', '').lower()

            # 如果数据长度为奇数，去掉最后一个字符
            if len(data) % 2 == 1:
                data = data[:-1]

            # 检查是否所有字符都是十六进制字符
            if not all(c in '0123456789abcdef' for c in data):
                QMessageBox.critical(self, '错误', '包含非十六进制数')
                return

            try:
                # 将十六进制字符串转换为字节数据
                hexData = bytes.fromhex(data)

                # 发送十六进制数据
                self.com.write(hexData)
            except Exception as e:
                # 如果发送过程中出现问题，显示错误消息
                QMessageBox.critical(self, '异常', f'发送十六进制数据失败: {e}')
        # print("what i send is:", txData)

    # 用于训练窗口数据的实时显示
    def handle_data_packet(self, packet):
        try:
            packet = packet.strip()
            if packet.startswith(b"VEL:") and b"POS:" in packet and b"TOR:" in packet:
                # 解析关节数据包
                data_parts = packet.split(b",")
                # print("data_parts:", data_parts)
                # print("len(data_parts):", len(data_parts))
                try:
                    # print("data_parts[0]:", data_parts[0])
                    # print("data_parts[1]:", data_parts[1])
                    # print("data_parts[2]:", data_parts[2])
                    velocity = -int(data_parts[0].split(b":")[1])
                    position = -int(data_parts[1].split(b":")[1])
                    torque = int(data_parts[2].split(b":")[1])
                    if self.StartTrainingFlag == 1:
                        # 计算训练时长
                        training_duration = time.time() - self.training_start_time
                        formatted_duration = f"{training_duration:.3f}"
                        # 使用记录开始的时间作为文件名，并将文件存储到log文件夹中
                        with open(self.filename, "a") as file:
                            file.write(f"{formatted_duration}, {velocity}, {position}, {torque} \n")
                    # 发送关节数据给训练窗口
                    self.joint_data_received.emit(velocity, position, torque)
                except ValueError:
                    QMessageBox.critical(self, '错误', '解析关节数据失败')
        except Exception as e:
            QMessageBox.critical(self, '错误', f'处理数据包时发生异常: {str(e)}')

    # 用于接收窗口的数据显示
    def Com_Receive_Data(self):
        try:
            rxData = bytes(self.com.readAll())
            self.rx_buffer += rxData
            # 检查是否收到完整的数据包，然后处理它
            if b'\n' in self.rx_buffer:
                packets = self.rx_buffer.split(b'\n')
                self.rx_buffer = packets.pop()  # 将最后一个（可能不完整的）数据包保留在缓冲区中
                for packet in packets:
                    self.handle_data_packet(packet)

        except Exception as e:
            QMessageBox.critical(self, '严重错误', f'串口接收数据错误: {str(e)}')

        if self.Hex.isChecked():
            Data = binascii.b2a_hex(rxData).decode('ascii')
            hexStr = ' 0x'.join(re.findall('(.{2})', Data))
            hexStr = '0x' + hexStr
            self.Showcontent.insertPlainText(hexStr)
            self.Showcontent.insertPlainText(' ')
        else:
            try:
                # 使用UTF-8编码解码接收到的数据
                decoded_data = rxData.decode('GBK')
                self.Showcontent.insertPlainText(decoded_data)
            except UnicodeDecodeError:
                QMessageBox.critical(self, '错误', '解码接收数据失败')

        # 将滚动条移动到底部
        self.Showcontent.moveCursor(QtGui.QTextCursor.End)

    def HexClicked(self):
        if self.Hex.isChecked() == True:
            self.Showcontent.insertPlainText('\n')

    def SendButton_clicked(self):
        txData = self.SendingTextEdit.toPlainText()
        self.Com_Send_Data(txData)

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
        # 设置数据位，默认为8位
        self.com.setDataBits(QSerialPort.Data8)
        # 设置校验位，默认为无校验
        self.com.setParity(QSerialPort.NoParity)
        # 设置停止位，默认为1位
        self.com.setStopBits(QSerialPort.OneStop)
        # 如果需要，还可以设置流控制，默认为无流控制
        # self.com.setFlowControl(QSerialPort.NoFlowControl)

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

    def CleanButton_clicked(self):
        self.Showcontent.clear()

    def SaveButton_clicked(self):
        # 获取 QTextEdit 中的数据
        text_data = self.Showcontent.toPlainText()

        # 创建一个 QFileDialog 来选择保存文件的位置和名称
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getSaveFileName(self, "保存数据", "", "所有文件 (*);;文本文件 (*.txt)",
                                                  options=options)

        # 检查用户是否选择了一个文件名
        if fileName:
            # 打开文件并写入数据
            with open(fileName, 'w') as f:
                f.write(text_data)

    def motor1initialButton_clicked(self):
        txData = 'a';
        self.Com_Send_Data(txData)

    def motor1enableButton_clicked(self):
        txData = 'b';
        self.Com_Send_Data(txData)

    def motor1disableButton_clicked(self):
        txData = 'c';
        self.Com_Send_Data(txData)

    def motor1stopButton_clicked(self):
        txData = 'd';
        self.Com_Send_Data(txData)

    def motor2initialButton_clicked(self):
        txData = 'e';
        self.Com_Send_Data(txData)

    def motor2enableButton_clicked(self):
        txData = 'f';
        self.Com_Send_Data(txData)

    def motor2disableButton_clicked(self):
        txData = 'g';
        self.Com_Send_Data(txData)

    def motor2stopButton_clicked(self):
        txData = 'h';
        self.Com_Send_Data(txData)

    def motor3initialButton_clicked(self):
        txData = 'i';
        self.Com_Send_Data(txData)

    def motor3enableButton_clicked(self):
        txData = 'j';
        self.Com_Send_Data(txData)

    def motor3disableButton_clicked(self):
        txData = 'k';
        self.Com_Send_Data(txData)

    def motor3stopButton_clicked(self):
        txData = 'l';
        self.Com_Send_Data(txData)
    def StartTrainingButton_clicked(self):
        self.StartTrainingFlag = True
        self.training_start_time = time.time()
        start_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.start_training_flag.emit(self.StartTrainingFlag)
        log_dir = "log"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        self.filename = f"log/joint_data_{start_time}.txt"
        self.filename_changed.emit(self.filename)
        # print(f"filename is: {self.filename}")



if __name__ == '__main__':
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
    QtGui.QGuiApplication.setAttribute(QtCore.Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    app = QApplication(sys.argv)
    myWin = MyTrainingWindow()
    myWin.show()
    sys.exit(app.exec_())

