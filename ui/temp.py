from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Main Window')
        self.setGeometry(100, 100, 300, 200)

        self.open_button = QPushButton('Open New Window', self)
        self.open_button.setGeometry(50, 50, 200, 50)
        self.open_button.clicked.connect(self.open_new_window)

    def open_new_window(self):
        self.new_window = NewWindow()
        self.new_window.show()

class NewWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('New Window')
        self.setGeometry(500, 100, 300, 200)

        self.new_window_button = QPushButton('Open Another New Window', self)
        self.new_window_button.setGeometry(50, 50, 200, 50)
        self.new_window_button.clicked.connect(self.open_another_new_window)

    def open_another_new_window(self):
        self.another_new_window = NewWindow()
        self.another_new_window.show()

if __name__ == '__main__':
    app = QApplication([])
    main_window = MainWindow()
    main_window.show()
    app.exec_()
