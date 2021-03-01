# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 's2.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

# import assign 보다 위에 있어야 한다.
car_number = [' ',' ',' ',' ']
car_number_str = ''

from PyQt5 import QtCore, QtGui, QtWidgets
from assign import assign_pop, assign_autopush


print("gui_CarNumber 불리는")
cnt = 0

class Ui_MainWindow1(object):

    def setup(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(100, 270, 626, 107))
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.gridLayoutWidget.setFont(font)
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.pushButton_4 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_4.sizePolicy().hasHeightForWidth())
        self.pushButton_4.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_4.setFont(font)
        self.pushButton_4.setObjectName("pushButton_4")
        self.gridLayout.addWidget(self.pushButton_4, 0, 3, 1, 1)
        self.pushButton_2 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_2.sizePolicy().hasHeightForWidth())
        self.pushButton_2.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_2.setFont(font)
        self.pushButton_2.setObjectName("pushButton_2")
        self.gridLayout.addWidget(self.pushButton_2, 0, 1, 1, 1)
        self.pushButton_5 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_5.sizePolicy().hasHeightForWidth())
        self.pushButton_5.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_5.setFont(font)
        self.pushButton_5.setObjectName("pushButton_5")
        self.gridLayout.addWidget(self.pushButton_5, 0, 4, 1, 1)
        self.pushButton_3 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_3.sizePolicy().hasHeightForWidth())
        self.pushButton_3.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_3.setFont(font)
        self.pushButton_3.setObjectName("pushButton_3")
        self.gridLayout.addWidget(self.pushButton_3, 0, 2, 1, 1)
        self.pushButton_8 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_8.sizePolicy().hasHeightForWidth())
        self.pushButton_8.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_8.setFont(font)
        self.pushButton_8.setObjectName("pushButton_8")
        self.gridLayout.addWidget(self.pushButton_8, 1, 2, 1, 1)
        self.pushButton_7 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_7.sizePolicy().hasHeightForWidth())
        self.pushButton_7.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_7.setFont(font)
        self.pushButton_7.setObjectName("pushButton_7")
        self.gridLayout.addWidget(self.pushButton_7, 1, 1, 1, 1)
        self.pushButton_9 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_9.sizePolicy().hasHeightForWidth())
        self.pushButton_9.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_9.setFont(font)
        self.pushButton_9.setObjectName("pushButton_9")
        self.gridLayout.addWidget(self.pushButton_9, 1, 3, 1, 1)
        self.pushButton_1 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_1.sizePolicy().hasHeightForWidth())
        self.pushButton_1.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_1.setFont(font)
        self.pushButton_1.setObjectName("pushButton_1")

        self.gridLayout.addWidget(self.pushButton_1, 0, 0, 1, 1)
        self.pushButton_6 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_6.sizePolicy().hasHeightForWidth())
        self.pushButton_6.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_6.setFont(font)
        self.pushButton_6.setObjectName("pushButton_6")
        self.gridLayout.addWidget(self.pushButton_6, 1, 0, 1, 1)
        self.pushButton_0 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_0.sizePolicy().hasHeightForWidth())
        self.pushButton_0.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_0.setFont(font)
        self.pushButton_0.setObjectName("pushButton_0")
        self.gridLayout.addWidget(self.pushButton_0, 1, 4, 1, 1)
        self.pushButton_cancel = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_cancel.sizePolicy().hasHeightForWidth())
        self.pushButton_cancel.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_cancel.setFont(font)
        self.pushButton_cancel.setObjectName("pushButton_cancel")
        self.gridLayout.addWidget(self.pushButton_cancel, 1, 5, 1, 1)
        self.pushButton_check = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_check.sizePolicy().hasHeightForWidth())
        self.pushButton_check.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_check.setFont(font)
        self.pushButton_check.setObjectName("pushButton_check")
        self.gridLayout.addWidget(self.pushButton_check, 0, 5, 1, 1)
        self.layoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget.setGeometry(QtCore.QRect(210, 50, 384, 190))
        self.layoutWidget.setObjectName("layoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.layoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label1 = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label1.sizePolicy().hasHeightForWidth())
        self.label1.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        font.setPointSize(72)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.label1.setFont(font)
        self.label1.setAlignment(QtCore.Qt.AlignCenter)
        self.label1.setObjectName("label1")
        self.horizontalLayout.addWidget(self.label1)
        self.label2 = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label2.sizePolicy().hasHeightForWidth())
        self.label2.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        font.setPointSize(72)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.label2.setFont(font)
        self.label2.setAlignment(QtCore.Qt.AlignCenter)
        self.label2.setObjectName("label2")
        self.horizontalLayout.addWidget(self.label2)
        self.label3 = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label3.sizePolicy().hasHeightForWidth())
        self.label3.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        font.setPointSize(72)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.label3.setFont(font)
        self.label3.setAlignment(QtCore.Qt.AlignCenter)
        self.label3.setObjectName("label3")
        self.horizontalLayout.addWidget(self.label3)
        self.label4 = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label4.sizePolicy().hasHeightForWidth())
        self.label4.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        font.setPointSize(72)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.label4.setFont(font)
        self.label4.setAlignment(QtCore.Qt.AlignCenter)
        self.label4.setObjectName("label4")
        self.horizontalLayout.addWidget(self.label4)
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setGeometry(QtCore.QRect(200, 400, 461, 111))
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        font.setPointSize(22)
        font.setItalic(True)
        self.label_5.setFont(font)
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.line = QtWidgets.QFrame(self.centralwidget)
        self.line.setGeometry(QtCore.QRect(250, 149, 311, 121))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.line.sizePolicy().hasHeightForWidth())
        self.line.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        font.setPointSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.line.setFont(font)
        self.line.setFrameShadow(QtWidgets.QFrame.Plain)
        self.line.setLineWidth(3)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setObjectName("line")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 25))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        ### 버튼 클릭했을 때 실행하는 함수들
        self.pushButton_1.clicked.connect(self.btn_1)
        self.pushButton_2.clicked.connect(self.btn_2)
        self.pushButton_3.clicked.connect(self.btn_3)
        self.pushButton_4.clicked.connect(self.btn_4)
        self.pushButton_5.clicked.connect(self.btn_5)
        self.pushButton_6.clicked.connect(self.btn_6)
        self.pushButton_7.clicked.connect(self.btn_7)
        self.pushButton_8.clicked.connect(self.btn_8)
        self.pushButton_9.clicked.connect(self.btn_9)
        self.pushButton_0.clicked.connect(self.btn_0)
        self.pushButton_check.clicked.connect(self.btn_check)
        self.pushButton_cancel.clicked.connect(self.btn_cancel)

    def retranslateUi(self, MainWindow):

        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton_4.setText(_translate("MainWindow", "4"))
        self.pushButton_2.setText(_translate("MainWindow", "2"))
        self.pushButton_5.setText(_translate("MainWindow", "5"))
        self.pushButton_3.setText(_translate("MainWindow", "3"))
        self.pushButton_8.setText(_translate("MainWindow", "8"))
        self.pushButton_7.setText(_translate("MainWindow", "7"))
        self.pushButton_9.setText(_translate("MainWindow", "9"))
        self.pushButton_1.setText(_translate("MainWindow", "1"))
        self.pushButton_6.setText(_translate("MainWindow", "6"))
        self.pushButton_0.setText(_translate("MainWindow", "0"))
        self.pushButton_cancel.setText(_translate("MainWindow", "지우기"))
        self.pushButton_check.setText(_translate("MainWindow", "확인"))
        self.label1.setText(_translate("MainWindow", car_number[0]))
        self.label2.setText(_translate("MainWindow", car_number[1]))
        self.label3.setText(_translate("MainWindow", car_number[2]))
        self.label4.setText(_translate("MainWindow", car_number[3]))
        self.label_5.setText(_translate("MainWindow", "차량번호 4자리를 입력하여 주십시오 "))



    @staticmethod
    def btn_1():
        global car_number
        global cnt

        if cnt != 4:
            cnt += 1
            if cnt == 1:
                car_number[0] = '1'
            elif cnt == 2:
                car_number[1] = '1'
            elif cnt == 3:
                car_number[2] = '1'
            elif cnt == 4:
                car_number[3] = '1'
    @staticmethod
    def btn_2():
        global car_number
        global cnt

        if cnt != 4:
            cnt += 1
            if cnt == 1:
                car_number[0] = '2'
            elif cnt == 2:
                car_number[1] = '2'
            elif cnt == 3:
                car_number[2] = '2'
            elif cnt == 4:
                car_number[3] = '2'
    @staticmethod
    def btn_3():
        global car_number
        global cnt

        if cnt != 4:
            cnt += 1
            if cnt == 1:
                car_number[0] = '3'
            elif cnt == 2:
                car_number[1] = '3'
            elif cnt == 3:
                car_number[2] = '3'
            elif cnt == 4:
                car_number[3] = '3'
    @staticmethod
    def btn_4():
        global car_number
        global cnt

        if cnt != 4:
            cnt += 1
            if cnt == 1:
                car_number[0] = '4'
            elif cnt == 2:
                car_number[1] = '4'
            elif cnt == 3:
                car_number[2] = '4'
            elif cnt == 4:
                car_number[3] = '4'
    @staticmethod
    def btn_5():
        global car_number
        global cnt

        if cnt != 4:
            cnt += 1
            if cnt == 1:
                car_number[0] = '5'
            elif cnt == 2:
                car_number[1] = '5'
            elif cnt == 3:
                car_number[2] = '5'
            elif cnt == 4:
                car_number[3] = '5'
    @staticmethod
    def btn_6():
        global car_number
        global cnt

        if cnt != 4:
            cnt += 1
            if cnt == 1:
                car_number[0] = '6'
            elif cnt == 2:
                car_number[1] = '6'
            elif cnt == 3:
                car_number[2] = '6'
            elif cnt == 4:
                car_number[3] = '6'
    @staticmethod
    def btn_7():
        global car_number
        global cnt

        if cnt != 4:
            cnt += 1
            if cnt == 1:
                car_number[0] = '7'
            elif cnt == 2:
                car_number[1] = '7'
            elif cnt == 3:
                car_number[2] = '7'
            elif cnt == 4:
                car_number[3] = '7'
    @staticmethod
    def btn_8():
        global car_number
        global cnt

        if cnt != 4:
            cnt += 1
            if cnt == 1:
                car_number[0] = '8'
            elif cnt == 2:
                car_number[1] = '8'
            elif cnt == 3:
                car_number[2] = '8'
            elif cnt == 4:
                car_number[3] = '8'
    @staticmethod
    def btn_9():
        global car_number
        global cnt

        if cnt != 4:
            cnt += 1
            if cnt == 1:
                car_number[0] = '9'
            elif cnt == 2:
                car_number[1] = '9'
            elif cnt == 3:
                car_number[2] = '9'
            elif cnt == 4:
                car_number[3] = '9'
    @staticmethod
    def btn_0():
        global car_number
        global cnt

        if cnt != 4:
            cnt += 1
            if cnt == 1:
                car_number[0] = '0'
            elif cnt == 2:
                car_number[1] = '0'
            elif cnt == 3:
                car_number[2] = '0'
            elif cnt == 4:
                car_number[3] = '0'


    @staticmethod
    def btn_check():
        global car_number_str

        car_number_str = car_number[0] + car_number[1] + car_number[2] + car_number[3]
        if len(car_number_str) != 4:
            print('4자리를 입력하지 않았습니다')
        else:
            #assign_autopush(car_number_str)     # 명근이가 바꿔줄때까지 임시로 씀
            assign_pop(car_number_str)

    #################################
    ##### 위에서 선언한 list와 변수 #####
    # car_number = [' ', ' ', ' ', ' ']
    # car_number_str = ''
    # cnt = 0

    @staticmethod
    def btn_cancel():
        global car_number_str
        global cnt
        if cnt == 4:
            car_number[3] = ' '
            cnt -= 1
        elif cnt == 3:
            car_number[2] = ' '
            cnt -= 1
        elif cnt == 2:
            car_number[1] = ' '
            cnt -= 1
        elif cnt == 1:
            car_number[0] = ' '
            cnt -= 1
        elif cnt == 0:
            print("삭제할 데이터가 없습니다.")

class Ui_MainWindow1_in(object):

    def setup(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(100, 270, 626, 107))
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.gridLayoutWidget.setFont(font)
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.pushButton_4 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_4.sizePolicy().hasHeightForWidth())
        self.pushButton_4.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_4.setFont(font)
        self.pushButton_4.setObjectName("pushButton_4")
        self.gridLayout.addWidget(self.pushButton_4, 0, 3, 1, 1)
        self.pushButton_2 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_2.sizePolicy().hasHeightForWidth())
        self.pushButton_2.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_2.setFont(font)
        self.pushButton_2.setObjectName("pushButton_2")
        self.gridLayout.addWidget(self.pushButton_2, 0, 1, 1, 1)
        self.pushButton_5 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_5.sizePolicy().hasHeightForWidth())
        self.pushButton_5.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_5.setFont(font)
        self.pushButton_5.setObjectName("pushButton_5")
        self.gridLayout.addWidget(self.pushButton_5, 0, 4, 1, 1)
        self.pushButton_3 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_3.sizePolicy().hasHeightForWidth())
        self.pushButton_3.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_3.setFont(font)
        self.pushButton_3.setObjectName("pushButton_3")
        self.gridLayout.addWidget(self.pushButton_3, 0, 2, 1, 1)
        self.pushButton_8 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_8.sizePolicy().hasHeightForWidth())
        self.pushButton_8.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_8.setFont(font)
        self.pushButton_8.setObjectName("pushButton_8")
        self.gridLayout.addWidget(self.pushButton_8, 1, 2, 1, 1)
        self.pushButton_7 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_7.sizePolicy().hasHeightForWidth())
        self.pushButton_7.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_7.setFont(font)
        self.pushButton_7.setObjectName("pushButton_7")
        self.gridLayout.addWidget(self.pushButton_7, 1, 1, 1, 1)
        self.pushButton_9 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_9.sizePolicy().hasHeightForWidth())
        self.pushButton_9.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_9.setFont(font)
        self.pushButton_9.setObjectName("pushButton_9")
        self.gridLayout.addWidget(self.pushButton_9, 1, 3, 1, 1)
        self.pushButton_1 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_1.sizePolicy().hasHeightForWidth())
        self.pushButton_1.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_1.setFont(font)
        self.pushButton_1.setObjectName("pushButton_1")

        self.gridLayout.addWidget(self.pushButton_1, 0, 0, 1, 1)
        self.pushButton_6 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_6.sizePolicy().hasHeightForWidth())
        self.pushButton_6.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_6.setFont(font)
        self.pushButton_6.setObjectName("pushButton_6")
        self.gridLayout.addWidget(self.pushButton_6, 1, 0, 1, 1)
        self.pushButton_0 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_0.sizePolicy().hasHeightForWidth())
        self.pushButton_0.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_0.setFont(font)
        self.pushButton_0.setObjectName("pushButton_0")
        self.gridLayout.addWidget(self.pushButton_0, 1, 4, 1, 1)
        self.pushButton_cancel = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_cancel.sizePolicy().hasHeightForWidth())
        self.pushButton_cancel.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_cancel.setFont(font)
        self.pushButton_cancel.setObjectName("pushButton_cancel")
        self.gridLayout.addWidget(self.pushButton_cancel, 1, 5, 1, 1)
        self.pushButton_check = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_check.sizePolicy().hasHeightForWidth())
        self.pushButton_check.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        self.pushButton_check.setFont(font)
        self.pushButton_check.setObjectName("pushButton_check")
        self.gridLayout.addWidget(self.pushButton_check, 0, 5, 1, 1)
        self.layoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget.setGeometry(QtCore.QRect(210, 50, 384, 190))
        self.layoutWidget.setObjectName("layoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.layoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label1 = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label1.sizePolicy().hasHeightForWidth())
        self.label1.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        font.setPointSize(72)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.label1.setFont(font)
        self.label1.setAlignment(QtCore.Qt.AlignCenter)
        self.label1.setObjectName("label1")
        self.horizontalLayout.addWidget(self.label1)
        self.label2 = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label2.sizePolicy().hasHeightForWidth())
        self.label2.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        font.setPointSize(72)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.label2.setFont(font)
        self.label2.setAlignment(QtCore.Qt.AlignCenter)
        self.label2.setObjectName("label2")
        self.horizontalLayout.addWidget(self.label2)
        self.label3 = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label3.sizePolicy().hasHeightForWidth())
        self.label3.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        font.setPointSize(72)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.label3.setFont(font)
        self.label3.setAlignment(QtCore.Qt.AlignCenter)
        self.label3.setObjectName("label3")
        self.horizontalLayout.addWidget(self.label3)
        self.label4 = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label4.sizePolicy().hasHeightForWidth())
        self.label4.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        font.setPointSize(72)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.label4.setFont(font)
        self.label4.setAlignment(QtCore.Qt.AlignCenter)
        self.label4.setObjectName("label4")
        self.horizontalLayout.addWidget(self.label4)
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setGeometry(QtCore.QRect(200, 400, 461, 111))
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        font.setPointSize(22)
        font.setItalic(True)
        self.label_5.setFont(font)
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.line = QtWidgets.QFrame(self.centralwidget)
        self.line.setGeometry(QtCore.QRect(250, 149, 311, 121))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.line.sizePolicy().hasHeightForWidth())
        self.line.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("NanumGothic")
        font.setPointSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.line.setFont(font)
        self.line.setFrameShadow(QtWidgets.QFrame.Plain)
        self.line.setLineWidth(3)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setObjectName("line")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 25))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        ### 버튼 클릭했을 때 실행하는 함수들
        self.pushButton_1.clicked.connect(self.btn_1)
        self.pushButton_2.clicked.connect(self.btn_2)
        self.pushButton_3.clicked.connect(self.btn_3)
        self.pushButton_4.clicked.connect(self.btn_4)
        self.pushButton_5.clicked.connect(self.btn_5)
        self.pushButton_6.clicked.connect(self.btn_6)
        self.pushButton_7.clicked.connect(self.btn_7)
        self.pushButton_8.clicked.connect(self.btn_8)
        self.pushButton_9.clicked.connect(self.btn_9)
        self.pushButton_0.clicked.connect(self.btn_0)
        self.pushButton_check.clicked.connect(self.btn_check)
        self.pushButton_cancel.clicked.connect(self.btn_cancel)

    def retranslateUi(self, MainWindow):

        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton_4.setText(_translate("MainWindow", "4"))
        self.pushButton_2.setText(_translate("MainWindow", "2"))
        self.pushButton_5.setText(_translate("MainWindow", "5"))
        self.pushButton_3.setText(_translate("MainWindow", "3"))
        self.pushButton_8.setText(_translate("MainWindow", "8"))
        self.pushButton_7.setText(_translate("MainWindow", "7"))
        self.pushButton_9.setText(_translate("MainWindow", "9"))
        self.pushButton_1.setText(_translate("MainWindow", "1"))
        self.pushButton_6.setText(_translate("MainWindow", "6"))
        self.pushButton_0.setText(_translate("MainWindow", "0"))
        self.pushButton_cancel.setText(_translate("MainWindow", "지우기"))
        self.pushButton_check.setText(_translate("MainWindow", "확인"))
        self.label1.setText(_translate("MainWindow", car_number[0]))
        self.label2.setText(_translate("MainWindow", car_number[1]))
        self.label3.setText(_translate("MainWindow", car_number[2]))
        self.label4.setText(_translate("MainWindow", car_number[3]))
        self.label_5.setText(_translate("MainWindow", "차량번호 4자리를 입력하여 주십시오 "))

    @staticmethod
    def btn_1():
        global car_number
        global cnt

        if cnt != 4:
            cnt += 1
            if cnt == 1:
                car_number[0] = '1'
            elif cnt == 2:
                car_number[1] = '1'
            elif cnt == 3:
                car_number[2] = '1'
            elif cnt == 4:
                car_number[3] = '1'

    @staticmethod
    def btn_2():
        global car_number
        global cnt

        if cnt != 4:
            cnt += 1
            if cnt == 1:
                car_number[0] = '2'
            elif cnt == 2:
                car_number[1] = '2'
            elif cnt == 3:
                car_number[2] = '2'
            elif cnt == 4:
                car_number[3] = '2'

    @staticmethod
    def btn_3():
        global car_number
        global cnt

        if cnt != 4:
            cnt += 1
            if cnt == 1:
                car_number[0] = '3'
            elif cnt == 2:
                car_number[1] = '3'
            elif cnt == 3:
                car_number[2] = '3'
            elif cnt == 4:
                car_number[3] = '3'

    @staticmethod
    def btn_4():
        global car_number
        global cnt

        if cnt != 4:
            cnt += 1
            if cnt == 1:
                car_number[0] = '4'
            elif cnt == 2:
                car_number[1] = '4'
            elif cnt == 3:
                car_number[2] = '4'
            elif cnt == 4:
                car_number[3] = '4'

    @staticmethod
    def btn_5():
        global car_number
        global cnt

        if cnt != 4:
            cnt += 1
            if cnt == 1:
                car_number[0] = '5'
            elif cnt == 2:
                car_number[1] = '5'
            elif cnt == 3:
                car_number[2] = '5'
            elif cnt == 4:
                car_number[3] = '5'

    @staticmethod
    def btn_6():
        global car_number
        global cnt

        if cnt != 4:
            cnt += 1
            if cnt == 1:
                car_number[0] = '6'
            elif cnt == 2:
                car_number[1] = '6'
            elif cnt == 3:
                car_number[2] = '6'
            elif cnt == 4:
                car_number[3] = '6'

    @staticmethod
    def btn_7():
        global car_number
        global cnt

        if cnt != 4:
            cnt += 1
            if cnt == 1:
                car_number[0] = '7'
            elif cnt == 2:
                car_number[1] = '7'
            elif cnt == 3:
                car_number[2] = '7'
            elif cnt == 4:
                car_number[3] = '7'

    @staticmethod
    def btn_8():
        global car_number
        global cnt

        if cnt != 4:
            cnt += 1
            if cnt == 1:
                car_number[0] = '8'
            elif cnt == 2:
                car_number[1] = '8'
            elif cnt == 3:
                car_number[2] = '8'
            elif cnt == 4:
                car_number[3] = '8'

    @staticmethod
    def btn_9():
        global car_number
        global cnt

        if cnt != 4:
            cnt += 1
            if cnt == 1:
                car_number[0] = '9'
            elif cnt == 2:
                car_number[1] = '9'
            elif cnt == 3:
                car_number[2] = '9'
            elif cnt == 4:
                car_number[3] = '9'

    @staticmethod
    def btn_0():
        global car_number
        global cnt

        if cnt != 4:
            cnt += 1
            if cnt == 1:
                car_number[0] = '0'
            elif cnt == 2:
                car_number[1] = '0'
            elif cnt == 3:
                car_number[2] = '0'
            elif cnt == 4:
                car_number[3] = '0'

    @staticmethod
    def btn_check():
        global car_number_str

        car_number_str = car_number[0] + car_number[1] + car_number[2] + car_number[3]
        if len(car_number_str) != 4:
            print('4자리를 입력하지 않았습니다')
        else:
            assign_autopush(car_number_str)
            print(car_number_str + "가 주차되었습니다.")


    @staticmethod
    def btn_cancel():
        global car_number_str
        global cnt
        if cnt == 4:
            car_number[3] = ' '
            cnt -= 1
        elif cnt == 3:
            car_number[2] = ' '
            cnt -= 1
        elif cnt == 2:
            car_number[1] = ' '
            cnt -= 1
        elif cnt == 1:
            car_number[0] = ' '
            cnt -= 1
        elif cnt == 0:
            print("삭제할 데이터가 없습니다.")



    #def btn_cancel():
'''
if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow1()
    ui.setup(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

'''

