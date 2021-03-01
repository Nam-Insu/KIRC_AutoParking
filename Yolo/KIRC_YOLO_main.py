# from openCV_yoloTiny_basic import kirc_yolo
import openCV_yoloTiny_basic

import cv2
import KIRC_master_function
from File_IO import *
import serial

if __name__ == '__main__':

 #   uartPORT = '/dev/ttyUSB0'
    cnt = 0
#    ser = serial.serial_for_url(uartPORT, baudrate=9600, timeout=1)
    yolo = openCV_yoloTiny_basic.kirc_yolo()
    master = KIRC_master_function.KIRC_function()
    while True:
        yolo.main_work()

        dirGui = 'GtoM'

        packetGui = ''
        packetYolo = yolo.Packet_ROI

        try:
            packetGui = File_read_gui(dirGui)
            File_write_gui(dirGui, '')

        except (RuntimeError, TypeError, NameError):
            print("Error !!")

        if packetGui != None and len(packetGui) >= 3:
            master.queueingBuffer(packetGui)

        master.bufToOrderING()  # 현재 실행 버퍼에 넣었다.
        # ING에 명령 들어갔으면서 명령이 Input 명령이면 바리케이트여는 신호를 보내
        if master.bFlagOrderING == True:
            master.bFlagOrderING = False
            ser.write(bytes('999', encoding='ascii'))  # 출력방식1
            
        # ROI에서 active된 애들 RoiList에 넣어줌
        # RoiList = [01, 07, 13, 00] 이런식으로 저장이 돼
        master.findActiveROI_makeRoiList(packetYolo)

        # 활성화된 roi가 내가 원하는 led 의 위치와 동일한지 확인
        # 동일하면 코어텍스로 명령어를 보내
        # 해당 Area에 대한 명령어가 종료되면 다음 명령어를 받을 준비.
        str_tmp_1 = ''
        str_tmp_2 = ''
        for R in master.RoiList:
            if master.que_orderING_1.__len__() > 0:
                str_tmp_1 = master.que_orderING_1[0][0] + master.que_orderING_1[0][1]
            if master.que_orderING_2.__len__() > 0:
                str_tmp_2 = master.que_orderING_2[0][0] + master.que_orderING_2[0][1]

            if str_tmp_1 == R:  # str_tmp_1 : '17', R = '17', 둘다 <str>
                print("현재 내릴 명령어 : " + master.que_orderING_1[0])
                ser.write(bytes(master.que_orderING_1[0], encoding='ascii'))  # 출력방식1
                master.que_orderING_1.popleft()  # 이 명령어를 실행했으면 이제 다시는 실행안할거니까 삭제
                if master.que_orderING_1.__len__() == 0:
                    master.bool_parkingArea_1 = True
                    print("1구역 명령완료")

            if str_tmp_2 == R:  # 16R 이런식으로 되어있어서 수정해야함
                print("현재 내릴 명령어 : " + master.que_orderING_2[0])
                ser.write(bytes(master.que_orderING_2[0], encoding='ascii'))  # 출력방식1
                master.que_orderING_2.popleft()  # 이 명령어를 실행했으면 이제 다시는 실행안할거니까 삭제
                if master.que_orderING_2.__len__() == 0:
                    master.bool_parkingArea_2 = True
                    print("2구역 명령완료")

        key = cv2.waitKey(1)
        if key == ord('q'):
            break
