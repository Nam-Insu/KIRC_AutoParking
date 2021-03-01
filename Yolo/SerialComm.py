# 사용법 :
# [Errno 2] could not open port COM1: [Errno 2] No such file or directory: 'COM1'
# 이런 오류 발생시 : http://js-cristo.blogspot.com/2012/05/tty.html 참고하여 dialout 설정해준다.(재부팅 필수)
# 유아트 com port 확인 방법은 터미널창에서 dmesg|grep tty 을 입력하여 확인 가능.

import time
import serial

# 포트 설정
PORT = '/dev/ttyUSB0'
# 연결
ser = serial.serial_for_url(PORT, baudrate=115200, timeout=1)

while True:
    print('Send message')
    ser.write(bytes('h', encoding='ascii')) #출력방식1
    time.sleep(1)
