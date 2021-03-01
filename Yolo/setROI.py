'''
    - q : quit operation
    - r : see all ROI you've saved
    - c : console out x,y
    - m : insert ROI with specific index

HOW TO INSERT index 'i' Roi
1. Press the key 'm'
2. Input the index in console sou want to insert ROI
3. Drag and drop mouse
4. Successfully it will be completed and saved in 'RoiList'.
  You can check this by pressing key 'c'.

If you want to clear mistaken rectangles, Press the key 'r'.
Then, you can see all ROI you've saved.

'''

import cv2
index = 1
RoiList = []

# ROI값 수동으로 저장(다음 형식으로 값 넣어줄것)
'''
list_tmp = [(50, 108), (106, 159), '01']
RoiList.append(list_tmp)
'''

x1 = 0
y1 = 0
x2 = 0
y2 = 0
cropping = False
str_index = ''
CAM_ID = 1

def capture(camid=CAM_ID):
    cam = cv2.VideoCapture(0)
    # cam = cv2.VideoCapture(1)
    if cam.isOpened() == False:
        print('cant open the cam (%d)' % camid)
        return None
    ret, frame = cam.read()
    if frame is None:
        print('frame is not exist')
        return None
    cv2.imwrite('ParkingLot.jpg', frame, params=[cv2.IMWRITE_PNG_COMPRESSION, 0])
    cam.release()


def click_and_crop(event, x, y, flags, param):
    # refPt와 cropping 변수를 global로 만듭니다.
    global RoiList, cropping
    global x1, x2, y1, y2
    global index

    # 왼쪽 마우스가 클릭되면 (x, y) 좌표 기록을 시작하고
    # cropping = True로 만들어 줍니다.
    if event == cv2.EVENT_LBUTTONDOWN:
        x1 = x
        y1 = y
        cropping = True

    # 왼쪽 마우스 버튼이 놓여지면 (x, y) 좌표 기록을 하고 cropping 작업을 끝냅니다.
    # 이 때 crop한 영역을 보여줍니다.
    elif event == cv2.EVENT_LBUTTONUP:
        x2 = x
        y2 = y

        if index < 10:
            str_index = '0' + str(index)
        elif index >= 10:
            str_index = str(index)

        list_tmp = [(x1, y1), (x2, y2), str_index]
        # index += 1
        # RoiList.append(tmpList)
        #같은 번호의 인덱스로 들어오면 지우고 다시 체크되는걸로
        if RoiList[index][2] == index:
            del RoiList[index]
        if index != 0:
            RoiList.insert(index, list_tmp)

        '''for R in RoiList:
            if R[0] != (0,0):
                print(R)'''

        cropping = False

        # ROI 사각형을 이미지에 그립니다.
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 1)
        cv2.imshow("image", image)


tup_00 = (0,0)
plusTenFlag = 0
capture()
index = 0
# initiaize RoiList
for i in range(0,31):
    RoiList.append([(0, 0), (0, 0), i])

# 이미지를 load 합니다.
image = cv2.imread("ParkingLot.jpg")
# 원본 이미지를 clone 하여 복사해 둡니다.
clone = image.copy()
# 새 윈도우 창을 만들고 그 윈도우 창에 click_and_crop 함수를 세팅해 줍니다.
cv2.namedWindow("image")
cv2.setMouseCallback("image", click_and_crop)


while True:
    #index = 0
    # 이미지를 출력하고 key 입력을 기다립니다.
    cv2.imshow("image", image)
    key = cv2.waitKey(1) & 0xFF
    # 만약 r이 입력되면, crop 할 영열을 리셋합니다.
    if key == ord("r"):
        image = clone.copy()
        for i in range(30):
            cv2.rectangle(image, RoiList[i][0], RoiList[i][1], (0, 255, 0), 1)


    # Console Out x,y
    elif key == ord("c"):
        for R in RoiList:
            if R[0] != tup_00:
                print(R)

    elif key == ord("m"):
        number = int(input("Input number : "))
        index = number
        del RoiList[number]

    # 만약 q가 입력되면 작업을 끝냅니다.
    elif key == ord("q"):
        break

    elif key == ord("1"):
        if plusTenFlag == 0:
            index = 1
            del RoiList[index]
        elif plusTenFlag == 1:
            index = 11
            del RoiList[index]
        elif plusTenFlag == 2:
            index = 21
            del RoiList[index]
    elif key == ord("2"):
        if plusTenFlag == 0:
            index = 2
            del RoiList[index]
        elif plusTenFlag == 1:
            index = 12
            del RoiList[index]
        elif plusTenFlag == 2:
            index = 22
            del RoiList[index]
    elif key == ord("3"):
        if plusTenFlag == 0:
            index = 3
            del RoiList[index]
        elif plusTenFlag == 1:
            index = 13
            del RoiList[index]
        elif plusTenFlag == 2:
            index = 23
            del RoiList[index]
    elif key == ord("4"):
        if plusTenFlag == 0:
            index = 4
            del RoiList[index]
        elif plusTenFlag == 1:
            index = 14
            del RoiList[index]
        elif plusTenFlag == 2:
            index = 24
            del RoiList[index]
    elif key == ord("5"):
        if plusTenFlag == 0:
            index = 5
            del RoiList[index]
        elif plusTenFlag == 1:
            index = 15
            del RoiList[index]
        elif plusTenFlag == 2:
            index = 25
            del RoiList[index]
    elif key == ord("6"):
        if plusTenFlag == 0:
            index = 6
            del RoiList[index]
        elif plusTenFlag == 1:
            index = 16
            del RoiList[index]
        elif plusTenFlag == 2:
            index = 26
            del RoiList[index]
    elif key == ord("7"):
        if plusTenFlag == 0:
            index = 7
            del RoiList[index]
        elif plusTenFlag == 1:
            index = 17
            del RoiList[index]
        elif plusTenFlag == 2:
            index = 27
            del RoiList[index]
    elif key == ord("8"):
        if plusTenFlag == 0:
            index = 8
            del RoiList[index]
        elif plusTenFlag == 1:
            index = 18
            del RoiList[index]
        elif plusTenFlag == 2:
            index = 28
            del RoiList[index]
    elif key == ord("9"):
        if plusTenFlag == 0:
            index = 9
            del RoiList[index]
        elif plusTenFlag == 1:
            index = 19
            del RoiList[index]
        elif plusTenFlag == 2:
            index = 29
            del RoiList[index]
    elif key == ord("0"):
        if plusTenFlag == 0:
            index = 10
            del RoiList[index]
        elif plusTenFlag == 1:
            index = 20
            del RoiList[index]
        elif plusTenFlag == 2:
            index = 30
            del RoiList[index]
        plusTenFlag += 1
        del RoiList[index]

# 모든 window를 종료합니다.
cv2.destroyAllWindows()
