import cv2

# 영상의 의미지를 연속적으로 캡쳐할 수 있게 하는 class
vidcap = cv2.VideoCapture('hand4.mp4')    # 이 부분 바꿔서 동영상 파일을 가져와서 캡쳐하기가 가능.

count = 0
frame_skip = 0

while (vidcap.isOpened()):

    # read()는 grab()와 retrieve() 두 함수를 한 함수로 불러옴
    # 두 함수를 동시에 불러오는 이유는 프레임이 존재하지 않을 때
    # grab() 함수를 이용하여 return false 혹은 NULL 값을 넘겨 주기 때문
    ret, image = vidcap.read()
    if ret:
        frame_skip += 1
        if(frame_skip % 5 == 0):
            # 캡쳐된 이미지를 저장하는 함수
            cv2.imwrite("hands_rawData/rframe%d.jpg" % count, image)

            print('Saved %d.jpg' % count)
            count += 1
    else:
        break


vidcap.release()
