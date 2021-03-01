# 만약 나중에 같은 packet_ROI를 연속으로 여러번 보내는게 문제가 된다면
# ActiveROI를 큐로 만들고 각각의 원소를 크기순으로 정렬을 한 다음에
# packet_ROI에 담고서 보내고
# pre_packet_ROI에 저장한 다음에
# 다음번에 패키지 보낼때 pre_packet_ROI != packet_ROI인지 확인하고 보내는 작업을 해야한다.

import cv2
import numpy as np
import time
import SetROI


#from File_IO import File_write_yolo, File_read_yolo

diryolo = '/home/chun/PycharmProjects/KIRC_Master/File_io/YtoM'

class kirc_yolo:
    def __init__(self):
        # Loading camera
        self.drc = '/home/chun/PycharmProjects/KIRC_Master/File_io/YtoM'

        self.cnt = 0
        self.net = cv2.dnn.readNet("yolov3-tiny.weights", "yolov3-tiny.cfg")
        self.classes = []
        with open("coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]

        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))
        self.cap = cv2.VideoCapture(0)           # 웹캠 번호가 0일수도, 1일수도 그 이상일 수도 있다.
        self.font = cv2.FONT_HERSHEY_PLAIN
        self.starting_time = time.time()
        self.frame_id = 0

        self.ActiveRoi = ''
        self.Packet_ROI = ''
        self.pre_Packet_ROI = ''
        self.ls_ROI = SetROI.RoiList


    def isInRoi(self, x, y):
        #roi가 x,y 중심좌표를 포함하고 있는지 판단.
        for R in self.ls_ROI:
            if R[0][0] < x < R[1][0] and R[0][1] < y < R[1][1]:
                self.ActiveRoi += R[2]
                break

    def RoiPackaging(self):
        while len(self.ActiveRoi) < 8:
            self.ActiveRoi += '00'

        self.Packet_ROI = self.ActiveRoi
        self.ActiveRoi = ''

    def main_work(self):
        _, frame = self.cap.read()
        self.frame_id += 1
        height, width, channels = frame.shape

        # Detecting objects
        # 320*320 #416*416 #609*609 <=== 정확도 조절
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (320, 320), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)
        # Showing informations on the screen
        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            # 객체 판별
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    # Object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)

                    # 여기서 해당 center_x, center_y 가 30개의 roi 에 하나라도 걸리는지 확인한다.
                    # 해당 roi에 걸리는게 있으면 ActiveRoi에 쌓임
                    self.isInRoi(center_x, center_y)

                    # print(class_id, center_x , center_y)
                    w = int(detection[2] * width)  # width of object
                    h = int(detection[3] * height)
                    # Rectangle coordinates
                    x = int(center_x - w / 2)  # the starting X position of detected object
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))  # percentage
                    class_ids.append(class_id)  # the name of detected object
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.4, 0.3)  # NMS 알고리즘(한 물체를 두개의 물체로 인식하면 이거 조정하면 돼)

        # Packet_ROI를 생성하고 8자리 string 으로 맞춤
        self.RoiPackaging()

        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                confidence = confidences[i]
                color = self.colors[class_ids[i]]
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                cv2.rectangle(frame, (x, y), (x + w, y + 30), color, -1)
                cv2.putText(frame, label + " " + str(round(confidence, 2)), (x, y + 30), self.font, 3, (255, 255, 255),
                            3)

        elapsed_time = time.time() - self.starting_time
        fps = self.frame_id / elapsed_time
        cv2.putText(frame, "FPS: " + str(round(fps, 2)), (10, 50), self.font, 3, (0, 0, 0), 3)

        #ROI 그려주기
        for r in self.ls_ROI:
            cv2.rectangle(frame, r[0], r[1], (0, 255, 0), 1)

        '''if self.Packet_ROI != '00000000' :
            File_in_yolo(self.drc,self.Packet_ROI)'''
            #print(self.Packet_ROI + '를 작성하였습니다.')

        cv2.imshow("Image", frame)


if __name__ == '__main__':
    kirc = kirc_yolo()
    while True:
        kirc.main_work()

        #여기에다가 master 넣어서 합쳐버릴까??


        key = cv2.waitKey(1)
        if key == ord('q'):
            break




