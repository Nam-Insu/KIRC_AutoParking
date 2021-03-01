class KIRC_function:
    def __init__(self):
        import serial
        from collections import deque
        self.n_parkingNumber = 7
        self.n_parkingArea = 2
        self.c_InOut = 'i'
        self.str_ledOrder_1 = ''
        self.str_ledOrder_2 = ''
        self.str_ledOrder_3 = ''
        self.que_parkingAreaBuf_1 = deque([])
        self.que_parkingAreaBuf_2 = deque([])
        self.que_orderING_1 = deque([])
        self.que_orderING_2 = deque([])
        self.bool_parkingArea_1 = True
        self.bool_parkingArea_2 = True
        self.str_ROI_1 = ''     #실행 명령 큐의 앞부분과 동일한지 판단해야함
        self.str_ROI_2 = ''
        self.str_ROI_3 = ''
        self.str_ROI_4 = ''
        self.RoiList = []
        self.flag_skip = 0
        self.bFlagOrderING = False


    # 몇번주차공간, 몇번구역, in인지 out인지 판단함수
    def whichAreaOrder(self, packetGui):
        # in 인지 Out인지 판단
        tmp = packetGui[0] + packetGui[1]
        InOut = packetGui[2]
        n_order = int(tmp)
        if n_order > 6:
            if InOut == 'i':
                self.n_parkingNumber = n_order
                self.n_parkingArea = 2
                self.c_InOut = 'i'

            elif InOut == 'o':
                self.n_parkingNumber = n_order
                self.n_parkingArea = 2
                self.c_InOut = 'o'

        else:
            if InOut == 'i':
                self.n_parkingNumber = n_order
                self.n_parkingArea = 1
                self.c_InOut = 'i'

            elif InOut == 'o':
                self.n_parkingNumber = n_order
                self.n_parkingArea = 1
                self.c_InOut = 'o'

    # 이 부분 led받는거 전부 다 인덱스 맞추어서 수정해주어야한다.
    def whichLEDneed(self):
        if self.c_InOut == 'i':
            if self.n_parkingNumber == 1:   #만약 1번 주차공간에 들어간다면 ?
                self.str_ledOrder_1 = "142"     #14는 led번호, 2는 회전 방법을 지시.
                self.str_ledOrder_2 = "164"     #22는 led번호, 4는 회전 방법을 지시.
                self.str_ledOrder_3 = "011"
            elif self.n_parkingNumber == 2:
                self.str_ledOrder_1 = "252"
                self.str_ledOrder_2 = "214"
                self.str_ledOrder_3 = "291"
            elif self.n_parkingNumber == 3:
                self.str_ledOrder_1 = "142"
                self.str_ledOrder_2 = "184"
                self.str_ledOrder_3 = "031"
            elif self.n_parkingNumber == 4:
                self.str_ledOrder_1 = "252"
                self.str_ledOrder_2 = "213"
                self.str_ledOrder_3 = "121"
            elif self.n_parkingNumber == 5:
                self.str_ledOrder_1 = "142"
                self.str_ledOrder_2 = "173"
                self.str_ledOrder_3 = "051"
            elif self.n_parkingNumber == 6:
                self.str_ledOrder_1 = "142"
                self.str_ledOrder_2 = "183"
                self.str_ledOrder_3 = "061"
        #####################################
            elif self.n_parkingNumber == 7:
                self.str_ledOrder_1 = "192"
                self.str_ledOrder_2 = "064"
                self.str_ledOrder_3 = "181"
        #####################################
            elif self.n_parkingNumber == 8:
                self.str_ledOrder_1 = "152"
                self.str_ledOrder_2 = "224"
                self.str_ledOrder_3 = "081"
            elif self.n_parkingNumber == 9:
                self.str_ledOrder_1 = "152"
                self.str_ledOrder_2 = "234"
                self.str_ledOrder_3 = "091"
            elif self.n_parkingNumber == 10:
                self.str_ledOrder_1 = "152"
                self.str_ledOrder_2 = "203"
                self.str_ledOrder_3 = "101"
            elif self.n_parkingNumber == 11:
                self.str_ledOrder_1 = "152"
                self.str_ledOrder_2 = "213"
                self.str_ledOrder_3 = "111"
            elif self.n_parkingNumber == 12:
                self.str_ledOrder_1 = "152"
                self.str_ledOrder_2 = "223"
                self.str_ledOrder_3 = "121"
            elif self.n_parkingNumber == 13:
                self.str_ledOrder_1 = "152"
                self.str_ledOrder_2 = "233"
                self.str_ledOrder_3 = "131"


        # 만약 주차장 led 설계 할때 Aisle_1,2의 LED와 Aisle_Ext led가 겹치면
        # 여기서 led 인덱스 조정 해주어야한다.
        elif self.c_InOut == 'o':
            if self.n_parkingNumber == 1:
                self.str_ledOrder_1 = "016"
                self.str_ledOrder_2 = "192"
            elif self.n_parkingNumber == 2:
                self.str_ledOrder_1 = "296"
                self.str_ledOrder_2 = "212" ## 좀 앞에있는 led로 줬음
            elif self.n_parkingNumber == 3:
                self.str_ledOrder_1 = "036"
                self.str_ledOrder_2 = "192"
            elif self.n_parkingNumber == 4:
                self.str_ledOrder_1 = "135"
                self.str_ledOrder_2 = "212"
            elif self.n_parkingNumber == 5:
                self.str_ledOrder_1 = "055"
                self.str_ledOrder_2 = "192"
            elif self.n_parkingNumber == 6:
                self.str_ledOrder_1 = "065"
                self.str_ledOrder_2 = "192"
            elif self.n_parkingNumber == 7:
                self.str_ledOrder_1 = "186"
                self.str_ledOrder_2 = "051"     #출차 led위치가 애매해
            elif self.n_parkingNumber == 8:
                self.str_ledOrder_1 = "086"
                self.str_ledOrder_2 = "241"
            elif self.n_parkingNumber == 9:
                self.str_ledOrder_1 = "096"
                self.str_ledOrder_2 = "241"
            elif self.n_parkingNumber == 10:
                self.str_ledOrder_1 = "045"
                self.str_ledOrder_2 = "051"
            elif self.n_parkingNumber == 11:
                self.str_ledOrder_1 = "115"
                self.str_ledOrder_2 = "241"
            elif self.n_parkingNumber == 12:
                self.str_ledOrder_1 = "125"
                self.str_ledOrder_2 = "241"
            elif self.n_parkingNumber == 13:
                self.str_ledOrder_1 = "135"
                self.str_ledOrder_2 = "241"

    #입차면 3자리 패킷3개 , 출차면 3자리 패킷3개
    def push2parkingAreaBuffer(self):
        if self.c_InOut == 'i':
            if self.n_parkingArea == 1:
                self.que_parkingAreaBuf_1.append(self.str_ledOrder_1)
                self.que_parkingAreaBuf_1.append(self.str_ledOrder_2)
                self.que_parkingAreaBuf_1.append(self.str_ledOrder_3)
            if self.n_parkingArea == 2:
                self.que_parkingAreaBuf_2.append(self.str_ledOrder_1)
                self.que_parkingAreaBuf_2.append(self.str_ledOrder_2)
                self.que_parkingAreaBuf_2.append(self.str_ledOrder_3)

        elif self.c_InOut == 'o':
            if self.n_parkingArea == 1:
                self.que_parkingAreaBuf_1.append(self.str_ledOrder_1)
                self.que_parkingAreaBuf_1.append(self.str_ledOrder_2)
            if self.n_parkingArea == 2:
                self.que_parkingAreaBuf_2.append(self.str_ledOrder_1)
                self.que_parkingAreaBuf_2.append(self.str_ledOrder_2)

    # GUI에서 들어온 차정보를 가지고 led명령화 시켜서 que_buffer에 쌓음
    def queueingBuffer(self,packetGui):
        self.whichAreaOrder(packetGui)
        self.whichLEDneed()
        self.push2parkingAreaBuffer()

    def isArea_enable(self, AreaNumber):
        if AreaNumber == 1:
            # ING로 명령 받을 준비가 되어있는 상태
            if self.que_parkingAreaBuf_1.__len__() > 0 and self.bool_parkingArea_1 is True:
                return True
            else:
                return False

        elif AreaNumber == 2:
            if self.que_parkingAreaBuf_2.__len__() > 0 and self.bool_parkingArea_2 is True:
                return True
            else:
                return False

    ###################################################################
    ##############################수정해야함#############################
    # 가져오는 명령어는 in이면 3개, out이면 2개로 가져와야하네.
    # bool_parkingArea는 orderING가 비어있으면 다시 True로 만들어줘야하는데
    def bufToOrderING(self):
        if self.isArea_enable(1):
            # 명령어는 2개씩 가져온다
            self.que_orderING_1.append(self.que_parkingAreaBuf_1.popleft())
            self.que_orderING_1.append(self.que_parkingAreaBuf_1.popleft())
            if int(self.que_orderING_1[0]) == 192 or int(self.que_orderING_1[0]) == 252:
                self.bFlagOrderING = True
                self.que_orderING_1.append(self.que_parkingAreaBuf_1.popleft())
                #print("ING 버퍼에 3개가 찼을거임")

            self.bool_parkingArea_1 = False

        if self.isArea_enable(2):
            self.que_orderING_2.append(self.que_parkingAreaBuf_2.popleft())
            self.que_orderING_2.append(self.que_parkingAreaBuf_2.popleft())
            if int(self.que_orderING_2[0]) == 192 or int(self.que_orderING_2[0]) == 252:
                self.bFlagOrderING = True
                self.que_orderING_2.append(self.que_parkingAreaBuf_2.popleft())
                #print("ING 버퍼에 3개가 찼을거임")
            self.bool_parkingArea_2 = False


    def whichROIactive(self, packetYolo):
        self.str_ROI_1 = packetYolo[0] + packetYolo[1]
        self.str_ROI_2 = packetYolo[2] + packetYolo[3]
        self.str_ROI_3 = packetYolo[4] + packetYolo[5]
        self.str_ROI_4 = packetYolo[6] + packetYolo[7]

    def Roi_Update(self):
        self.RoiList = []
        self.RoiList.append(self.str_ROI_1)
        self.RoiList.append(self.str_ROI_2)
        self.RoiList.append(self.str_ROI_3)
        self.RoiList.append(self.str_ROI_4)

    def findActiveROI_makeRoiList(self, packetYolo):
        self.whichROIactive(packetYolo)
        # 지금 명령어를 실행중인지 아니면 넣어줄수 있는 상태인지 판단하는 함수
        self.Roi_Update()

    #아직 다 된거 아님
    # 활성화된 roi가 내가 원하는 led 의 위치와 동일한지 확인
    # 동일하면 코어텍스로 명령어를 보내
    # 해당 Area에 대한 명령어가 종료되면 다음 명령어를 받을 준비.
    def isTargetRoi_commandLIFI(self):
        for R in self.RoiList:
            if R == self.que_orderING_1[0]:  # 16R 이런식으로 되어있어서 수정해야함
                #ser.write(bytes(self.que_orderING_1[0], encoding='ascii'))  # 출력방식1
                self.que_orderING_1.popleft()  # 이 명령어를 실행했으면 이제 다시는 실행안할거니까 삭제
                if self.que_orderING_1.__len__() == 0:
                    self.bool_parkingArea_1 = True
                    print("1구역 명령완료")

