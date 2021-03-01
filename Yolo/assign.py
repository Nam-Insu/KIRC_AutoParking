import random
from File_IO import File_in_gui

list_parkingArea1 = [1,2,3,4,5,6]
list_parkingArea2 = [7,8,9,11,12,13]

# parkingArea 리스트는 주차 가능한 공간들의 모임.

# 일단 차 넣어놓기 ㅋㅋㅋㅋ
dic = {10:'1035'}
# dic = {}
Flag_ForceOrder = False   #True 이면 강제명령임
areaNumber = 0
areaAssignFlag = 2

# 차 빠지면 리스트에서 해당번호 복구
# 사전[areNUmer] = 0
# g -> m 주차공간 정보 전송
def assign_pop(value):
    if value not in dic.values():
        print(dic)
    else:
        # k = 칸번호, v = 차번호
        for k, v in dic.items():  # mydict에 아이템을 하나씩 접근해서, key, value를 각각 k, v에 저.장.
            if v == value:
                #k 가 10의 자리인지 1의자리인지 따져야함
                if k >= 10:
                    File_in_gui("/home/chun/PycharmProjects/KIRC_yolo/GtoM", str(k) + 'o')
                elif k < 10:
                    File_in_gui("/home/chun/PycharmProjects/KIRC_yolo/GtoM", '0' + str(k) + 'o')

                if k > 6:
                    list_parkingArea2.append(k)
                else:
                    list_parkingArea1.append(k)

                print(str(k) + '칸에 있던' + dic[k] + '가 출차 됩니다.')
                del dic[k]
                break

        for k, v in dic.items():
            print(str(k) + ' 번에' + str(v) + '가 있습니다.')



def assign_autopush(value):       # 입력값은 차량번호 4자리. 16개 자리 체크 후 할당.  # 바리케이트에서 번호를 인식하여 차량을 등록하는 함수.j
                                  # value는 string 으로 입력받는게 좋음. 예를들어 '3450' 이런식으로.
    global areaAssignFlag
    global Flag_ForceOrder
    global areaNumber
    if value[0] == '7':
        Flag_ForceOrder = True
        areaNumber = 8
        areaAssignFlag = 2
        dic[areaNumber] = value
        list_parkingArea2.remove(areaNumber)
    elif value[0] == '1' and value[1] == '0':
        for area in list_parkingArea2:
            if area == 10:
                Flag_ForceOrder = True
                areaNumber = 10
                areaAssignFlag = 2
                dic[areaNumber] = value
                list_parkingArea2.remove(areaNumber)
    elif value[0] == '2':
        Flag_ForceOrder = True
        areaNumber = 2
        areaAssignFlag = 1
        dic[areaNumber] = value
        list_parkingArea1.remove(areaNumber)
    elif value[0] == '4':
        Flag_ForceOrder = True
        areaNumber = 4
        areaAssignFlag = 1
        dic[areaNumber] = value
        list_parkingArea1.remove(areaNumber)

    if areaAssignFlag == 2:
        if Flag_ForceOrder is False:
            while True:
                areaNumber= random.choice(list_parkingArea2)
                if areaNumber != 8 and areaNumber != 10:
                    break
            dic[areaNumber] = value
            list_parkingArea2.remove(areaNumber)

        Flag_ForceOrder = False
        # g -> m 으로 areaNumber 전송
        #숫자를 무조건 2자리로 맞추기 위한작업
        if areaNumber >= 10:
            str_areaNumber = str(areaNumber)
        elif areaNumber < 10:
            str_areaNumber = '0' + str(areaNumber)
        File_in_gui("/home/chun/PycharmProjects/KIRC_yolo/GtoM", str_areaNumber + 'i')

        areaAssignFlag = 1

        print(str(areaNumber) + ' 번 주차공간에 ' + str(value) + ' 차량 등록됨')

        # for k, v in dic.items():
        #     print(str(k) + ' 번에' + str(v) + '가 있습니다.')

    elif areaAssignFlag == 1:
        if Flag_ForceOrder is False:
            while True:
                areaNumber = random.choice(list_parkingArea1)
                if areaNumber != 2 and areaNumber != 4:
                    break
            list_parkingArea1.remove(areaNumber)
            dic[areaNumber] = value

        Flag_ForceOrder = False
        # g -> m 으로 areaNumber 전송
        # 숫자를 무조건 2자리로 맞추기 위한작업
        str_areaNumber = '0' + str(areaNumber)
        File_in_gui("/home/chun/PycharmProjects/KIRC_yolo/GtoM", str_areaNumber + 'i')

        # 디버깅용
        print(str(areaNumber) + ' 번 주차공간에 ' + str(value) + ' 차량 등록됨')
        # for k, v in dic.items():
        #     print(str(k) + ' 번에' + str(v) + '가 있습니다.')

        areaAssignFlag = 2

