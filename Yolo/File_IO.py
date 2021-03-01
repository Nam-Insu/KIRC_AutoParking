def File_write_gui(drc, data):
    #파이썬에서 외부 txt 파일에 값을 적을 때 쓰는 코드
    f = open(drc, 'w') #txt 파일 열기 'w'는 쓰기 전용으로 열었다는
    #for x in range(1,11):
    #    data = "%d ," % x
    #    f.write(data)
    f.write(data) #txt 파일에 문자열 적기
    f.close() #txt 파일 닫기
    #파일을 열고 닫을 때 까지 쓴 문자열만 저장 되면서 이전에 있던 내용을 다 날아감

def File_read_gui(drc):
    #txt 파일의 문자열을 가져오는 코드
    #파일입출력 형식은 열고 닫는 형식이 좋을거 같습니다
    f = open(drc, 'r') #txt 파일을 열기, 'r'은 읽기 전용
    data = (f.read())  #변수 num에 txt 파일의 문자열을 담는다
    return data


    '''
    n1 = int(int(data)/100)
    n2 = int(data)%100
    print(n1)
    print(n2)
    '''
    f.close()#txt 파일을 닫는다


def File_write_yolo(drc, data):
    #파이썬에서 외부 txt 파일에 값을 적을 때 쓰는 코드
    f = open(drc, 'w') #txt 파일 열기 'w'는 쓰기 전용으로 열었다는
    #for x in range(1,11):
    #    data = "%d ," % x
    #    f.write(data)
    f.write(data) #txt 파일에 문자열 적기
    f.close() #txt 파일 닫기

    #파일을 열고 닫을 때 까지 쓴 문자열만 저장 되면서 이전에 있던 내용을 다 날아감

def File_read_yolo(drc):
    #txt 파일의 문자열을 가져오는 코드
    #파일입출력 형식은 열고 닫는 형식이 좋을거 같습니다
    f = open(drc, 'r') #txt 파일을 열기, 'r'은 읽기 전용
    data = (f.read())  #변수 num에 txt 파일의 문자열을 담는다
    #print(data)
    #print(type(data))

    f.close()#txt 파일을 닫는다


if __name__ == "__main__":
    diretory = "/home/chun/Desktop/KIRC_GYM/GtoM"
    packet = "c12a1rp2ep1p2p3p4p5p6p7o7io4i4ifidifjrfirjfirjfirjfijrfi"
    File_in_gui(diretory,packet)

    File_out_gui(diretory)
