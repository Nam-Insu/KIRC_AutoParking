# KIRC2019 Project
  
## Project : AutoParking System (feat.BARAM)  
  
지도 교수 : 백주훈 교수님  
참여 인원 : 15 정명근, 15 전성호, 17 김민재, 16 강동운, 16 남인수, 18 이혜진
참여 기간 : 2019.7 ~ 2019.10

## Development Background  
미래 차세대 신기술 중 하나인 자율주행 자동차가 가지고 있는  
여러 기술적 과제들 중 실내주차에 관심을 갖게 됨  
실내공간에서의 주차는 GPS를 통한 측위결정을 사용할 수 없기 때문에  
현 상황에서 구현하고자 했을 때 각종 값비싼 센서들의 도입이 필요함  
하지만 본 팀은 기존의 주차장에서  
어떤 자율주행차량에도 적용가능한 자율발렛 주차시스템을 고안함  
  
## Main Feature  
1. 딥러닝 객체인식 (Yolo Algorithm)  
2. Lifi (빛의 깜박임으로 통신)  
3. 차량 충돌방지 알고리즘  
4. User Interface (Application)  
<img src="https://user-images.githubusercontent.com/52377778/86992312-3129a480-c1dc-11ea-86f7-450bf71c74f9.png" width="600" height="200" />  

## Using Language  
1. V-rep Simulation : C++, Lua Script
2. ReinForce : Python
3. DataServer : Python
  
## System Architecture (BEFORE)

  
## System Architecture (AFTER)
<img src="https://user-images.githubusercontent.com/52377778/86992322-37b81c00-c1dc-11ea-9cdf-731fd7c9c73c.png" width="600" height="400" />  
  
## Hardware Architecture
<img src="https://user-images.githubusercontent.com/52377778/86992292-266f0f80-c1dc-11ea-84cd-bb781e421fe9.png" width="600" height="400" />  
<img src="https://user-images.githubusercontent.com/52377778/86992330-3d156680-c1dc-11ea-8ba1-fa65e82e245a.png" width="600" height="400" />  

## Project Sinario
시나리오 1  
<img src="https://user-images.githubusercontent.com/52377778/86992351-469ece80-c1dc-11ea-8e72-9a5e3145150c.png" width="350" height="350" />  
차량이 들어오면 주차장시스템에서 비어있는 주차공간을 할당하고 경로를 생성한다.  

시나리오 2  
<img src="https://user-images.githubusercontent.com/52377778/86992357-4999bf00-c1dc-11ea-9177-911440cfbde4.png" width="350" height="350" />  
주차장내 CCTV를 활용하여 차량의 위치를 트래킹하고 교차로에서 Lifi통신으로 차량을 유도한다.  
  
시나리오 3  
<img src="https://user-images.githubusercontent.com/52377778/86992362-4bfc1900-c1dc-11ea-8baf-1a6a87eb798c.png" width="350" height="350" />  
사용자가 출차를 요청하면 출구까지 Lifi통신으로 차량을 유도한다.  

## Project GIF  






