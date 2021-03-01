/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define RX_BUFFER_SIZE 4
#define THRESHOLD_RATIO 0.4

/*  Lifi를 위한 변수 */
#define DELAY 6   
#define START_DELAY 0

#define FS 100  //forward speed  전진 속도
#define BS 20  //backward speed  후진 속도
#define FW 0 //모터 앞으로
#define BW 1 //모터 뒤로

///////////////////////////////////////////////////////////////////////////////////////
//             LED COUNT
///////////////////////////////////////////////////////////////////////////////////////
#define LED_COUNT 300 //led 몇번 카운트 이상이면 판단할지

///////////////////////////////////////////////////////////////////////////////////////
//PD제어값
///////////////////////////////////////////////////////////////////////////////////////
volatile double Front_KpX = 0.3;
volatile double Front_KdX = 5;

///////////////////////////////////////////////////////////////////////////////////////
//조정해보면서 확인하는 변수.. 나중엔 다시 바꿔야함.
///////////////////////////////////////////////////////////////////////////////////////
volatile int x;
volatile int y;

volatile int Time_Left_Turn = 6500;
volatile int Time_Right_Turn = 4700;
volatile int Time_Right_Out_Turn = 7300;
volatile int Angle_Left_Max = 300;
volatile int Angle_Right_Max = 1000;

volatile int Time_Left_Park = 5000;
volatile int Time_Right_Park = 3200;
volatile int Time_Right_Park_Out = 7500;
volatile int Time_Right_Park_Out_Turn = 7500;
volatile int Angle_Left_Half = 500;
volatile int Angle_Right_Half = 880;
volatile int ServoCenter = 700;           //서보모터 정중앙값
volatile double differential_ratio = 0.755;            //differential ratio  차동기어비!!
volatile int L=0;
volatile int R=0;
volatile int LEFT_LIMIT=610;
volatile int RIGHT_LIMIT=790;   //1500  750  700
volatile int Time_Delay=2500;
volatile int nFilterLIFI;

int threshold = 10000;             //To determine 0s and 1s
uint8_t moveToNextBit = 0;      
const int precision = 1001;     
int sensorValue=0;
char fname[20] = "";
uint8_t data;
volatile int chk=0;
int led_cnt[8];
volatile int lifi_order = 0;

int pre_mode;

//디버깅용
int g_Cnt;
volatile int pre_lifi_value = 0;
int arLifiValue[100] = {0};
int lifi_test = 0;

volatile int ADC_Max = 0;
volatile int ADC_Min = 4000;


volatile int Lifi_Value=0;
volatile int blue_button_cnt=0;
volatile int Pin_State;


volatile int Real_Right=0;


volatile int check_timer9 = 0;                                                                                                           //check를 위한 변수
volatile int check_mode0 = 0;
volatile int check_mode00 = 0;
volatile int check_while = 0;
volatile int start_flag = 0;



volatile int front_cnt=0;
volatile int cnt = 0;
volatile int back_cnt = 0;
volatile int delay_cnt=0;
volatile int turn_cnt=0;
volatile int mode = 8;                         // 주행 모드 설정
volatile int first_mode = 0;                  //버튼에 따라 모드를 8번에서 0번 왔다갔다리 하는거
volatile int PSD_temp = 4;                  // 어떤 PSD에 걸렸는지 임시로 담아두기 위한 변수.   4는 정상상태
volatile int motor_dir = FW;                //DC모터 디렉션 설정
volatile int DCMotor_CCR;       //0~100 사이의 값
volatile int ServoMotor_CCR;   //750~850 사이의 값



//제어기를 위한 변수들
volatile int pre_errX;
volatile int now_X;
volatile int pre_now_X = 0;
volatile int move_X = 800;  //이동할 서보모터값. 초기값 = 중앙값
volatile int errX;
volatile int sum_err_X;
volatile int PD_result =800; //이동할 서보모터값. 초기값 = 중앙값

//전조등 위한 플래그
volatile int led_off_flag=0; // 초기 값 0


// 영상처리 결과를 DMA로 받기 위한 배열과 변수
uint8_t tx_buffer[] = "f";
uint8_t tx_buffer2[] = "b";
uint8_t OpenCV_Value[RX_BUFFER_SIZE];
uint8_t tx_len;

double dAlpha = 0.6;

//PSD ADC값  받기위한 배열   index 0~3까지는 PSD, 
uint16_t ADC_Value[5];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Lifi_Init();
uint8_t getValue();
uint8_t convertToDecimal(int arr[8]);

void MotorCCR(int Left, int Right){
  TIM4->CCR1 = Left;
  Real_Right = (int)Right*differential_ratio;
  TIM5->CCR1 = Real_Right;
}
void SendToRasp(char order){
  if (order == 'f'){
    tx_len = sizeof(tx_buffer)-1;
    HAL_UART_Transmit(&huart6,tx_buffer,tx_len,HAL_MAX_DELAY);
  }else if(order == 'b'){
    tx_len = sizeof(tx_buffer2)-1;
    HAL_UART_Transmit(&huart6,tx_buffer2,tx_len,HAL_MAX_DELAY);
  }
}
int OpenCV_ParsingX(){  //receive buffer에 담긴 X좌표 파싱하는 함수                                                                                      
  int data = (OpenCV_Value[0]-48)*100 +  (OpenCV_Value[1]-48)*10 +  (OpenCV_Value[2]-48);
  return data;
}

int Front_PD(int X){  //전방 PD제어 함수
  errX = X - 320;
  sum_err_X = errX * Front_KpX + Front_KdX*(errX-pre_errX);
  pre_errX = errX;
  move_X = move_X + sum_err_X;
  if(move_X > RIGHT_LIMIT)
  {
    move_X = RIGHT_LIMIT;
  }else if(move_X < LEFT_LIMIT)
  {
    move_X = LEFT_LIMIT;
  }
  return move_X;
}
void Lifi_1_Count(int data){
  if(data == 1){
    led_cnt[1]++;
  }
}
void Lifi_1234_Count(int data){
  switch( data ){
  case 1:
    led_cnt[1]++;
    break;
  case 2:
    led_cnt[2]++;
    break;
  case 3:
    led_cnt[3]++;
    break;
  case 4:
    led_cnt[4]++;
    break;
  }
}
void Lifi_56_Count(int data){
  switch( data ){
  case 5:
    led_cnt[5]++;
    break;
  case 6:
    led_cnt[6]++;
    break;
  }
}
void Lifi_1234_Count_Init(){
  for(int i=1;i<5;i++){
    led_cnt[i]=0;
  }
}




/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if (GPIO_Pin == GPIO_PIN_13){
    
    blue_button_cnt++;
    
    if(first_mode == 0){
      mode = 0;
      first_mode = 1;
    }else if(first_mode == 1){
      mode = 8;
      first_mode = 0;
    }
  }
}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //Timer interrupt Routine
{
  
  if(htim == &htim9)  // 1ms 마다 실행된다. 0.001초마다.
  {
    nFilterLIFI = (int)(dAlpha * ADC_Value[4]) + (int)((1-dAlpha) * nFilterLIFI);
    check_timer9++;
    if(check_timer9>1000) check_timer9=1;
    PSD_temp = 4;                       // 이상 PSD 변수 초기화
        
    if(ADC_Value[0]>2200){
      PSD_temp = 0;
    }
    
    switch(PSD_temp){
      
    case 0:   // 전방 걸린 상황  -  바리케이트는 모터 멈추기, 이후부터는 뒤로 후진
      /*if(start_flag == 0){
      MotorCCR(0,0);
      
    }else if(start_flag == 1){
      motor_dir = BW;
      TIM3->CCR1 = ServoCenter-100;
      
      MotorCCR(FS,FS);            
    }*/
      break;
      
    case 4:   // PSD 정상상태
      
      switch(mode){
        
        
        /////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////
        /////////////////       0번 모드 - 전진 주행         ////////////////
        ////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////
      case 0: // 전진주행
        start_flag = 1;
        
        // 라즈베리에 앞에 카메라 키고 값 보내라는 명령주기
        if(check_timer9%50==1) SendToRasp('f');            // 0.5초마다 Rsp에 'f'를 보냄
        
        // 앞에 카메라 영상처리 결과 X좌표 받아오기
        pre_now_X = now_X;
        now_X = OpenCV_ParsingX();
        if(pre_now_X != now_X){
          PD_result = Front_PD(now_X);
        }else if(pre_now_X == now_X){
          PD_result = PD_result;
        }
        //DC모터변수에 앞으로전진 명령
        motor_dir = FW;                                                                                    // DC모터 전진? 
        
        //set Motor
        //TIM3->CCR1 = PD_result;   // 전방 PD제어값 계산 후 모터변수에 전달
        TIM3->CCR1 = PD_result;
        MotorCCR(FS,FS);
        
        
               
        ////////////////////////////////////////////////////////////////////
        Lifi_1234_Count(Lifi_Value);                               // Lifi카운팅
        
        for(int i = 1; i<5; i++){
          if(led_cnt[i]>=LED_COUNT){    // 300count
            lifi_order = i;
            lifi_test = i;
            break;
          }
        }
        
        //디버깅용
        if (pre_lifi_value != lifi_test)
        {
          arLifiValue[g_Cnt++] = lifi_test;
        }
        pre_lifi_value = lifi_test;
        
        // 0번 주행모드에서 lifi 값중 1~4를 받았을 때 모드변경
        switch(lifi_order){
        case 1:  //좌회전
          mode = 1;
          lifi_order = 0;
          pre_errX = 0;
          move_X = 0;
          Lifi_1234_Count_Init();
          break;
        case 2:  //우회전
          mode = 2;
          lifi_order = 0;
          pre_errX = 0;
          move_X = 0;
          Lifi_1234_Count_Init();
          break;
        case 3:  //왼쪽뒤
          mode = 3;
          lifi_order = 0;
          pre_errX = 0;
          move_X = 0;
          Lifi_1234_Count_Init();
          break;
        case 4:  //오른쪽뒤
          mode = 4;
          lifi_order = 0;
          pre_errX = 0;
          move_X = 0;
          Lifi_1234_Count_Init();
          break;
        }
        
        
        break;
        
        /////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////
        /////////////////       1번 모드 - 좌회전 주행       ///////////////
        ////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////
      case 1: // 좌회전 주행  -> cnt는 1ms마다 증가한다.
        
        motor_dir = FW;                                                                                // DC모터 전진? 
        //set Motor
        if(pre_mode!=1)
        {
          cnt=0;
        }
        pre_mode=1;
        MotorCCR(70,100);
        
        cnt++;
        if(cnt < Time_Left_Turn ){ 
          TIM3->CCR1 = Angle_Left_Max; 
        }else if( cnt==Time_Left_Turn){
          mode = 0;
          cnt = 0;
        }
        
        break;
        
        /////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////
        /////////////////       2번 모드 - 우회전 주행       ///////////////
        ////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////
      case 2:  //우회전 주행  -> cnt는 40ms마다 증가한다. 0.04초
        
        motor_dir = FW;                                                                                    // DC모터 전진? 
        //set Motor
        if(pre_mode!=2)
        {
          cnt=0;
        }
        pre_mode=2;
        MotorCCR(100,80);
        
        cnt++;
        if(cnt < Time_Right_Turn ){ 
          TIM3->CCR1 = Angle_Right_Max; 
        }else if( cnt==Time_Right_Turn){
          mode = 0;
          cnt = 0;
        }
        
        
        break;
        
        
        /////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////
        /////////////////        3번 모드 - 왼쪽 뒤로 주차      ////////////
        ////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////
      case 3:
        
        motor_dir = BW;
        //SendToRasp('b');   //원래같으면 켜놨어야함.
        if(pre_mode!=3)
        {
          cnt=0;
        }
        pre_mode=3;
        cnt++;
        if(cnt < Time_Left_Park ){ 
          TIM3->CCR1 = Angle_Left_Half; 
          MotorCCR(70,100);
        }else if( cnt>=Time_Left_Park){
          back_cnt++;
          TIM3->CCR1 = ServoCenter; 
          MotorCCR(70,70);
        }
        
        //////////////////////////// LIFI 부분 /////////////////////////////
        
        Lifi_1_Count(Lifi_Value);                               // Lifi 1 카운팅
        if(led_cnt[1] >= LED_COUNT){
          mode = 7;
          pre_errX = 0;
          move_X = 0;
          led_cnt[1]=0;
          cnt = 0;
        }
        
        break;
        
        
        
        /////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////
        /////////////////     4번 모드 - 오른쪽 뒤로 주차         //////////
        ////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////
      case 4: 
        
        motor_dir = BW;
        //SendToRasp('b');   //원래같으면 켜놨어야함.
        if(pre_mode!=4)
        {
          cnt=0;
        }
        
        pre_mode=4;
        cnt++;
        if(cnt < Time_Right_Park ){ 
          TIM3->CCR1 = 880; 
          MotorCCR(100,87);
        }else if( cnt>=Time_Right_Park){
          back_cnt++;
          TIM3->CCR1 = 700;   ////////////////////////////////////////////////////////대칭
          MotorCCR(100,100);
        }
        
        //////////////////////////// LIFI 부분 /////////////////////////////
        
        Lifi_1_Count(Lifi_Value);                               // Lifi 1 카운팅
        if(led_cnt[1] >= LED_COUNT){
          mode = 7;
          pre_errX = 0;
          move_X = 0;
          led_cnt[1]=0;
          cnt = 0;
        }
        
        
        
        break;
        
        
        /////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////
        /////////////////     5번 모드 - 좌측으로 나갈때       ////////////
        ////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////
      case 5:   // 좌측으로 나갈때
        motor_dir = FW;
        if(pre_mode!=5)
        {
          cnt=0;
        }
        pre_mode=5;
        if(back_cnt>0){
          back_cnt--;
          //TIM3->CCR1 = ServoCenter; 
          MotorCCR(70,70);
        }else if(back_cnt==0){
          front_cnt++;
          if(front_cnt<Time_Left_Park){
            TIM3->CCR1 = Angle_Left_Half; 
            MotorCCR(70,100);
          }else if( front_cnt>=Time_Left_Park){
            mode = 1;
            front_cnt=0;
            
          }
          
        }
        
        break;
        
        /////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////
        /////////////////     6번 모드 - 우측으로 나갈때       //////////
        ////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////
      case 6:   // 우측으로 나갈때
        
        motor_dir = FW;  
        if(pre_mode!=6)
        {
          cnt=0;
        }
        pre_mode=6;
        if(back_cnt>0){
          back_cnt--;
          TIM3->CCR1 = 700; ////////////////////////////////////////////////////////////대칭
          MotorCCR(100,100);
        }else if(back_cnt==0){
          front_cnt++;
          if(front_cnt<Time_Right_Park){
            TIM3->CCR1 = Angle_Right_Half; 
            MotorCCR(100,87);
          }
          else if( front_cnt>=Time_Right_Park){
            mode = 0;
            front_cnt = 0;
            cnt = 0;
          }
        }
        /*
        if(back_cnt>0){
        back_cnt--;
        //TIM3->CCR1 = 710; ////////////////////////////////////////////////////////////대칭
        MotorCCR(80,80);
      }else if(back_cnt==0){
        front_cnt++;
        if(front_cnt<Time_Right_Park){
        TIM3->CCR1 = Angle_Right_Half; 
        MotorCCR(100,80);
      }else if( front_cnt>=Time_Right_Park){
        if(delay_cnt<Time_Delay){
        TIM3->CCR1 =750;
        MotorCCR(100,100);
        delay_cnt++;}
        else if(delay_cnt>=Time_Delay){
        mode = 2;
        front_cnt=0;
        cnt=0;
        delay_cnt=0;
      }
      }
        
      }*/
        
        break;
        
        /////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////
        /////////////////     7번 모드 - 주차 상태               ////////////
        ////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////
      case 7:  
        if(pre_mode!=7)
        {
          cnt=0;
        }
        pre_mode=7;
        chk++;
        led_off_flag = 1;
        //set Motor
        MotorCCR(0,0);
        
        //////////////////////////// LIFI 부분 /////////////////////////////
        if(chk>2000){
          led_cnt[5]=0;
          led_cnt[6]=0;
          chk=0;
        }
        Lifi_56_Count(Lifi_Value);                               // Lifi카운팅
        for(int i = 5; i<7; i++){
          if(led_cnt[i]>=LED_COUNT){
            lifi_order = i;
            break;
          }
        }
        
        
        switch(lifi_order){
        case 5:  //좌로 앞으로 나가
          mode = 5;
          led_cnt[5] = 0;
          led_cnt[6] = 0;
          led_off_flag = 0;
          lifi_order = 0;
          break;
        case 6:  //우로 앞으로 나가
          mode = 6;
          led_cnt[5] = 0;
          led_cnt[6] = 0;
          led_off_flag = 0;
          lifi_order = 0;
          break;
        }
        
        
        break;
        
        
        
        
        /////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////
        /////////////////     8번 모드 - 정규화 하는 작업     ////////////
        ////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////
      case 8:
        pre_mode=8;
        MotorCCR(L,R);
        TIM3->CCR1 = ServoMotor_CCR;
        cnt=0;
        front_cnt = 0;
        back_cnt = 0;
        
        
        
        if(ADC_Min>ADC_Value[4]){
          ADC_Min = ADC_Value[4];
        }
        if(ADC_Max<ADC_Value[4]){
          ADC_Max = ADC_Value[4];
        }
        
        threshold =  ADC_Min + (( ADC_Max - ADC_Min ) * THRESHOLD_RATIO);
        //threshold = ( ADC_Min + ADC_Max ) * THRESHOLD_RATIO;
        
        break;
        
        /////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////
        /////////////////     9번 모드 - 후진 모드   ////////////
        ////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////
      case 9:
        
        motor_dir = BW;
        
        if(pre_mode!=9)
        {
          cnt=0;
        }
        pre_mode=9;
        TIM3->CCR1 = 700;
        MotorCCR(100,100);
        
        break;
        
      }
    }
    
    
    
    
  }
  
}

void HAL_UART_RxCpltCallBack(UART_HandleTypeDef *huart)
{
  
  
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM9_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Value, 5);   // PSD ADC값 DMA로 받기
  Lifi_Init();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  //3번 TIMER는 PWM으로 ch1번 사용
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  //3번 TIMER는 PWM으로 ch1번 사용
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);  //5번 TIMER는 PWM으로 ch1번 사용
  HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_1);  // 모터 돌리려면 뭔가 이것도 해줘야함
  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_1);  // 모터 돌리려면 뭔가 이것도 해줘야함
  HAL_TIMEx_PWMN_Start(&htim5, TIM_CHANNEL_1); // 모터 돌리려면 뭔가 이것도 해줘야함
  HAL_UART_Receive_DMA(&huart6, OpenCV_Value, RX_BUFFER_SIZE); //USART6 RX DMA 활성화
  
  HAL_TIM_Base_Start_IT(&htim9);  //Timer 9 Interrupt 활성화 0.001초 =  1ms
  
  
  
  
  
  
  
  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);  // 시작
    Pin_State = HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin);
    
    Lifi_Value=getValue()-48;
    
    check_while++;
    // 차량 전조등, 후미등
    if(led_off_flag==0){  // 차량 전조등 후미등 꺼놓은 상태
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);  // PC10 에 LED 4개 물림
    }else if(led_off_flag==1){  // 자량 전조등 후미등 켜놓은 상태
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);  // PC10 에 LED 4개 물림
    }
    
    // DC모터 도는 방향 설정
    if(motor_dir==1){
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);      // 왼쪽 모터 디렉션 주는것
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);  // 왼쪽 모터 디렉션 주는것
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);       // 오른쪽 모터 디렉션 주는것
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   // 오른쪽 모터 디렉션 주는것
    }else if(motor_dir==0){
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);      // 왼쪽 모터 디렉션 주는것
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);  // 왼쪽 모터 디렉션 주는것
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);       // 오른쪽 모터 디렉션 주는것
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);   // 오른쪽 모터 디렉션 주는것
    }
    
    
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);  // 끝
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 168;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 42;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 42;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 100-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim5);

}

/* TIM9 init function */
static void MX_TIM9_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 84;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void Lifi_Init(){
  
  HAL_Delay(50);
  threshold=10000;   
  //  for(int i=0; i<precision;i++){
  //    threshold += ADC_Value[4];
  //  }
  //  
  //  threshold = threshold / precision;
  //  threshold = threshold + 300;
}
uint8_t convertToDecimal(int arr[3])
{
  int j = 3;
  uint8_t n;
  uint8_t t = 1;
  uint8_t rec = 0;
  while(j >= 0)
  {
    if(arr[j] > threshold) // Data = 1;
    {
      n = j;
      while(n != 0)
      {
        t=t*2;
        n--;
      }
      rec = rec + t;
      t = 1;
    }  
    j--;
  }
  return rec;
}
uint8_t getValue()
{
  int d[3]; //array to store incoming 16 bit values
  uint8_t startBitReceived = 0;
  uint8_t cnt;
  while(!startBitReceived){
    
    //만약 nFilterLIFI = 0 으로만 나오면 자료형 문제
    cnt++;
    
    //sensorValue = nFilterLIFI;
    //    sensorValue = ADC_Value[4];
    if(cnt>50) break;
    HAL_Delay(0);       //system hold
    //    nFilterLIFI = (int)(dAlpha * ADC_Value[4]) + (int)((1-dAlpha) * nFilterLIFI);
    if(nFilterLIFI< threshold) //If we detected start bit
    {
      startBitReceived = 1;
    }
    //    if((int)(dAlpha * ADC_Value[4]) + (int)((1-dAlpha) * nFilterLIFI) < threshold) //If we detected start bit
    //    {
    //      nFilterLIFI = (int)(dAlpha * ADC_Value[4]) + (int)((1-dAlpha) * nFilterLIFI);
    //      
    //      startBitReceived = 1;
    //    }
  }
  //Just delay here until we get to the middle of the start bit
  HAL_Delay(START_DELAY);                        //control halfr
  
  //Get bits based on timer0
  for(unsigned long i = 0; i < 3; i++)
  {
    
    while(!moveToNextBit){
      HAL_Delay(DELAY);                    //control
      d[i] = nFilterLIFI;
      
      //      d[i] =(int)(dAlpha * ADC_Value[4]) + (int)((1-dAlpha) * nFilterLIFI);
      //      nFilterLIFI = (int)(dAlpha * ADC_Value[4]) + (int)((1-dAlpha) * nFilterLIFI);
      moveToNextBit = 1;
      
    }  
    
    moveToNextBit = 0;
    
  }
  
  
  
  uint8_t recVal = convertToDecimal(d)+40;
  
  
  //Busy wait on stop bit
  HAL_Delay(DELAY);                       //control
  return recVal;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
