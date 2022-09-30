#include <Ethernet.h>

/*
  Mouse.h

  Copyright (c) 2015, Arduino LLC
  Original code (pre-library): Copyright (c) 2011, Peter Barrett

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef USER_DEFINE_h
#define USER_DEFINE_h

#include <Arduino.h>


#ifndef LED_BUILTIN
#define LED_BUILTIN 13  // GPIO for LED Status Control
#endif

#define HW_TIMER_INTERVAL_MS 1000

#define TIMER_INTERVAL_1S 1000L
#define TIMER_INTERVAL_1MS 1L
#define TIMER_INTERVAL_10MS 10L

//CAN BUS ID
#define CAN_ID_ENC_RD       0x183
#define CAN_ID_TORQ_RD      0x410   //TORQ  CAN ID SWITCCH(0001)
#define CAN_ID_TORQ_WR      0x510   //TORQ  CAN ID SWITCCH(0001)
#define CAN_ID_DISP_RSP     0x5FB
#define CAN_ID_DISP_WR      0x67B


#define CAN_ID_ENC_WR       0x000


//Winstar Display CAN ID (0x200x)
#define DISP_ID_SET_TORQ    0x00
#define DISP_ID_SET_ENC     0x01
#define DISP_ID_GAUGE_TORQ  0x02
#define DISP_ID_GAUGE_ENC   0x03
#define DISP_ID_TXT_TORQ    0x04
#define DISP_ID_TXT_ENC     0x05
#define DISP_ID_LED_VIN     0x06
#define DISP_ID_LED_DISP    0x07
#define DISP_ID_LED_TORQ    0x08
#define DISP_ID_LED_ENC     0x09

//ADC Unit Value (2^10 = 1024)
#define ADC_Unit 0.00097751711  //0.00097751711 = 1/1023

//DAC Unit Value, 3.3V / 4096
#define DAC_Unit 4096/3.3

//rad Unit
#define RAD_Unit PI/180.0

//

#define FLASH_T_UNIT_AMP  0x21000    //Amaplitude
#define FLASH_T_UNIT_UT   0x21001    //Unit Time
#define FLASH_T_UNIT_CC   0x21002    //Cycle Count
#define FLASH_T_UNIT_WT   0x21003    //Wait Time


//Low Pass Filter
#define LPF_1ST_FC  10                 //Cutoff Frequency




// Init SAMDUE_ISR_Timer
// Each SAMDUE_ISR_Timer can service 16 different ISR-based timers
ISR_Timer SAMDUE_ISR_Timer;

/* Types used for the tables below */
typedef struct UVAR
{
  char      uFlag_Enc_Rx;
  char      uFlag_Disp_Rx;
  char      uFlag_Torq_Rx;

  char      stat_led;
  char      stat_disp;
  char      stat_enc;
  char      stat_torq;

  char      Data_Sign;

  char      array_enc_hex[8];
  char      array_torq_hex[8];

  uint16_t  uCount;
  uint16_t  uCount_Err_VIN;
  uint16_t  uCount_Err_DISP;
  uint16_t  uCount_Err_ENC;
  uint16_t  uCount_Err_TORQ;
  uint16_t  uCount_1ms;

  uint16_t  uFlag_Enc_Zero_Cmd;
  uint16_t  uFlag_Torq_Zero_Cmd;
  uint16_t  uFlag_Cmd; // 1=ZeroSet(Torq), 2=ZeroSet(Enc)
  uint16_t  uFlag_1ms;
  uint16_t  uFlag_10ms;
  uint16_t  uFlag_100ms;

  uint16_t  uFlag_CanTx;

  uint16_t  uAI_A0;       // variable to store the value coming from the sensor
  uint16_t  uAI_A1;       // variable to store the value coming from the sensor
  uint16_t  uAI_A0_DISP;  // variable to store the value coming from the sensor
  uint16_t  uAI_A1_DISP;  // variable to store the value coming from the sensor

  uint16_t  uCAN_Data_Enc;
  uint16_t  uCAN_Data_Torq;

  uint8_t   uFlag_Check_Vin;
  uint8_t   uFlag_Check_Enc;
  uint8_t   uFlag_Check_Torq;
  uint8_t   uFlag_Check_Disp;

  float     fAI_A0;
  float     fAI_A1;
};
extern UVAR User_Var;  //전역변수(구조체) User_Var 선언


/* 1st Low Pass Filter */
// (((lpf.coeff*lpf.prev) + (lpf.sampl_Freq*lpf.raw))/(lpf.coeff+lpf.sampl_Freq));
typedef struct LPF_1ST
{
  float sampl_Freq;
  float coeff;
  float prev;
  float raw;
  float out;
  float filter_out;
};
extern LPF_1ST lpf_1st;


typedef struct AVERAGE_FILTER
{
  uint16_t  uCount_Data;
  int  iAvg_Data;

  uint16_t  uUnit_Data;
  uint16_t  uSum_Data;
  
  float     fstack_torq_data[100];
};
extern AVERAGE_FILTER Avg_Filter;

//Flash test, T.A.2
// The struct of the configuration.
struct Parameter_Unit_Step {
  byte amp;
  byte unit_time;
  byte cycle_count;
  byte wait_time;
};
extern Parameter_Unit_Step Param_Unit;

//Flash test, T.A.2
struct Parameter_Frequency {
  uint8_t amp;
  uint8_t freq_min;
  uint8_t freq_max;
  uint8_t wait_time;
};
extern Parameter_Frequency Param_Freq;

// Test
// say you want to store a struct with parameters:
struct parameter {
  float offset;
  float amplitude;
  float freqency;
  float sampling_time;
  float fCount_time;
  float funit_cycle;
};
extern parameter para;


// CAN Layer functions
extern byte canInit(byte cPort, long lBaudRate);
extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte* cData, byte cDataLen);
extern byte canRx(byte cPort, long* lMsgID, bool* bExtendedFormat, byte* cData, byte* cDataLen);

extern byte cTxMsg_torq_text[3][8] = { {0x21, 0x04, 0x20, 0x09, 0x0e, 0x00, 0x00, 0x00}, \
                                          {0x00, 0x33, 0x00, 0x36, 0x00, 0x30, 0x00, 0x2e}, \
                                          {0x11, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00}};  // Period 1ms Message for Torque sensor

extern byte cTxMsg_encoder_text[3][8] = { {0x21, 0x05, 0x20, 0x09, 0x0e, 0x00, 0x00, 0x00}, \
                                          {0x00, 0x33, 0x00, 0x36, 0x00, 0x30, 0x00, 0x2e}, \
                                          {0x11, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00}};  // Period 1ms Message for Torque sensor                                          
                                          
                                          
extern byte cTxMsg_led[4][8] = {{0x2B, DISP_ID_LED_VIN, 0x20, 0x07, 0x00, 0x00, 0x00, 0x00},  \
                                {0x2B, DISP_ID_LED_DISP, 0x20, 0x07, 0x00, 0x00, 0x00, 0x00}, \
                                {0x2B, DISP_ID_LED_TORQ, 0x20, 0x07, 0x00, 0x00, 0x00, 0x00}, \
                                {0x2B, DISP_ID_LED_ENC, 0x20, 0x07, 0x00, 0x00, 0x00, 0x00}};


/*Torque Sensor*/
//array[3] area, update n_period[ms] as ascii type
//default - Period 1ms

//lMsgID (Torque Sensor:0x510)
//MsgFormat (false)
//cDataLen (4)
extern byte cTxMsg_Torq_Period_Cmd[] = { 0x50, 0x52, 0x30, 0x31 };  // cData (0x50, 0x52, 0x30, 0x31) -> PR01(period 1ms command)
extern byte cTxMsg_Torq_Zero_Cmd[] = { 0x5A, 0x45, 0x52, 0x4F };  // cData (0x50, 0x52, 0x30, 0x31) -> PR01(period 1ms command)

//lMsgID (Encoder:0x603)
extern byte cTxMsg_Enc_Zero_Cmd[2][8] = {{0x23, 0x03, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00},  \
                                         {0x23, 0x10, 0x10, 0x01, 0x73, 0x61, 0x76, 0x65}};

//ASCII Code Table(0 ~ 9), (10 -> .)
extern byte Array_ASCII[] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x2E};

// CAN Layer functions
extern byte canInit(byte cPort, long lBaudRate);
extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte* cData, byte cDataLen);
extern byte canRx(byte cPort, long* lMsgID, bool* bExtendedFormat, byte* cData, byte* cDataLen);


//Declare User Functions
void BlinkLED();
void TimerHandler();
uint16_t attachDueInterrupt(double microseconds, timerCallback callback, const char* TimerName);
void Timer_1ms();

void Read_CAN_Message();
char Check_VIN(float a0, float a1);
void Make_Msg_Num_to_ASCII(uint16_t num, char sign, uint16_t id);
void Display_Update();


void Read_ADC();
void Check_Status();
void Handle_CAN_Disp_Data();
char Check_DISP(char flag);
char Check_ENC(char flag);
char Check_TORQ(char flag);




//void TimerHandler()
void TimerHandler() {
  SAMDUE_ISR_Timer.run();
}

//uint16_t attachDueInterrupt()
uint16_t attachDueInterrupt(double microseconds, timerCallback callback, const char* TimerName) {
  DueTimerInterrupt dueTimerInterrupt = DueTimer.getAvailable();
  dueTimerInterrupt.attachInterruptInterval(microseconds, callback);
  uint16_t timerNumber = dueTimerInterrupt.getTimerNumber();

  return timerNumber;
}

//void Timer_1ms()
void Timer_1ms() { 

  //1ms Counter
  if(User_Var.uCount_1ms<999)   User_Var.uCount_1ms++;
  else                          User_Var.uCount_1ms=0;

  //1ms Flag
  User_Var.uFlag_1ms = 1;

  //10ms Flag
  if(User_Var.uCount_1ms%10==0) User_Var.uFlag_10ms = 1;
  else                          User_Var.uFlag_10ms = 0;

  //100ms Flag
  if(User_Var.uCount_1ms%100==0) User_Var.uFlag_100ms = 1;
  //else                  uFlag_100ms = 0;

}

//Check Device
char Check_VIN(float a0, float a1) {

  //Check Voltage Input Range 
  //(조건1) 12.0V, 오차 5% 이내
  //(조건2) 15.0V ~ 30.0V 범위 이내
  //그 외 3초 이상 유지시 Status LED 적색점등
  if((a1<12.60)&&(a1>11.40))
  {
    if((a0<30.00)&&(a0>15.00))  User_Var.uCount_Err_VIN=0;
    else                        User_Var.uCount_Err_VIN++;
  }
  else
  {
    User_Var.uCount_Err_VIN++;
  }
  
  if(User_Var.uCount_Err_VIN>30)   
  {
    User_Var.uCount_Err_VIN=30;
    return (char)HIGH;    //Abnormal
  }
  else
  {
    return (char)LOW;     //Normal
  }
}

void Read_CAN_Message()
{
    /* Nothing to do all is done by hardware. Even no interrupt required. */
  long lMsgID;
  bool bExtendedFormat;
  byte cRxData[8];
  byte cDataLen;
  uint16_t i=0;

  if(canRx(0, &lMsgID, &bExtendedFormat, &cRxData[0], &cDataLen) == CAN_OK) 
  {
    //Read Encoder
    if(lMsgID == CAN_ID_ENC_RD) 
    {
      User_Var.uFlag_Enc_Rx = 1;
      for(i=0;i<cDataLen;i++)
      {
        User_Var.array_enc_hex[i] = cRxData[i];     
      }
    }
    else  
    {
      if(User_Var.uCount_1ms==999)  User_Var.uFlag_Enc_Rx = 0;
      else;
    }

    //Check Winstar Display 
    if(lMsgID == CAN_ID_DISP_RSP) 
    {
      User_Var.uFlag_Disp_Rx = 1; //Normal
    }
    else                          
    {
      if(User_Var.uCount_1ms==999)  User_Var.uFlag_Disp_Rx = 0;
      else;
    }

    //Read Torque Sensor
    if(lMsgID == CAN_ID_TORQ_RD) 
    {
      User_Var.uFlag_Torq_Rx = 1; //Normal
      
      for(i=0;i<cDataLen;i++)
      {
        User_Var.array_torq_hex[i] = cRxData[i];          
      }
    }
    else                
    {
      if(User_Var.uCount_1ms==999)  User_Var.uFlag_Torq_Rx = 0;
      else;
    }

    if(lMsgID == 0x601)
    {
      if(cRxData[1]==0x00 && cRxData[4]==0x01)  
      {
        User_Var.uFlag_Torq_Zero_Cmd=1;  
        Serial.print("stat : Set Zero(Torque)");Serial.print("\n\r");
      }
              
      if(cRxData[1]==0x1 && cRxData[4]==0x01) 
      {
        User_Var.uFlag_Enc_Zero_Cmd=1;  
        Serial.print("stat : Set Zero(encoder)");Serial.print("\n\r");
      }
    }
  }

  lMsgID = 0;
}

//void Num_to_ASCII(), 숫자 6자리(654321) -> ASCII 코드 변환
//숫자 6 : 100
//숫자 5 : 10
//숫자 4 : 1
//숫자 3 : 0.1
//숫자 2 : 0.01
//숫자 1 : 0.001
void Make_Msg_Num_to_ASCII(uint16_t num, char sign, uint16_t id) {
  char  temp_100, temp_010, temp_001 = 0;
  char  temp_0p1, temp_0p2, temp_0p3 = 0;

  switch (id) {

    case  CAN_ID_TORQ_RD :
        temp_100 =  (char)((uint16_t)(num*0.0001));
        temp_010 =  (char)((uint16_t)(num%10000)*0.001);
        temp_001 =  (char)((uint16_t)(num%1000)*0.01);
        temp_0p1 =  (char)((uint16_t)(num%100)*0.1);
        temp_0p2 =  (char)((uint16_t)(num%10));
        temp_0p3 =  (char)((uint16_t)(num%1));

        if(sign==1)   cTxMsg_torq_text[1][1] = 0x2D; //sign
        else          cTxMsg_torq_text[1][1] = 0x2B; //sign

        cTxMsg_torq_text[1][3] = '0'+ temp_100;    //
        cTxMsg_torq_text[1][5] = '0'+ temp_010;    //
        cTxMsg_torq_text[1][7] = '0'+ temp_001;    //
        cTxMsg_torq_text[2][2] = 0x2E;
        cTxMsg_torq_text[2][4] = '0'+ temp_0p1;
        cTxMsg_torq_text[2][6] = '0'+ temp_0p2;
      break;
  
    case  CAN_ID_ENC_RD :
        temp_100 =  (char)((uint16_t)(num*0.0001));
        temp_010 =  (char)((uint16_t)(num%10000)*0.001);
        temp_001 =  (char)((uint16_t)(num%1000)*0.01);
        temp_0p1 =  (char)((uint16_t)(num%100)*0.1);
        temp_0p2 =  (char)((uint16_t)(num%10)*1);
        temp_0p3 =  (char)((uint16_t)(num%1)*0.1);

        cTxMsg_encoder_text[1][1] = '0'+ temp_100;    //
        cTxMsg_encoder_text[1][3] = '0'+ temp_010;    //
        cTxMsg_encoder_text[1][5] = '0'+ temp_001;    //
        cTxMsg_encoder_text[1][7] = 0x2E;
        cTxMsg_encoder_text[2][2] = '0'+ temp_0p1;
        cTxMsg_encoder_text[2][4] = '0'+ temp_0p2;
        cTxMsg_encoder_text[2][6] = '0'+ temp_0p3;
      break;

    default :
      break;
  }

}

//void Display_Update(struct UVAR)
//1. Winstar Display Update.
//2. if not Response, Trasnfer Request Command.
void Display_Update()
{
  switch (User_Var.uCount_1ms%100) {

    case  0 :
      canTx(0, 0x67B, false, &cTxMsg_torq_text[0][0], 8);  //Init Torque Sensor
      break;

    case  2 :
      canTx(0, 0x67B, false, &cTxMsg_torq_text[1][0], 8);  //Init Torque Sensor
      break;

    case  4 :
      canTx(0, 0x67B, false, &cTxMsg_torq_text[2][0], 8);  //Init Torque Sensor
      break;

    case  6 :
      canTx(0, 0x67B, false, &cTxMsg_encoder_text[0][0], 8);  //Init Torque Sensor
      break;

    case  8 :
      canTx(0, 0x67B, false, &cTxMsg_encoder_text[1][0], 8);  //Init Torque Sensor
      break;

    case  10 :
      canTx(0, 0x67B, false, &cTxMsg_encoder_text[2][0], 8);  //Init Torque Sensor
      break;

    case  12 :
      canTx(0, 0x67B, false, &cTxMsg_led[0][0], 8);  //Update S4 Status LED
      break;

    case  14 :
      canTx(0, 0x67B, false, &cTxMsg_led[1][0], 8);  //Update Display Connect Status LED
      break;

    case  16 :
      canTx(0, 0x67B, false, &cTxMsg_led[2][0], 8);  //Update Display Connect Status LED
      break;

    case  18 :
      canTx(0, 0x67B, false, &cTxMsg_led[3][0], 8);  //Update Display Connect Status LED
      break;              

    case  92 :
      if(User_Var.uFlag_Enc_Zero_Cmd==1)  {
        //encoder zero set
        canTx(0, 0x603, false, &cTxMsg_Enc_Zero_Cmd[0][0], 8);  //Torque Sensor (Zero Set)
        User_Var.uFlag_Enc_Zero_Cmd=2;
      }
      break;   

    case  94 :
      if(User_Var.uFlag_Enc_Zero_Cmd==2)  {
        //encoder zero set
        canTx(0, 0x603, false, &cTxMsg_Enc_Zero_Cmd[1][0], 8);  //Torque Sensor (Zero Set)
        User_Var.uFlag_Enc_Zero_Cmd=0;
        Serial.print("stat : Set Zero(encoder) complete");Serial.print("\n\r");
      }
      break;
  

    case  96 :
      if(User_Var.uFlag_Torq_Zero_Cmd==1) {
        //Torque 센서 무응답시 1ms 주기명령 인가
        canTx(0, 0x510, false, &cTxMsg_Torq_Zero_Cmd[0], 4);  //Torque Sensor (Zero Set)
        User_Var.uFlag_Torq_Zero_Cmd=0;
      }
      break;
  
    case  98 :
      //Torque 센서 무응답시 1ms 주기명령 인가
      if(User_Var.stat_torq==1)     canTx(0, 0x510, false, &cTxMsg_Torq_Period_Cmd[0], 4);  //Torque Sensor (PR01)
      else;
      break;

    default :
      break;
  }
}

void Read_ADC()
{
//Read Analog Channel
  User_Var.uAI_A0 = analogRead(A0);  //Read ch_A0 Data
  User_Var.uAI_A1 = analogRead(A1);  //Read ch_A1 Data

  /*A0는 전원입력, 정상범위 15V~30V*/
  User_Var.fAI_A0 = (float)(User_Var.uAI_A0*ADC_Unit*3.3*9.33333);  //Unit Value = ADC_Unit * 3.3V, Gain=9.33333
  /*A1는 12V 내부전원, 정상범위 11.4V~12.6V*/
  User_Var.fAI_A1 = (float)(User_Var.uAI_A1*ADC_Unit*3.3*4);        //Unit Value = ADC_Unit * 3.3V, Gain=4.0
}

void Check_Status()
{
  //Check Voltage Input Range
  User_Var.stat_led = Check_VIN(User_Var.fAI_A0, User_Var.fAI_A1);
  digitalWrite(LED_BUILTIN, User_Var.stat_led); //Abnormal

  //Check Display Connection 
  //stat_disp=1, Abnormal
  //stat_disp=0, Normal
  User_Var.stat_disp = Check_DISP(User_Var.uFlag_Disp_Rx);
  User_Var.uFlag_Disp_Rx = 0;

  //Check Torque Sensor Connection
  //stat_torq=1, Abnormal
  //stat_torq=0, Normal      
  User_Var.stat_torq = Check_TORQ(User_Var.uFlag_Torq_Rx);
  User_Var.uFlag_Torq_Rx = 0;

  //Check Encoder Connection
  //stat_enc=1, Abnormal
  //stat_enc=0, Normal
  User_Var.stat_enc = Check_ENC(User_Var.uFlag_Enc_Rx);
  User_Var.uFlag_Enc_Rx = 0;

  //Update LED Status of Display
  cTxMsg_led[0][4] = (char)(~User_Var.stat_led&0X01);
  cTxMsg_led[2][4] = (char)(~User_Var.stat_torq&0X01);
  cTxMsg_led[3][4] = (char)(~User_Var.stat_enc&0X01);
}

void Handle_CAN_Disp_Data()
{
  //User_Var.uCAN_Data_Torq = (User_Var.array_torq_hex[1]<<8)|(User_Var.array_torq_hex[0]);
  //User_Var.uCAN_Data_Enc  = (User_Var.array_enc_hex[3]<<24)|(User_Var.array_enc_hex[2]<<16)|(User_Var.array_enc_hex[1]<<8)|(User_Var.array_enc_hex[0]);

  if((User_Var.uCAN_Data_Torq&0x8000)>>15) 
  {
    User_Var.uCAN_Data_Torq=(~User_Var.uCAN_Data_Torq&0xFFFF)+1;
    User_Var.Data_Sign = 1;   //Negative(-)
  }
  else
  {
    User_Var.Data_Sign = 0;   //positive(+)
  }

  Make_Msg_Num_to_ASCII(User_Var.uCAN_Data_Torq*10.2, User_Var.Data_Sign, CAN_ID_TORQ_RD);  //1[Nm] = 10.2[kgf-cm], 
  Make_Msg_Num_to_ASCII(User_Var.uCAN_Data_Enc, 0, CAN_ID_ENC_RD);  //
}

//Check Winstar Display CAN Bus connection Status
char Check_DISP(char flag)
{
  if(flag==1)   User_Var.uCount_Err_DISP=0;
  else          User_Var.uCount_Err_DISP++;

  if(User_Var.uCount_Err_DISP>11)
  {
    User_Var.uCount_Err_DISP=11;
    return (char)HIGH;      //Abnormal
  }
  else  return (char)LOW;   //Normal
}

//Check Turck Encoder CAN Bus connection Status
char Check_ENC(char flag)
{
  if(flag==1)   User_Var.uCount_Err_ENC=0;
  else          User_Var.uCount_Err_ENC++;

  if(User_Var.uCount_Err_ENC>10)
  {
    User_Var.uCount_Err_ENC=10;
    return (char)HIGH;      //Abnormal
  }
  else  return (char)LOW;   //Normal
}

//Check ALBOT Torque sensor CAN Bus connection Status
char Check_TORQ(char flag)
{
  if(flag==1)   User_Var.uCount_Err_TORQ=0;
  else          User_Var.uCount_Err_TORQ++;

  if(User_Var.uCount_Err_TORQ>10)
  {
    User_Var.uCount_Err_TORQ=10;
    return (char)HIGH;      //Abnormal
  }
  else  return (char)LOW;   //Normal
}

//1st Low Pass Filter
// y = (Tau*pre_y + Tsampling*x)/(Tau + Tsampling)
// y : output Result, pre_y : previous Result, x : input, Tau : time constant(1/(cutoff Frequency)), Tsampling : sampling time
// sampling time : 50usec(20KHz),  Tau : 2msec(500Hz)

//float raw, float coeff, float prev, float sampl_Freq
float low_pass_filter_1st(LPF_1ST lpf)
{
	float iReturn = 0.0F;
	//1st Low Pass Filter
	//coeff -> (0 < coeff < 1), weight factor
	// y[k] = (1-coeff) * y[k-1] + coeff * x[k]

	//iReturn = (1-coeff)*prev + (coeff*raw);
	lpf.out   = (((lpf.coeff*lpf.prev) + (lpf.sampl_Freq*lpf.raw))/(lpf.coeff+lpf.sampl_Freq));
  lpf.prev  = lpf.out;

	return iReturn=lpf.out;
}




#endif