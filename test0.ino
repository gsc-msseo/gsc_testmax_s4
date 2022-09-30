/****************************************************************************************************************************
  TimerInterruptLEDDemo.ino
  For SAM DUE boards
  Written by Khoi Hoang
  
  Now even you use all these new 16 ISR-based timers,with their maximum interval practically unlimited (limited only by
  unsigned long miliseconds), you just consume only one Hardware timer and avoid conflicting with other cores' tasks.
  The accuracy is nearly perfect compared to software timers. The most important feature is they're ISR-based timers
  Therefore, their executions are not blocked by bad-behaving functions / tasks.
  This important feature is absolutely necessary for mission-critical tasks.
  
  Based on SimpleTimer - A timer library for Arduino.
  Author: mromani@ottotecnica.com
  Copyright (c) 2010 OTTOTECNICA Italy
  
  Based on BlynkTimer.h
  Author: Volodymyr Shymanskyy
  
  Built by Khoi Hoang https://github.com/khoih-prog/TimerInterrupt_Generic
  Licensed under MIT license
*****************************************************************************************************************************/

/*
   Notes:
   Special design is necessary to share data between interrupt code and the rest of your program.
   Variables usually need to be "volatile" types. Volatile tells the compiler to avoid optimizations that assume
   variable can not spontaneously change. Because your function may change variables while your program is using them,
   the compiler needs this hint. But volatile alone is often not enough.
   When accessing shared variables, usually interrupts must be disabled. Even with volatile,
   if the interrupt changes a multi-byte variable between a sequence of instructions, it can be read incorrectly.
   If your data is multiple variables, such as an array and a count, usually interrupts need to be disabled
   or the entire sequence of your code which accesses the data.
*/

#if !(defined(ARDUINO_SAM_DUE) || defined(__SAM3X8E__))
#error This code is designed to run on SAM DUE board / platform! Please check your Tools->Board setting.
#endif

// These define's must be placed at the beginning before #include "TimerInterrupt_Generic.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
// Don't define TIMER_INTERRUPT_DEBUG > 2. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG 0
#define _TIMERINTERRUPT_LOGLEVEL_ 0

#include <DueFlashStorage.h>          //Flash Memory, T.A.2
#include "string.h"                   //  
#include "TimerInterrupt_Generic.h"   //Timer 
#include "ISR_Timer_Generic.h"        //Timer 
#include "DueCANLayer.h"              //CAN
#include "User_Define.h"              //

//구조체 선언
DueFlashStorage dueFlashStorage;  //Flash Memory Access class

UVAR            User_Var;       //전역변수(구조체) User_Var 선언
AVERAGE_FILTER  Avg_Filter;
LPF_1ST         lpf_1st;
Parameter_Unit_Step Param_Unit;   //T.A.2
Parameter_Frequency Param_Freq;   //T.A.2

parameter para;   //test



// In SAM-DUE, avoid doing something fancy in ISR, for example complex Serial.print with String() argument
// The pure simple Serial.prints here are just for demonstration and testing. Must be eliminate in working environment
// Or you can get this run-time error / crash
void setup() {

  //test
  memset(&para, 0, sizeof(parameter)); 

  para.amplitude  = 1.0;
  para.freqency   = 10;
  para.offset     = 1.0;
  para.sampling_time  = 0.001;
  
  para.funit_cycle = 1/para.freqency/para.sampling_time; //1ms count (@ unit period)
  para.fCount_time  = 0; 
  
  //test, T.A.2
  /*
  Param_Unit.amp = 100;
  Param_Unit.unit_time = 5;
  Param_Unit.cycle_count = 3;
  Param_Unit.wait_time= 2;

  byte array[sizeof(Parameter_Unit_Step)];
  memcpy(array, &Param_Unit, sizeof(Parameter_Unit_Step));
  dueFlashStorage.write(FLASH_T_UNIT_AMP, array, sizeof(Parameter_Unit_Step));
  */

  //dueFlashStorage.write(0, 0, 1);   //T.A.2

  //Initialize Struct
  memset(&User_Var, 0, sizeof(UVAR));    
  memset(&Avg_Filter, 0, sizeof(AVERAGE_FILTER));    
  memset(&Avg_Filter, 0, sizeof(LPF_1ST));    

  
  lpf_1st.sampl_Freq = 0.001;
  lpf_1st.coeff = 1/LPF_1ST_FC;               

  //Initialize DAC resolution (12bit, 4096)
  analogWriteResolution(12);

  //Initialize Serial Comm Baudrate(115200bps)
  Serial.begin(115200);

  //Initialize both CAN controllers
  if (canInit(0, CAN_BPS_1000K) == CAN_OK)
    Serial.print("CAN0: Initialized Successfully.\n\r");
  else
    Serial.print("CAN0: Initialization Failed.\n\r");

  if (canInit(1, CAN_BPS_1000K) == CAN_OK)
    Serial.print("CAN1: Initialized Successfully.\n\r");
  else
    Serial.print("CAN1: Initialization Failed.\n\r");

  // configure pin in output mode
  pinMode(LED_BUILTIN, OUTPUT);

  // Interval in 1 millisecond(ms)
  attachDueInterrupt(HW_TIMER_INTERVAL_MS, TimerHandler, "ITimer");

  // Just to demonstrate, don't use too many ISR Timers if not absolutely necessary
  // You can use up to 16 timer for each SAMDUE_ISR_Timer
  SAMDUE_ISR_Timer.setInterval(TIMER_INTERVAL_1S, BlinkLED);
  SAMDUE_ISR_Timer.setInterval(TIMER_INTERVAL_1MS, Timer_1ms);

  delayMicroseconds(20);
}

float test_dac = 0.0;
uint16_t utime = 0;

void loop() {

  while(1) 
  {
    // Check for received message
    Read_CAN_Message();

    //1ms period
    if(User_Var.uFlag_1ms==1)
    {
      User_Var.uFlag_1ms=0;

      //Update Winstar Display Data
      Display_Update();     

      User_Var.uCAN_Data_Torq = (User_Var.array_torq_hex[1]<<8)|(User_Var.array_torq_hex[0]);
      User_Var.uCAN_Data_Enc  = (User_Var.array_enc_hex[3]<<24)|(User_Var.array_enc_hex[2]<<16)|(User_Var.array_enc_hex[1]<<8)|(User_Var.array_enc_hex[0]);

      
      //DAC output test
      test_dac = para.amplitude*sin(2*PI*para.freqency*para.fCount_time)+para.offset;
      analogWrite(DAC1, test_dac*DAC_Unit);
        

      if(para.fCount_time<para.funit_cycle)  para.fCount_time=para.fCount_time+para.sampling_time;
      else                                   para.fCount_time=0;

      //Flash write test, T.A.2
      if(Serial.available())
      {
        if(Serial.read()=='a')
        {
            Param_Unit.amp = 100;
            Param_Unit.unit_time = 5;
            Param_Unit.cycle_count = 3;
            Param_Unit.wait_time= 2;
        }

        byte array[sizeof(Parameter_Unit_Step)];
        memcpy(array, &Param_Unit, sizeof(Parameter_Unit_Step));
        dueFlashStorage.write(FLASH_T_UNIT_AMP, array, sizeof(Parameter_Unit_Step));
      }

    }

    //100ms period
    if(User_Var.uFlag_100ms==1)
    {
      User_Var.uFlag_100ms=0;

      //1. Check S4 & CAN Status 
      //2. Diplay Status LED 
      Check_Status();

      //Read ADC Data
      Read_ADC();

      //Handle CAN Data for Display
      Handle_CAN_Disp_Data();
    }

  }

}


//void BlinkLED()
void BlinkLED() {

  //Flash Read test, T.A.2
  //Base Address = 0x80000;
  Serial.print("0x81000 : ");
  Serial.println(dueFlashStorage.read(FLASH_T_UNIT_AMP));
  Serial.print("0x81001 : ");
  Serial.println(dueFlashStorage.read(FLASH_T_UNIT_UT));
  Serial.print("0x81002 : ");
  Serial.println(dueFlashStorage.read(FLASH_T_UNIT_CC));
  Serial.print("0x81003 : ");
  Serial.println(dueFlashStorage.read(FLASH_T_UNIT_WT));

  //if(dueFlashStorage.read(FLASH_T_UNIT_AMP)==100)   digitalWrite(LED_BUILTIN, 1); //Abnormal
  //else                                              digitalWrite(LED_BUILTIN, 0); //Abnormal
}


