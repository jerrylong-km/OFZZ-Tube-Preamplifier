/////////////////////////////////////////////////
//           OFZZ RF-5 胆前级控制程序            //
//           Arduino Pro Mini从机程序           //
//         By Jerry Long     Mar 2024          //
/////////////////////////////////////////////////
#include <EEPROM.h>
#include <Arduino.h>
#include "PinDefinitionsAndMore.h" //针脚定义
#include <IRremote.hpp> //红外遥控解码库

#define EEPROM_BAL_SE_ADDRESS 0
#define EEPROM_MUTE_ADDRESS   1

//////////////74HC165与Arduino连线////////////////
/*
    1，VCC(16)接Arduino的5V
    2，GND(8)、DS(10)和CE(15)接Arduino的GND
    3，CP(2)接Arduino的D4
    4，PL(1)接Arduino的D5
    5，Q7(9)接Arduino的D3
 */
/////////////////////////////////////////////////////
const uint8_t ISRDataPin  = 3;  // connected to 74HC165 QH (9) pin
const uint8_t ISRClockPin = 4;  // connected to 74HC165 CLK (2) pin
const uint8_t ISRLatchPin = 5;  // connected to 74HC165 SH/LD (1) pin

//////////////继电器与Arduino连线////////////////
/*
    1，BAL/SE继电器控制接Arduino的D8
    2，MUTE继电器控制接Arduino的D9
 */
///////////////////////////////////////////////
#define BAL_SE_PIN 8
#define MUTE_PIN   9

///////////////////////////////////////////////
//记录各按键的状态信息所需的变量
bool BAL_SE_ON  = false;
bool BALANCE_ON = false;
bool MONO_ON    = false;
bool INVERT_ON  = false;
bool MUTE_ON    = false;

///////////////////////////////////////////////
//红外遥控相关的变量
/*
    1，红外接收器Signal接Arduino的D2
 */
#define DECODE_SONY
unsigned long lastDebounceTime = 0; //记录上次执行操作的时间点
unsigned long debounceDelay = 300; //“去抖动”时间间隔

void setup() {
   // Serial Monitor
   Serial.begin(115200);  // initialize serial bus
   while (!Serial);     // wait for serial connection

   
   //初始化红外接受器
   IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
    
   // 74HC165 shift register
   pinMode(ISRDataPin, INPUT);
   pinMode(ISRLatchPin, OUTPUT);
   pinMode(ISRClockPin, OUTPUT);

   //控制BAL/SE和MUTE继电器的脚位类别为OUTPUT
   pinMode(BAL_SE_PIN, OUTPUT);
   pinMode(MUTE_PIN, OUTPUT);

 
   //读取上次关机时BAL/SE和MUTE按键的信息，并控制相应的继电器
   BAL_SE_ON = EEPROM.read(EEPROM_BAL_SE_ADDRESS);
   if(BAL_SE_ON == true)
   {
     digitalWrite(BAL_SE_PIN, HIGH);
   }
   else
   {
     digitalWrite(BAL_SE_PIN, LOW);
   }

   MUTE_ON = EEPROM.read(EEPROM_MUTE_ADDRESS);
   if(MUTE_ON == true)
   {
     digitalWrite(MUTE_PIN, HIGH);
   }
   else
   {
     digitalWrite(MUTE_PIN, LOW);
   }
   
}



void loop()
{
   // 处理读取的前按键板信号
   readAndPrintInputsOnChange();

   //处理读取的红外遥控信号
   Read_And_Print_IR_On_Change();

   delay(300);
   
}
uint8_t isrReadRegister() {
   uint8_t inputs = 0;
   digitalWrite(ISRClockPin, HIGH);  // preset clock to retrieve first bit
   digitalWrite(ISRLatchPin, HIGH);  // disable input latching and enable shifting
   inputs = shiftIn(ISRDataPin, ISRClockPin, MSBFIRST);  // capture input values
   digitalWrite(ISRLatchPin, LOW);  // disable shifting and enable input latching
   return inputs;
}


void readAndPrintInputsOnChange() {
   static uint8_t previousInputs = 0;
   uint8_t currentInputs = isrReadRegister();  // read all inputs from shift register
   if (currentInputs != previousInputs) {  // print values only if they changed
      previousInputs = currentInputs;
      delay(10);
      //Serial.println(currentInputs);

      //通过串口把按键信息传递给主控MCU，以显示对应的信息
      switch(currentInputs)
      {
        case 30: //BALANCE
        Serial.print(1);
        break;

        case 29: //BAL/SE
        Serial.print(2);
        break;

        case 27: //HRS
        Serial.print(3);
        break;

        case 23: //INVERT
        Serial.print(4);
        break;

        case 15: //MUTE
        Serial.print(5);
        break;

        default:
        break;
      }


      //若按下的是BAL/SE或MUTE，Arduino控制相应的继电器，其他按键交由ESP32主控芯片处理
      //处理BAL/SE按键的相关操作
      if(currentInputs == 29)
      {
        BAL_SE_ON = (BAL_SE_ON == false ? true : false);
        if (BAL_SE_ON == true)
        {
          digitalWrite(BAL_SE_PIN, HIGH);
        }
        else
        {
          digitalWrite(BAL_SE_PIN, LOW);
        }
        EEPROM.update(EEPROM_BAL_SE_ADDRESS, BAL_SE_ON);
      }

      //处理MUTE按键的相关操作
      if(currentInputs == 15)
      {
        MUTE_ON = (MUTE_ON == false ? true : false);
        if (MUTE_ON == true)
        {
          digitalWrite(MUTE_PIN, HIGH);
        }
        else
        {
          digitalWrite(MUTE_PIN, LOW);
        }
        EEPROM.update(EEPROM_MUTE_ADDRESS, MUTE_ON);
      }

      
     }//end of if
}

void Read_And_Print_IR_On_Change()
{
  if (IrReceiver.decode())
  {
    if((millis() - lastDebounceTime) > debounceDelay)
    {    
        switch(IrReceiver.decodedIRData.command)
        {
          case 0x16: //HOURS
          Serial.print(3);//向ESP32发送与面板按键相同编码“3”
          break;

          case 0x15: //POWER
          Serial.print(6);
          break;

          case 0x39: //VOL UP
          Serial.print(7);
          break;

          case 0x3A: //VOL DN
          Serial.print(8);
          break;

          case 0x3B: //BAL L
          Serial.print(9);
          break;

          case 0x3C: //BAL R
          Serial.print(10);
          break;

          case 0x3D: //MUTE
          Serial.print(5);//向ESP32发送与面板按键相同编码“5”
          MUTE_ON = (MUTE_ON == false ? true : false);
          if (MUTE_ON == true)
          {
            digitalWrite(MUTE_PIN, HIGH);
          }
          else
          {
            digitalWrite(MUTE_PIN, LOW);
          }
          EEPROM.update(EEPROM_MUTE_ADDRESS, MUTE_ON);
          break;

          case 0x67: //TUNER
          Serial.print(11);
          break;

          case 0x68: //CD
          Serial.print(12);
          break;

          case 0x69: //VIDEO
          Serial.print(13);
          break;

          case 0x4C: //PHONO
          Serial.print(14);
          break;

          case 0x59: //AUX1
          Serial.print(15);
          break;

          case 0x42: //AUX2
          Serial.print(16);
          break;

          case 0x1B: //DSP DN
          Serial.print(17);
          break;

          case 0x1A: //MONO
          Serial.print(18);
          break;

          case 0x1C: //DSP UP
          Serial.print(19);
          break;

          case 0x57: //INVERT
          Serial.print(4);//向ESP32发送与面板按键相同编码“4”
          break;

          case 0x19: //BAL/SE
          Serial.print(2);//向ESP32发送与面板按键相同编码“2”
          BAL_SE_ON = (BAL_SE_ON == false ? true : false);
          if (BAL_SE_ON == true)
          {
            digitalWrite(BAL_SE_PIN, HIGH);
          }
          else
          {
            digitalWrite(BAL_SE_PIN, LOW);
          }
          EEPROM.update(EEPROM_BAL_SE_ADDRESS, BAL_SE_ON);
          break;

          case 0x18: //PROC
          Serial.print(29);
          break;

          default:
          break;
        }//end of switch
        
        IrReceiver.resume();
       
    }//end of debounce
    lastDebounceTime = millis();

  }//end of decode*/
}
