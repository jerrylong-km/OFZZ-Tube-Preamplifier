/////////////////////////////////////////////////
//           OFZZ RF-5 胆前级控制程序            //
//           Arduino Pro Mini从机程序           //
//         By Jerry Long     Mar 2024          //
/////////////////////////////////////////////////
#include <EEPROM.h>

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

void setup() {
   // Serial Monitor
   Serial.begin(115200);  // initialize serial bus
   while (!Serial);     // wait for serial connection
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



void loop() {
   // Read and print inputs at the specified sampling rate
   readAndPrintInputsOnChange();
   
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

      //通过串口把按键信息传递给主控MCU，以显示对应的信息
      if(currentInputs == 15)//因为ESP32接收到的Arduino串口发送的信息，MONO按键和MUTE按键都是102，产生冲突，所以MUTE按键的信息改为发送数字1
      {
        Serial.println(1);
      }
      else
      {
        Serial.println(currentInputs);
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
