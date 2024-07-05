/////////////////////////////////////////////////
//           OFZZ RF-5 胆前级控制程序            //
//                ESP32主控程序                 //
//         By Jerry Long     Mar 2024          //
/////////////////////////////////////////////////
#include <lvgl.h>
#include <Arduino_GFX_Library.h>
#include <AiEsp32RotaryEncoder.h>
#include <ui.h>
#include <EEPROM.h>
#include <Muses72323.h>
#include <SoftTimers.h>

/////////////////////////////////////////////
////          胆管使用时间定义部分          ////
/////////////////////////////////////////////
unsigned int Tube_Used_Hours = 0;
char Tube_Used_Hours_Text[16];
SoftTimer Hours_Counter;
unsigned int Time_Count = 0;
unsigned int Old_Time_Count =0;

/////////////////////////////////////////////
////           TFT显示屏定义部分           ////
/////////////////////////////////////////////

//GFX 显示屏数据总线和驱动设定
#define GFX_BL DF_GFX_BL //默认背光引脚，可以将DF_GFX_BL设置为实际使用的引脚

/* 更多开发板套件设置可参考: https://github.com/moononournation/Arduino_GFX/wiki/Dev-Device-Declaration */
#if defined(DISPLAY_DEV_KIT)
Arduino_GFX *gfx = create_default_Arduino_GFX();
#else /* !defined(DISPLAY_DEV_KIT) */

/* 更多数据总线类型可参考: https://github.com/moononournation/Arduino_GFX/wiki/Data-Bus-Class */
Arduino_DataBus *bus = new Arduino_ESP32LCD8(
    11 /*DC*/, 12 /*CS*/, 10 /* WR */, 9 /* RST */,
    46 /* D0 */, 3 /* D1 */, 8 /* D2 */, 18 /* D3 */, 17 /* D4 */, 16 /* D5 */, 15 /* D6 */, 7 /* D7 */);

/* 更多显示器类型可参考: https://github.com/moononournation/Arduino_GFX/wiki/Display-Class */
Arduino_GFX *gfx = new Arduino_HX8369A(bus, DF_GFX_RST, 1 /* rotation */, false /* IPS */);

#endif /* !defined(DISPLAY_DEV_KIT) */

/*可根据实际使用显示屏的分辨率设定*/
//static const uint16_t screenWidth  = 480;
//static const uint16_t screenHeight = 800;
static uint32_t screenWidth;
static uint32_t screenHeight;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_color_t *disp_draw_buf2;
static lv_disp_drv_t disp_drv;

//相关显示内容的参数
char VolumeText[16];
int32_t VolumeVal = 0;
char SourceText[16];
int32_t SourceVal = 0;
char BalanceText[16];
int vol_counter = 0;//用于记录通过触屏调整音量大小时，更新关联的参数，该变量值是UI库中需要进行外部引用


/////////////////////////////////////////////
////           旋转编码器定义部分           ////
/////////////////////////////////////////////
/*
 音量旋转编码器接线
 旋转编码器端                   ESP32端
 ----------                   ----------
 CLK(A)                       GPIO14
 DT(B)                        GPIO19
 SW                           GPIO13
*/
#define VOLUME_ROTARY_ENCODER_A_PIN 14
#define VOLUME_ROTARY_ENCODER_B_PIN 19
#define VOLUME_ROTARY_ENCODER_BUTTON_PIN 13
#define VOLUME_ROTARY_ENCODER_STEPS 4
#define VOLUME_ROTARY_ENCODER_VCC_PIN -1 /* 当旋转编码器的供电VCC直接接板子输出5V时，该值设为-1 */
//定义音量旋转编码器对象
AiEsp32RotaryEncoder Volume_rotaryEncoder = AiEsp32RotaryEncoder(VOLUME_ROTARY_ENCODER_A_PIN, VOLUME_ROTARY_ENCODER_B_PIN, VOLUME_ROTARY_ENCODER_BUTTON_PIN, VOLUME_ROTARY_ENCODER_VCC_PIN, VOLUME_ROTARY_ENCODER_STEPS);

/*
 音源旋转编码器接线
 旋转编码器端                   ESP32端
 ----------                   ----------
 CLK(A)                       GPIO42
 DT(B)                        GPIO2
 SW                           GPIO1
 */
#define SOURCE_ROTARY_ENCODER_A_PIN 42
#define SOURCE_ROTARY_ENCODER_B_PIN 2
#define SOURCE_ROTARY_ENCODER_BUTTON_PIN 1
#define SOURCE_ROTARY_ENCODER_STEPS 4
#define SOURCE_ROTARY_ENCODER_VCC_PIN -1 /* 当旋转编码器的供电VCC直接接板子输出5V时，该值设为-1 */

//定义音源旋转编码器对象
AiEsp32RotaryEncoder Source_rotaryEncoder = AiEsp32RotaryEncoder(SOURCE_ROTARY_ENCODER_A_PIN, SOURCE_ROTARY_ENCODER_B_PIN, SOURCE_ROTARY_ENCODER_BUTTON_PIN, SOURCE_ROTARY_ENCODER_VCC_PIN, SOURCE_ROTARY_ENCODER_STEPS);
//定义当前Source
static int currentSource = 0;

/////////////////////////////////////////////
////        Muses72323芯片定义部分         ////
/////////////////////////////////////////////
/*
    Muses72323芯片SPI接线：
      Latch(20#)——>SDL——>GPIO48(SS)
      Clock(19#)——>SCL——>GPIO45(SCK)
      Data(18#) ——>SDA——>GPIO0(MOSI)
 */

//定义Muses72323芯片的地址（根据ADR0-#30和ADR1-#31脚位的高低电位）
static const byte LEFT_MUSES_ADDRESS  = 0;//A0和A1两个跳线都开路
static const byte RIGHT_MUSES_ADDRESS = 1;//A0跳线(W4)开路，A1跳线(W3)短接

static Muses72323 MusesLeft(LEFT_MUSES_ADDRESS); //根据上述芯片地址创建Muses72323对象
static Muses72323 MusesRight(RIGHT_MUSES_ADDRESS);


//定义默认音量，当前音量，最大和最小音量
static Muses72323::volume_t defaultVolumeLeft  = 0; //重启后的默认音量-50dB
static Muses72323::volume_t defaultVolumeRight = 0;
static Muses72323::volume_t currentVolumeLeft  = 0; //当前音量，-20dB开始
static Muses72323::volume_t currentVolumeRight = 0;
int MaxVolume = 0; //最大音量0dB
int MinVolume = -479; //最小音量-111.75dB
static Muses72323::volume_t currentVolume = 0; //测试用参数，当前音量，-20dB开始

//定义与左右声道平衡调节相关的参数
int Balance_Num = 0;//设置左右声道平衡，取值范围：-100~+100，中点为0，负值为左声道音量增大，正值为右声道音量加大
bool Balance_ON = false;//用于标识当前音量旋转编码器所处状态，false为正常调整音量状态，true为调节左右声道平衡状态
bool Balance_OFF = false;//用于标识编码器作为声道平衡调节状态是否结束

#define CLK_PIN  45
#define MISO_PIN 13
#define MOSI_PIN 0
#define CS_PIN   48

/////////////////////////////////////////////
////     音源信号输入板74HC595定义部分      ////
/////////////////////////////////////////////
/*
 音源74HC595接线
 74HC595 端                   ESP32端
 ----------                   ----------
 SER(Data)                    GPIO47
 RCLK(Latch)                  GPIO21
 SRCLK(Clock)                 GPIO20
 */
#define LatchPin 21
#define ClockPin 20
#define DataPin  47

/////////////////////////////////////////////
////            EEPROM定义部分            ////
/////////////////////////////////////////////
//定义使用的EEPROM容量大小为10(0，音量L；1, 音量R；2，音源；3，平衡/单端；4，单声道；5，反向；6，静音；7，胆管使用时间；8，显示屏音量；9，左右声道平衡)
#define EEPROM_SIZE               10
#define EEPROM_VOLUME_L_ADDRESS   0
#define EEPROM_VOLUME_R_ADDRESS   1
#define EEPROM_INPUT_ADDRESS      2
#define EEPROM_BAL_SE_ADDRESS     3
#define EEPROM_MONO_ADDRESS       4
#define EEPROM_INVERT_ADDRESS     5
#define EEPROM_MUTE_ADDRESS       6
#define EEPROM_TUBE_HRS_ADDRESS   7
#define EEPROM_VOL_DISP_ADDRESS   8
#define EEPROM_BALANCE_ADDRESS    9

/////////////////////////////////////////////
////        ESP32和Arduino通讯            ////
/////////////////////////////////////////////
#define RXp 41
#define TXp 37
int Arduino_Message_Input = 0;
//记录按键状态
bool BAL_SE_ON  = false;
bool MONO_ON    = false;
bool INVERT_ON  = false;
bool MUTE_ON    = false;

/////////////////////////////////////////////
////        DG409反向芯片控制              ////
/////////////////////////////////////////////
#define A0_Pin 35
#define A1_Pin 36

/////////////////////////////////////////////
////        处理屏幕显示的相应操作          ////
/////////////////////////////////////////////
//增加触屏板头文件
#include "touch.h"

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

/* 显示屏填充 */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    #if (LV_COLOR_16_SWAP != 0)
      gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
    #else
      gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
    #endif

    lv_disp_flush_ready( disp );
}

/*读取触控板*/
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
   //已更改为GT911驱动所需格式
   if (touch_has_signal())
    {
        if (touch_touched())
        {
            data->state = LV_INDEV_STATE_PR;

            /*Set the coordinates*/
            data->point.x = touch_last_x;
            data->point.y = touch_last_y;
        }
        else if (touch_released())
        {
            data->state = LV_INDEV_STATE_REL;
        }
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }
}

/////////////////////////////////////////////
////  处理音量旋转编码器按钮按下的相应操作    ////
/////////////////////////////////////////////
void Volume_rotary_onButtonClick()
{
  static unsigned long lastTimePressed = 0;
  //ignore multiple press in that time milliseconds
  if (millis() - lastTimePressed < 500)
  {
    return;
  }
  lastTimePressed = millis();
  Serial.print("button pressed ");
  Serial.print(millis());
  Serial.println("milliseconds after restart");
}

/////////////////////////////////////////////
////    处理音量旋转编码器转动的相应操作      ////
/////////////////////////////////////////////
void Volume_rotary_loop()
{
  //dont print anything unless value changed
  if (Volume_rotaryEncoder.encoderChanged())
  {
    currentVolumeLeft = Volume_rotaryEncoder.readEncoder();
    currentVolumeRight = currentVolumeLeft;
    
    //用设定的Balan_Num对左右声道进行平衡修正
    if(Balance_Num <= 0) //声道平衡设置时，左声道>右声道，用平衡数值修正左声道音量
    {
      MusesLeft.setVolume(currentVolumeLeft - Balance_Num, currentVolumeLeft - Balance_Num);
      MusesRight.setVolume(currentVolumeRight, currentVolumeRight);
    } //声道平衡设置时，左声道<右声道，用平衡数值修正右声道音量
    else
    {
      MusesLeft.setVolume(currentVolumeLeft, currentVolumeLeft);
      MusesRight.setVolume(currentVolumeRight + Balance_Num, currentVolumeRight + Balance_Num);
    }

    //改变显示屏的音量显示数字
    VolumeVal = map(currentVolumeLeft, -470, 0, 0, 100);
    itoa(VolumeVal, VolumeText, 10);
    lv_label_set_text(ui_Label2, VolumeText);
    lv_slider_set_value(ui_Slider1, VolumeVal, LV_ANIM_ON);
       
    //音量值发生变化时，将当前音量值写入EEPROM，因EEPROM存储的数值范围是0-255，不能存储负值，所以取反后存入
    EEPROM.write(EEPROM_VOLUME_L_ADDRESS, -currentVolumeLeft);
    EEPROM.write(EEPROM_VOLUME_R_ADDRESS, -currentVolumeRight);
    EEPROM.write(EEPROM_VOL_DISP_ADDRESS, VolumeVal);
    EEPROM.commit();
    Serial.print("currentVolume: ");
    Serial.println(currentVolumeLeft);
  }
  if (Volume_rotaryEncoder.isEncoderButtonClicked())
  {
    Volume_rotary_onButtonClick();
  }
}

/////////////////////////////////////////////
////        关联音量旋转编码器中断          ////
/////////////////////////////////////////////
void IRAM_ATTR Volume_readEncoderISR()
{
  Volume_rotaryEncoder.readEncoder_ISR();
}

/////////////////////////////////////////////
////  处理音源旋转编码器按钮按下的相应操作    ////
/////////////////////////////////////////////
void Source_rotary_onButtonClick()
{
  static unsigned long lastTimePressed = 0;
  //ignore multiple press in that time milliseconds
  if (millis() - lastTimePressed < 500)
  {
    return;
  }
  lastTimePressed = millis();
  Serial.print("button pressed ");
  Serial.print(millis());
  Serial.println(" milliseconds after restart");
}

/////////////////////////////////////////////
////    处理音源旋转编码器转动的相应操作      ////
/////////////////////////////////////////////
void Source_rotary_loop()
{
  //dont print anything unless value changed
  if (Source_rotaryEncoder.encoderChanged())
  {
    currentSource = Source_rotaryEncoder.readEncoder();
    //根据音源编号对74HC595进行写入，打开对应的继电器
    UpdateShiftRegister(pow(2, currentSource));

    //改变显示屏的音源标签
       switch(currentSource)
       {
         case 0:
         lv_label_set_text(ui_Source, "CD");
         break;

         case 1:
         lv_label_set_text(ui_Source, "TUNER");
         break; 
         
         case 2:
         lv_label_set_text(ui_Source, "PHONO");
         break;
         
         case 3:
         lv_label_set_text(ui_Source, "VIDEO");
         break;
         
         case 4:
         lv_label_set_text(ui_Source, "AUX1");
         break;
         
         case 5:
         lv_label_set_text(ui_Source, "AUX2");
         break;
         
         case 6:
         lv_label_set_text(ui_Source, "PROC");
         break;

         default:
         break;
       }
    //音源值发生变化时，将当前音源值写入EEPROM
    EEPROM.write(EEPROM_INPUT_ADDRESS, currentSource);
    EEPROM.commit();
    Serial.print("currentSource: ");
    Serial.println(currentSource);
  }
  if (Source_rotaryEncoder.isEncoderButtonClicked())
  {
    Source_rotary_onButtonClick();
  }
}

/////////////////////////////////////////////
////        关联音源旋转编码器中断          ////
/////////////////////////////////////////////
void IRAM_ATTR Source_readEncoderISR()
{
  Source_rotaryEncoder.readEncoder_ISR();
}

/////////////////////////////////////////////
////    处理触屏音量Slider变动的相应操作     ////
/////////////////////////////////////////////
void Volume_Slider_loop()
{
  static int last_vol_counter = 0;//音量旋转编码器计数状态重置
  
   // 音量发生变化
   if(vol_counter != last_vol_counter)
   {
       Serial.print("vol_counter = ");
       Serial.println(vol_counter);

       //改变显示屏的音量显示数字
       VolumeVal = vol_counter;
       itoa(VolumeVal, VolumeText, 10);
       lv_label_set_text(ui_Label2, VolumeText);
       lv_slider_set_value(ui_Slider1, VolumeVal, LV_ANIM_ON);

       //根据音量滑动条的位置设定音量旋转编码器的位置
       currentVolumeLeft = map(VolumeVal, 0, 100, -470, 0);
       currentVolumeRight = currentVolumeLeft;
       Volume_rotaryEncoder.setEncoderValue(currentVolumeLeft);
       //同步更新Muses72323的增益
       //用设定的Balan_Num对左右声道进行平衡修正
       if(Balance_Num <= 0) //声道平衡设置时，左声道>右声道，用平衡数值修正左声道音量
       {
         MusesLeft.setVolume(currentVolumeLeft - Balance_Num, currentVolumeLeft - Balance_Num);
         MusesRight.setVolume(currentVolumeRight, currentVolumeRight);
       } //声道平衡设置时，左声道<右声道，用平衡数值修正右声道音量
       else
       {
         MusesLeft.setVolume(currentVolumeLeft, currentVolumeLeft);
         MusesRight.setVolume(currentVolumeRight + Balance_Num, currentVolumeRight + Balance_Num);
       }

       //将变化后的音量值写入EEPROM，因EEPROM存储的数值范围是0-255，不能存储负值，所以取反后存入
       EEPROM.write(EEPROM_VOLUME_L_ADDRESS, -currentVolumeLeft);
       EEPROM.write(EEPROM_VOLUME_R_ADDRESS, -currentVolumeRight);
       EEPROM.write(EEPROM_VOL_DISP_ADDRESS, VolumeVal);
       EEPROM.commit();
       last_vol_counter = vol_counter;

   }
}

/////////////////////////////////////////////
////       74HC595寄存器写入程序           ////
/////////////////////////////////////////////
void UpdateShiftRegister(byte Input_Num)
{
   digitalWrite(LatchPin, LOW);
   shiftOut(DataPin, ClockPin, LSBFIRST, Input_Num);
   digitalWrite(LatchPin, HIGH);
}

/////////////////////////////////////////////
////  处理从Arduino接收的按键/红外遥控信息   ////
/////////////////////////////////////////////
void Arduino_Message_Read()
{
    if(Serial2.available())
    {
      Arduino_Message_Input = Serial2.read() + Serial2.read();
      
      switch(Arduino_Message_Input)
      {
        case 48:
        //按下“BALANCE”按钮后，标识Balance_ON为true, 设置旋转编码器复用参数，显示界面切换到Screen2
        Balance_ON = true;
        currentVolumeLeft = Volume_rotaryEncoder.readEncoder();//切换为“平衡模式”前保存当前音量数据
        Volume_rotaryEncoder.setBoundaries(-100, 100, false);
        Volume_rotaryEncoder.disableAcceleration();
        Volume_rotaryEncoder.setEncoderValue(Balance_Num);//切换为“平衡模式”后编码器位置设置为上次的平衡数据
        _ui_screen_change( &ui_Screen2, LV_SCR_LOAD_ANIM_FADE_ON, 50, 50, &ui_Screen2_screen_init);
        itoa(Balance_Num, BalanceText, 10);
        lv_label_set_text(ui_BALNUMBER, BalanceText);
        break;

        case 49: //按下“BAL/SE”切换RCA和XLR
        BAL_SE_ON = (BAL_SE_ON == false ? true : false);
        if (BAL_SE_ON == true)
        {
          lv_label_set_text(ui_Input, "SE");
        }
        else
        {
          lv_label_set_text(ui_Input, "BAL");
        }
        EEPROM.write(EEPROM_BAL_SE_ADDRESS, BAL_SE_ON);
        EEPROM.commit();
        break;

        case 50://按下"HRS"按钮后，显示界面切换到Screen3
        _ui_screen_change( &ui_Screen3, LV_SCR_LOAD_ANIM_FADE_ON, 50, 50, &ui_Screen3_screen_init);
        itoa(Tube_Used_Hours, Tube_Used_Hours_Text, 10);
        lv_label_set_text(ui_Label6, Tube_Used_Hours_Text);
        break;

        case 51:
        INVERT_ON = (INVERT_ON == false ? true : false);
        if (INVERT_ON == true)
        {
          //控制DG409做相应的反向
          digitalWrite(A0_Pin, HIGH);
          digitalWrite(A1_Pin, LOW);
          lv_label_set_text(ui_InvertOn, "INVERT");
        }
        else
        {
          //控制DG409做相应的正向
          digitalWrite(A0_Pin, LOW);
          digitalWrite(A1_Pin, LOW);
          lv_label_set_text(ui_InvertOn, "");
        }
        EEPROM.write(EEPROM_INVERT_ADDRESS, INVERT_ON);
        EEPROM.commit();
        break;

        case 52:
        MUTE_ON = (MUTE_ON == false ? true : false);
        if (MUTE_ON == true)
        {
          lv_label_set_text(ui_MuteOn, "MUTE ON");
        }
        else
        {
          lv_label_set_text(ui_MuteOn, "");
        }
        EEPROM.write(EEPROM_MUTE_ADDRESS, MUTE_ON);
        EEPROM.commit();
        break;

        case 54:
        Serial.println("VOL UP");
        currentVolumeLeft += 4;//音量编码器旋转一格音量变化0.25dB，遥控器按一下音量变化1dB
        currentVolumeRight = currentVolumeLeft;
        
        //用设定的Balan_Num对左右声道进行平衡修正
        if(Balance_Num <= 0) //声道平衡设置时，左声道>右声道，用平衡数值修正左声道音量
        {
          MusesLeft.setVolume(currentVolumeLeft - Balance_Num, currentVolumeLeft - Balance_Num);
          MusesRight.setVolume(currentVolumeRight, currentVolumeRight);
        } //声道平衡设置时，左声道<右声道，用平衡数值修正右声道音量
        else
        {
          MusesLeft.setVolume(currentVolumeLeft, currentVolumeLeft);
          MusesRight.setVolume(currentVolumeRight + Balance_Num, currentVolumeRight + Balance_Num);
        }
        Volume_rotaryEncoder.setEncoderValue(currentVolumeLeft);
        
        //改变显示屏的音量显示数字
        VolumeVal = map(currentVolumeLeft, -470, 0, 0, 100);
        itoa(VolumeVal, VolumeText, 10);
        lv_label_set_text(ui_Label2, VolumeText);
        lv_slider_set_value(ui_Slider1, VolumeVal, LV_ANIM_ON);
        break;

        case 55:
        Serial.println("VOL DN");
        currentVolumeLeft -= 4;//音量编码器旋转一格音量变化0.25dB，遥控器按一下音量变化1dB
        currentVolumeRight = currentVolumeLeft;
        
        
        //用设定的Balan_Num对左右声道进行平衡修正
        if(Balance_Num <= 0) //声道平衡设置时，左声道>右声道，用平衡数值修正左声道音量
        {
          MusesLeft.setVolume(currentVolumeLeft - Balance_Num, currentVolumeLeft - Balance_Num);
          MusesRight.setVolume(currentVolumeRight, currentVolumeRight);
        } //声道平衡设置时，左声道<右声道，用平衡数值修正右声道音量
        else
        {
          MusesLeft.setVolume(currentVolumeLeft, currentVolumeLeft);
          MusesRight.setVolume(currentVolumeRight + Balance_Num, currentVolumeRight + Balance_Num);
        }
        Volume_rotaryEncoder.setEncoderValue(currentVolumeLeft);
        
        //改变显示屏的音量显示数字
        VolumeVal = map(currentVolumeLeft, -470, 0, 0, 100);
        itoa(VolumeVal, VolumeText, 10);
        lv_label_set_text(ui_Label2, VolumeText);
        lv_slider_set_value(ui_Slider1, VolumeVal, LV_ANIM_ON);
        break;

        case 56:
        Serial.println("BAL L");
        break;

        case 97:
        Serial.println("BAL R");
        break;

        case 98:
        lv_label_set_text(ui_Source, "TUNER");
        currentSource = 1;
        UpdateShiftRegister(pow(2, currentSource));
        Source_rotaryEncoder.setEncoderValue(currentSource);
        EEPROM.write(EEPROM_INPUT_ADDRESS, currentSource);
        EEPROM.commit();
        break;

        case 99:
        lv_label_set_text(ui_Source, "CD");
        currentSource = 0;
        UpdateShiftRegister(pow(2, currentSource));
        Source_rotaryEncoder.setEncoderValue(currentSource);
        EEPROM.write(EEPROM_INPUT_ADDRESS, currentSource);
        EEPROM.commit();
        break;

        case 100:
        lv_label_set_text(ui_Source, "VIDEO");
        currentSource = 3;
        UpdateShiftRegister(pow(2, currentSource));
        Source_rotaryEncoder.setEncoderValue(currentSource);
        EEPROM.write(EEPROM_INPUT_ADDRESS, currentSource);
        EEPROM.commit();
        break;

        case 101:
        lv_label_set_text(ui_Source, "PHONO");
        currentSource = 2;
        UpdateShiftRegister(pow(2, currentSource));
        Source_rotaryEncoder.setEncoderValue(currentSource);
        EEPROM.write(EEPROM_INPUT_ADDRESS, currentSource);
        EEPROM.commit();
        break;

        case 102:
        lv_label_set_text(ui_Source, "AUX1");
        currentSource = 4;
        UpdateShiftRegister(pow(2, currentSource));
        Source_rotaryEncoder.setEncoderValue(currentSource);
        EEPROM.write(EEPROM_INPUT_ADDRESS, currentSource);
        EEPROM.commit();
        break;

        case 103:
        lv_label_set_text(ui_Source, "AUX2");
        currentSource = 5;
        UpdateShiftRegister(pow(2, currentSource));
        Source_rotaryEncoder.setEncoderValue(currentSource);
        EEPROM.write(EEPROM_INPUT_ADDRESS, currentSource);
        EEPROM.commit();
        break;

        case 107:
        lv_label_set_text(ui_Source, "PROC");
        currentSource = 6;
        UpdateShiftRegister(pow(2, currentSource));
        Source_rotaryEncoder.setEncoderValue(currentSource);
        EEPROM.write(EEPROM_INPUT_ADDRESS, currentSource);
        EEPROM.commit();
        break;

        default:
        break;
      }//end of switch
      
    }//end of if
}

/////////////////////////////////////////////
////         记录开机后运行时间             ////
/////////////////////////////////////////////
void Tube_Used_Hours_Count()
{
  Time_Count = Hours_Counter.getLoopCount();
  if(Time_Count != Old_Time_Count)//每小时存储一次累计运行时间
  {
    Tube_Used_Hours++;
    itoa(Tube_Used_Hours, Tube_Used_Hours_Text, 10);
    lv_label_set_text(ui_Hour, Tube_Used_Hours_Text);
    EEPROM.write(EEPROM_TUBE_HRS_ADDRESS, Tube_Used_Hours);
    EEPROM.commit();

    Old_Time_Count = Time_Count;
  }
}

/////////////////////////////////////////////
////          重设胆管使用时间             ////
/////////////////////////////////////////////
void Tube_Used_Hours_Rest()
{
  
}

/////////////////////////////////////////////
////           调整左右声道平衡            ////
/////////////////////////////////////////////
void Balance_Adjust()
{
  Balance_Num = Volume_rotaryEncoder.readEncoder();
  itoa(Balance_Num, BalanceText, 10);
  lv_label_set_text(ui_BALNUMBER, BalanceText);
  lv_slider_set_value(ui_Slider2, Balance_Num, LV_ANIM_ON);
  //用设定的Balan_Num对左右声道进行平衡修正
  if(Balance_Num <= 0) //声道平衡设置时，左声道>右声道，用平衡数值修正左声道音量
  {
    MusesLeft.setVolume(currentVolumeLeft - Balance_Num, currentVolumeLeft - Balance_Num);
    MusesRight.setVolume(currentVolumeRight, currentVolumeRight);
  } //声道平衡设置时，左声道<右声道，用平衡数值修正右声道音量
  else
  {
    MusesLeft.setVolume(currentVolumeLeft, currentVolumeLeft);
    MusesRight.setVolume(currentVolumeRight + Balance_Num, currentVolumeRight + Balance_Num);
  }
}

/////////////////////////////////////////////
////    处理触屏平衡Slider变动的相应操作     ////
/////////////////////////////////////////////
void Balance_Slider_loop()
{
  
}
/////////////////////////////////////////////
////结束左右声道平衡调整恢复编码器音量调节功能 ////
/////////////////////////////////////////////
void Balance_Adjust_End()
{
  //Balance_Num = Volume_rotaryEncoder.readEncoder();//切换为“音量模式”前保存当前平衡数据
  Volume_rotaryEncoder.setBoundaries(-470, 0, false);
  Volume_rotaryEncoder.setAcceleration(80);
  Volume_rotaryEncoder.setEncoderValue(currentVolumeLeft);//切换为“音量模式”后编码器位置设置为上次的音量数据
  
  //改变显示屏的音量显示数字
  VolumeVal = map(currentVolumeLeft, -470, 0, 0, 100);
  itoa(VolumeVal, VolumeText, 10);
  lv_label_set_text(ui_Label2, VolumeText);
  lv_slider_set_value(ui_Slider1, VolumeVal, LV_ANIM_ON);
  Balance_OFF = false;
  EEPROM.write(EEPROM_BALANCE_ADDRESS, Balance_Num + 100);
  EEPROM.commit();
}

/////////////////////////////////////////////
////            Setup设置程序             ////
/////////////////////////////////////////////
void setup() {
  // 初始化串口
  Serial.begin(9600);

  //////////////////////////////////////////
  //初始化与Arduino的串口通讯
  Serial2.begin(115200, SERIAL_8N1, RXp, TXp);
  Serial2.setTimeout(100);

  //////////////////////////////////////////
  //初始化EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  //////////////////////////////////////////
  //初始化显示屏
    #ifdef GFX_PWD
      pinMode(GFX_PWD, OUTPUT);
      digitalWrite(GFX_PWD, HIGH);
    #endif

    // Init Display
    touch_init();//触摸屏初始化
    gfx->begin();
    gfx->fillScreen(BLACK);

    #ifdef GFX_BL
      pinMode(GFX_BL, OUTPUT);
      digitalWrite(GFX_BL, HIGH);
    #endif

    //////////////////////////////////////////
    //LVGL库初始化
    lv_init();

    screenWidth = gfx->width();
    screenHeight = gfx->height();

    #ifdef ESP32
      disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * 32, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
      // disp_draw_buf2 = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * 32, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    #else
      disp_draw_buf = (lv_color_t *)malloc(sizeof(lv_color_t) * screenWidth * 32);
    #endif
    if (!disp_draw_buf)
    {
      Serial.println("LVGL disp_draw_buf allocate failed!");
    }
    else
    {
    
    if (!disp_draw_buf2)
    {
      Serial.println("LVGL disp_draw_buf2 not allocated!");
      lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * 32);
    }
    else
    {
      lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, disp_draw_buf2, screenWidth * 32);
    }
    
    //Initialize the display//
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    //Change the following line to your display resolution//
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    //Initialize the (dummy) input device driver//
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );

    //////////////////////////////////////////
    //UI界面初始化
    ui_init();

    Serial.println( "Setup done" );
    }
  
  //////////////////////////////////////////
  //初始化音量旋转编码器
  Volume_rotaryEncoder.begin();
  Volume_rotaryEncoder.setup(Volume_readEncoderISR);
  //设置编码器是否循环
  bool circleValues = false;
  //设置编码器的取值范围（-470, 0），对应的音量衰减范围（-111.75dB, 0）
  Volume_rotaryEncoder.setBoundaries(-470, 0, circleValues);

  //设置是否使用加速功能
  //Volume_rotaryEncoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
  Volume_rotaryEncoder.setAcceleration(80); //数值越大加速越快

  //////////////////////////////////////////
  //初始化音源旋转编码器
  Source_rotaryEncoder.begin();
  Source_rotaryEncoder.setup(Source_readEncoderISR);
  //设置编码器的取值范围（0, 6），对应的音源通道编号
  Source_rotaryEncoder.setBoundaries(0, 6, circleValues);

  //音源旋转编码器不使用加速功能
  Source_rotaryEncoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
  
  //////////////////////////////////////////
  //设置74HC595寄存器控制引脚
  pinMode(LatchPin, OUTPUT);
  pinMode(ClockPin, OUTPUT);
  pinMode(DataPin, OUTPUT);

  //////////////////////////////////////////
  //初始化DG409反向芯片控制引脚
  pinMode(A0_Pin, OUTPUT);
  pinMode(A1_Pin, OUTPUT);
  
  //初始化Muses72323
  pinMode(CLK_PIN, OUTPUT);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  SPI.begin(CLK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  MusesLeft.begin();
  MusesRight.begin();  
  
  //设置Muses72323状态
  MusesLeft.setExternalClock(false); //不使用外部时钟，使用芯片内置的时钟
  MusesLeft.setZeroCrossingOn(true); //启用ZeroCrossing功能
  MusesRight.setExternalClock(false);
  MusesRight.setZeroCrossingOn(true);

  //读取保存的声道平衡数值，因范围是-100~+100，所以读出的值要减去100
  Balance_Num = EEPROM.read(EEPROM_BALANCE_ADDRESS)-100;

  //启动时设置为上次关机时音量，并同时将旋转编码器位置设定为相应数值，因EEPROM存储的数值范围是0-255，不能存储负值，所以读出后取反
  currentVolumeLeft  = -EEPROM.read(EEPROM_VOLUME_L_ADDRESS);
  currentVolumeRight = -EEPROM.read(EEPROM_VOLUME_R_ADDRESS);
  //用设定的Balan_Num对左右声道进行平衡修正
  if(Balance_Num <= 0) //声道平衡设置时，左声道>右声道，用平衡数值修正左声道音量
  {
    MusesLeft.setVolume(currentVolumeLeft - Balance_Num, currentVolumeLeft - Balance_Num);
    MusesRight.setVolume(currentVolumeRight, currentVolumeRight);
  } //声道平衡设置时，左声道<右声道，用平衡数值修正右声道音量
  else
  {
    MusesLeft.setVolume(currentVolumeLeft, currentVolumeLeft);
    MusesRight.setVolume(currentVolumeRight + Balance_Num, currentVolumeRight + Balance_Num);
  }
  Volume_rotaryEncoder.setEncoderValue(currentVolumeLeft);

  //启动时显示屏显示音量大小设置为上次关机时的数字
  VolumeVal = EEPROM.read(EEPROM_VOL_DISP_ADDRESS);
  itoa(VolumeVal, VolumeText, 10);
  lv_label_set_text(ui_Label2, VolumeText);
  lv_slider_set_value(ui_Slider1, VolumeVal, LV_ANIM_ON);

  //启动时设置为上次关机时的音源，并同时将旋转编码器位置设定为相应数值
  currentSource = EEPROM.read(EEPROM_INPUT_ADDRESS);
  Source_rotaryEncoder.setEncoderValue(currentSource);
  UpdateShiftRegister(pow(2, currentSource));
  switch(currentSource)
       {
         case 0:
         lv_label_set_text(ui_Source, "CD");
         break;

         case 1:
         lv_label_set_text(ui_Source, "TUNER");
         break; 
         
         case 2:
         lv_label_set_text(ui_Source, "PHONO");
         break;
         
         case 3:
         lv_label_set_text(ui_Source, "VIDEO");
         break;
         
         case 4:
         lv_label_set_text(ui_Source, "AUX1");
         break;
         
         case 5:
         lv_label_set_text(ui_Source, "AUX2");
         break;
         
         case 6:
         lv_label_set_text(ui_Source, "PROC");
         break;

         default:
         break;
       }
       
  //////////////////////////////////////////////////////////////
  //从板载ROM中读取前面板按键的上次关机时状态，并根据相应状态设定显示内容
  BAL_SE_ON = EEPROM.read(EEPROM_BAL_SE_ADDRESS);
  if(BAL_SE_ON == true)
  {
    lv_label_set_text(ui_Input, "SE");
  }
  else
  {
    lv_label_set_text(ui_Input, "BAL");
  }

  MONO_ON = EEPROM.read(EEPROM_MONO_ADDRESS);
  if(MONO_ON == true)
  {
    lv_label_set_text(ui_MonoOn, "MONO");
  }
  else
  {
    lv_label_set_text(ui_MonoOn, "");
  }
  
  INVERT_ON = EEPROM.read(EEPROM_INVERT_ADDRESS);
  if(INVERT_ON == true)
  {
    //控制DG409做相应的反向
    digitalWrite(A0_Pin, HIGH);
    digitalWrite(A1_Pin, LOW);
    lv_label_set_text(ui_InvertOn, "INVERT");
  }
  else
  {
    //控制DG409做相应的反向
    digitalWrite(A0_Pin, LOW);
    digitalWrite(A1_Pin, LOW);
    lv_label_set_text(ui_InvertOn, "");
  }
  
  MUTE_ON = EEPROM.read(EEPROM_MUTE_ADDRESS);
  if(MUTE_ON == true)
  {
    lv_label_set_text(ui_MuteOn, "MUTE ON");
  }
  else
  {
    lv_label_set_text(ui_MuteOn, "");
  }

  //////////////////////////////////////////////////////////////
  //开机运行时间计时功能的初始化
  Hours_Counter.setTimeOutTime(3600000);//每分钟计时一次
  Hours_Counter.reset();
  
  //从板载ROM中读上次关机时Tube的累计运行时间，并设定显示内容
  Tube_Used_Hours= EEPROM.read(EEPROM_TUBE_HRS_ADDRESS);
  itoa(Tube_Used_Hours, Tube_Used_Hours_Text, 10);
  lv_label_set_text(ui_Hour, Tube_Used_Hours_Text);

}

/////////////////////////////////////////////
////            loop主循环程序            ////
/////////////////////////////////////////////
void loop() 
{

  // 进行旋转编码器的循环
  if(Balance_ON == false && Balance_OFF == false) //编码器处于“音量调节模式”
  {
    Volume_rotary_loop();
  }
  
  Source_rotary_loop();

  //编码器处于“平衡调节模式”
  if(Balance_ON == true)
  {
    Balance_Adjust();
  }
  
  //当声道平衡调节结束时，编码器恢复音量调节的参数
  if(Balance_OFF == true)
  {
    Balance_Adjust_End();
  }


  //Arduino传输信息读取程序循环
  Arduino_Message_Read();

  //触屏滑动条信息处理程序的循环
  Volume_Slider_loop();
  
  //统计累计运行时间的循环
  Tube_Used_Hours_Count();
  
  //LVGL库的运行程序循环
  lv_timer_handler();


  delay(5);

}
