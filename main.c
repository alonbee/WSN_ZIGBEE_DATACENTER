


/* Includes ------------------------------------------------------------------*/
#include "stm8l10x.h"
#include "board.h"

#define USE_FULL_ASSERT

//#define WATCHDOG

#define WORK_TO_WAKE_RATIO  1  //工作与唤醒次数比例，每WORK_TO_WAKE_RATIO次唤醒（一般为30s）进行一次采集与上传，设置值1到32767

#define USE_SEC_ADDR    //定义使用地址设置第二方案-串口设置地址，如采用IO口地址设置方式，请注释掉此句
//#define SETTING_PANID     4372  //PANID，范围1到65535，注释掉此句则不设置
//#define SETTING_TX_POWER  19    //TX_POWER，范围0~19（13.3.6后可能到21），注释掉此句则不设置
//#define SETTING_CHANNEL   8192  //CHANNEL，范围2048到134215680，注释掉此句则不设置
//#define SETTING_POLL_RATE 3000  //POLL_RATE，范围0到65535，注释掉此句则不设置

//#define UI_STRING   //定义为字符串输出界面

/* Private defines -----------------------------------------------------------*/
volatile u8 i;
u16 Temprature;
u8  TH,TL,Config;
u8  AddrHi,AddrLo;
u16 Addr,SettingTemp;
u8  UARTSendDataBuf[32];
s32 WakeCount=-1;   //用于统计唤醒次数以实现每WORK_TO_WAKE_RATIO次唤醒一次采集上传

/* Private function prototypes -----------------------------------------------*/
void Delay(uint16_t nCount);
void DS18B20_Init(void);
u8 DS18B20_Read(void);
void DS18B20_Write(u8 Data);
void UART_Send_Data(u8 DataBuf[], u8 DataLength);

/* Private functions ---------------------------------------------------------*/

void main(void)
{
  /*----------IO口设置----------*/
  GPIO_Init(ADDR_LOW_PORT, GPIO_Pin_All, GPIO_Mode_Out_PP_Low_Slow);        //8位地址
  
  GPIO_Init(ADDR_HIGH_PORT0, ADDR_HIGH_PIN0, GPIO_Mode_Out_PP_Low_Slow);    //未使用口设置为输出低电平
  
  //P1.3/mode0 - 模块输入睡眠，stm8输出睡眠
  GPIO_Init( MODE0_PORT, MODE0_PIN, GPIO_Mode_Out_PP_Low_Slow);
  
  //P1.5/mode1 - 模块输出睡眠，stm8输入睡眠
  //Zigbee透传模块输出给stm8的唤醒信号为高电平，并在10ms后开始发送串口信号
  //由于stm8输入无内部下拉选项，因此只能设置为浮动输入
  //当不接Zigbee模块单独进行stm8程序调试时，浮动输入将可能导致持续发生中断，所以请务必接入Zigbee模块或在不接入模块时改为输入上拉
  GPIO_Init( MODE1_PORT, MODE1_PIN, GPIO_Mode_In_FL_IT);
  EXTI_SetPinSensitivity(EXTI_Pin_3, EXTI_Trigger_Rising);
  
  GPIO_Init(SENSOR_DATA_PORT, SENSOR_DATA_PIN, GPIO_Mode_Out_PP_High_Fast);   //传感器数据口拉高
  //等效为如下配置
  //SENSOR_DATA_PORT->ODR |= SENSOR_DATA_PIN;   //输出高电平
  //SENSOR_DATA_PORT->CR1 |= SENSOR_DATA_PIN;   //推挽输出
  //SENSOR_DATA_PORT->CR2 &= (uint8_t)(~(SENSOR_DATA_PIN));   //10MHz高速输出
  //SENSOR_DATA_PORT->DDR |= SENSOR_DATA_PIN;   //设置为输出
  
  //GPIO_Init(SENSOR_DATA_PORT, SENSOR_DATA_PIN, GPIO_Mode_In_PU_No_IT);   //传感器数据口输入上拉无中断
  //等效为如下配置
  //SENSOR_DATA_PORT->ODR |= SENSOR_DATA_PIN;   //输出高电平，此项在输入时无关紧要
  //SENSOR_DATA_PORT->CR1 |= SENSOR_DATA_PIN;   //输入上拉
  //SENSOR_DATA_PORT->CR2 &= (uint8_t)(~(SENSOR_DATA_PIN));   //无中断
  //SENSOR_DATA_PORT->DDR &= (uint8_t)(~(SENSOR_DATA_PIN));   //设置为输入
  //由此可见，当进行18B20的单总线输入输出切换时，只需要最后一行配置DDR即可以高速切换，这样就可以精确控制时间
  
  
  /*----------系统周期设置与内部模块使能----------*/
  CLK_DeInit();
  CLK_PeripheralClockConfig(CLK_Peripheral_AWU, ENABLE);      //使能唤醒
  CLK_MasterPrescalerConfig(CLK_MasterPrescaler_HSIDiv8);     //时钟8分频，2MHz
  
  
  /*----------唤醒初始化----------*/
  AWU_DeInit();
  
  
  /*----------串口初始化----------*/
  CLK_PeripheralClockConfig(CLK_Peripheral_USART, ENABLE);   //使能串口
  GPIO_ExternalPullUpConfig(GPIOC,GPIO_Pin_2|GPIO_Pin_3, ENABLE);   //拉高电平
  USART_DeInit();
  USART_Init(115200,                            //波特率115200
            USART_WordLength_8D,                //8位数据位
            USART_StopBits_1,                   //1位停止位
            USART_Parity_No,                    //无校验
            USART_Mode_Rx | USART_Mode_Tx);     //接收和发送使能
  //USART_ITConfig(USART_IT_TXE, ENABLE);         //使能发送中断
  USART_Cmd(ENABLE);    //串口开始工作
  
  
  /*----------看门狗初始化----------*/
  //注意！！看门狗的最长喂狗时限仅1~2秒，而本程序中单片机休眠时间最长设置为30秒
  //所以需要将Option Byte中的OPT4由默认的0x00改为0x01，以使休眠时看门狗暂停
  //Option Byte无法在程序中修改，只能通过烧写软件如STVP在烧写时由SWIM协议外部写入
  //因此在调试时看门狗功能无法实现
  //由SWIM外部烧写时请自行宏定义WATCHDOG以使看门狗生效
#ifdef  WATCHDOG
  IWDG_Enable();
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(IWDG_Prescaler_256);    //看门狗时限设为最长的1724.63ms
  IWDG_SetReload(0xFF);
  IWDG_ReloadCounter();
#endif
  
  
  /*----------以下进行Zigbee串口透传模块地址的设置----------*/
  //我们以STM8L 96bit唯一序列号中的8bit作为地址低位，地址高3bit放空为0x000，实际应用中可以采用拨码开关等方式进行设定
  //注意！如果采用IO口设置地址，IO口电平很大程度将影响设备功耗，为此做如下考虑
  //Zigbee串口透传模块的输入为内部下拉，如STM8L长期将IO口置于高电平将有近毫安级的电流损耗
  //因此Zigbee模块在开机后1秒时读入IO口设置地址，其余时间应将IO口置于低电平以降低功耗（见模块手册）
  //对于STM8L而言应在开机时设置地址IO口，并在0.5秒后将IO口恢复低电平
  //如采用拨码设置地址的方式，则首先应拨好设置地址，然后上电，并在0.5秒后将开关拨回至0
  
  //如果采用串口设置地址模式，则将地址设置IO口全部下拉，写入串口设置地址值后重启模块即可以完成地址设置
  
  AddrHi=0;   //地址高位置0  注意！！本程序放空了模块地址高位设置口，故地址高位为0，如需改为自己的程序请记得设置该地址并在下面的程序中设置对应的地址IO
  FLASH_SetProgrammingTime(FLASH_ProgramTime_Standard);
  FLASH_Unlock(FLASH_MemType_Program);
  //AddrLo=FLASH_ReadByte(0x4926);    //采用X-coordinator
  AddrLo=FLASH_ReadByte(0x4928);    //采用Y-coordinator
  //AddrLo=(FLASH_ReadByte(0x4926)<<4) | (FLASH_ReadByte(0x4928)&0x0F);   //X-coordinator和Y-coordinator各占4bit
  FLASH_Lock(FLASH_MemType_Program);
  Addr = ((u16)AddrHi<<8) | AddrLo; //将高低位地址组合成完整地址
  
  //等待模块初始化完成以能响应串口设置命令
  //注意！！这里
  for(u8 i=0; i<6; i++)
  {
    Delay(0xFFFF);    //每个Delay约200ms，总延时1秒以上，这里要有足量等待时间，因为模块上电一段时间后串口才开始工作接收数据
  }
  
#ifndef USE_SEC_ADDR    //未定义使用串口设置方案，则采用IO口设置方案

  //GPIO_Write(ADDR_HIGH_PORT, AddrHi);   //注意！！本程序放空了模块地址高位设置口，所以这里无需再拉IO口，如改为自己的程序请记得设置高位的地址并在此设置IO
  GPIO_Write(ADDR_LOW_PORT, AddrLo);    //写入低8位地址，高3位放空
  for(u8 i=0; i<3; i++)
  {
    Delay(0xFFFF);    //每个Delay约200ms，总延时0.5秒以上，因为在0.5秒时读入地址IO口，之后地址IO口状态不影响地址设置
  }
  //GPIO_Write(ADDR_HIGH_PORT, 0);
  GPIO_Write(ADDR_LOW_PORT, 0x00);    //将地址IO口置回0以降低功耗

#else   //定义了串口设置地址方案，采用串口设置地址

  //采用串口设置地址方式需下拉所有地址IO口使之为0x0000
  //本程序IO口初始化时均为下拉低电平，无需更改电平，注意如果拉高任何地址IO口，则串口地址无效，自动采用IO口所设置的地址
  //GPIO_Write(ADDR_LOW_PORT, 0);   //地址低位写入低电平，初始化已为低电平，无需写入
  //GPIO_Write(ADDR_HIGH_PORT, 0);  //地址高位写入低电平，初始化已为低电平，无需写入
  
  //以下将地址转换为串口设置命令字符串，首先输入设置命令UNI_SEC_ADDR和命令与设置值之间的空格
  UARTSendDataBuf[0]='U';UARTSendDataBuf[1]='N';UARTSendDataBuf[2]='I';UARTSendDataBuf[3]='_';UARTSendDataBuf[4]='S';
  UARTSendDataBuf[5]='E';UARTSendDataBuf[6]='C';UARTSendDataBuf[7]='_';UARTSendDataBuf[8]='A';UARTSendDataBuf[9]='D';
  UARTSendDataBuf[10]='D';UARTSendDataBuf[11]='R';UARTSendDataBuf[12]=' ';
  //然后将13bit地址转换为四字节字符串，如0x0789转化为“1929”，0x0089转化为“0137”，并将值填入UARTSendDataBuf[13]~UARTSendDataBuf[16]
  SettingTemp=Addr;
  for(u8 i=0; i<4; i++)
  {
    UARTSendDataBuf[16-i] = '0' + (SettingTemp % 10);
    SettingTemp /= 10;
  }
  //发送设置地址命令
  UART_Send_Data(UARTSendDataBuf, 17);
  
  Delay(0x3000);    //稍作等待
  
#endif
  
  /*----------以下进行其他参数设置，请在35行处取消需要设置值的#define注释并设置相应值----------*/
  
#ifdef  SETTING_PANID   //PANID设置，注意设置的PANID要与主机和路由器一致才可正常工作
  assert_param(SETTING_PANID>=1 && SETTING_PANID<=65535);
  
  UARTSendDataBuf[0]='P';UARTSendDataBuf[1]='A';UARTSendDataBuf[2]='N';UARTSendDataBuf[3]='I';UARTSendDataBuf[4]='D';
  UARTSendDataBuf[5]=' ';
  //然后将两字节PANID转换为五字节字符串，并将值填入UARTSendDataBuf[6]~UARTSendDataBuf[10]
  SettingTemp=SETTING_PANID;
  for(u8 i=0; i<5; i++)
  {
    UARTSendDataBuf[10-i] = '0' + (SettingTemp % 10);
    SettingTemp /= 10;
  }
  //发送设置命令
  UART_Send_Data(UARTSendDataBuf, 11);
  
  Delay(0x3000);    //稍作等待  
#endif  //SETTING_PANID
  
#ifdef  SETTING_TX_POWER   //TX_POWER发射功率设置
  assert_param(SETTING_TX_POWER>=0 && SETTING_TX_POWER<=21);
  
  UARTSendDataBuf[0]='T';UARTSendDataBuf[1]='X';UARTSendDataBuf[2]='_';UARTSendDataBuf[3]='P';UARTSendDataBuf[4]='O';
  UARTSendDataBuf[5]='W';UARTSendDataBuf[6]='E';UARTSendDataBuf[7]='R';UARTSendDataBuf[8]=' ';
  //然后将TX_POWER转换为两字节字符串，并将值填入UARTSendDataBuf[9]~UARTSendDataBuf[10]
  SettingTemp=SETTING_TX_POWER;
  for(u8 i=0; i<2; i++)
  {
    UARTSendDataBuf[10-i] = '0' + (SettingTemp % 10);
    SettingTemp /= 10;
  }
  //发送设置命令
  UART_Send_Data(UARTSendDataBuf, 11);
  
  Delay(0x3000);    //稍作等待  
#endif  //SETTING_TX_POWER
  
#ifdef  SETTING_CHANNEL   //CHANNEL设置，注意设置的CHANNEL要与主机和路由器一致才可正常工作
  assert_param(SETTING_CHANNEL>=2048 && SETTING_CHANNEL<=134215680);
  
  UARTSendDataBuf[0]='C';UARTSendDataBuf[1]='H';UARTSendDataBuf[2]='A';UARTSendDataBuf[3]='N';UARTSendDataBuf[4]='N';
  UARTSendDataBuf[5]='E';UARTSendDataBuf[6]='L';UARTSendDataBuf[7]=' ';
  //然后将CHANNEL转换为十字节字符串，并将值填入UARTSendDataBuf[8]~UARTSendDataBuf[17]
  SettingTemp=SETTING_CHANNEL;
  for(u8 i=0; i<10; i++)
  {
    UARTSendDataBuf[17-i] = '0' + (SettingTemp % 10);
    SettingTemp /= 10;
  }
  //发送设置命令
  UART_Send_Data(UARTSendDataBuf, 18);
  
  Delay(0x3000);    //稍作等待  
#endif  //SETTING_CHANNEL
  
#ifdef  SETTING_POLL_RATE   //POLL_RATE终端定期唤醒查询数据时限 
  assert_param(SETTING_POLL_RATE>=0 && SETTING_POLL_RATE<=65535);
  
  UARTSendDataBuf[0]='P';UARTSendDataBuf[1]='O';UARTSendDataBuf[2]='L';UARTSendDataBuf[3]='L';UARTSendDataBuf[4]='_';
  UARTSendDataBuf[5]='R';UARTSendDataBuf[6]='A';UARTSendDataBuf[7]='T';UARTSendDataBuf[8]='E';UARTSendDataBuf[9]=' ';
  //然后将POLL_RATE转换为五字节字符串，并将值填入UARTSendDataBuf[10]~UARTSendDataBuf[14]
  SettingTemp=SETTING_POLL_RATE;
  for(u8 i=0; i<5; i++)
  {
    UARTSendDataBuf[14-i] = '0' + (SettingTemp % 10);
    SettingTemp /= 10;
  }
  //发送设置命令
  UART_Send_Data(UARTSendDataBuf, 15);
  
  Delay(0x3000);    //稍作等待  
#endif  //SETTING_POLL_RATE
  
  
  //写入重启模块命令PW_RESET 1
  UARTSendDataBuf[0]='P';UARTSendDataBuf[1]='W';UARTSendDataBuf[2]='_';UARTSendDataBuf[3]='R';UARTSendDataBuf[4]='E';
  UARTSendDataBuf[5]='S';UARTSendDataBuf[6]='E';UARTSendDataBuf[7]='T';UARTSendDataBuf[8]=' ';UARTSendDataBuf[9]='1';
  //发送重启命令
  UART_Send_Data(UARTSendDataBuf, 10);
  
  //等待模块重启，重启0.5秒时读入地址设置IO口为0x0000，进入串口设置地址模式，读取刚才我们串口所设置的地址并工作在该地址
  for(u8 i=0; i<6; i++)
  {
    Delay(0xFFFF);    //每个Delay约200ms，总延时1秒以上，这里要有足量等待时间，因为模块上电一段时间后串口才开始工作接收数据
  }
    
  
  //以下进行18B20采样精度设置
  DS18B20_Init();
  DS18B20_Write(0xCC);  //跳过ROM操作
  DS18B20_Write(0xBE);  //读配置
  //读出TH，TL值以便设置时将其原样写入不进行改变
  Temprature=DS18B20_Read();    //低8位
  Temprature=Temprature | (DS18B20_Read()<<8);  //高8位
  TH=DS18B20_Read();
  TL=DS18B20_Read();
  Config=DS18B20_Read();
  
  DS18B20_Init();
  DS18B20_Write(0xCC);  //跳过ROM操作
  DS18B20_Write(0x4E);  //写配置
  DS18B20_Write(TH);    //原样写入不改变
  DS18B20_Write(TL);    //原样写入不改变
  DS18B20_Write(0x1F);  //采用较低分辨率能大幅降低功耗 1F：9位，93.75ms 3F：10位，187.5ms 5F：11位，375ms 7F：12位，750ms
    
#ifdef UI_STRING    //字符串输出界面
  //格式 ID:1234 T:+025.0
  //0~3，AA XX XX 55，目的地址
  //4~6, 3字节头
  //7~10, 4字节地址
  //11~13, 3字节中
  //14，符号位
  //15~17，整数位
  //18，小数点
  //19，小数位
  //20，1字节尾
  UARTSendDataBuf[0]=0xAA;UARTSendDataBuf[1]=0x00;UARTSendDataBuf[2]=0x00;UARTSendDataBuf[3]=0x55;
  UARTSendDataBuf[4]='I';UARTSendDataBuf[5]='D';UARTSendDataBuf[6]=':';
  //将本机地址转换为十字节字符串，并将值填入UARTSendDataBuf[10]~UARTSendDataBuf[19]
  SettingTemp=Addr;
  for(u8 i=0; i<4; i++)
  {
    UARTSendDataBuf[10-i] = '0' + (SettingTemp % 10);
    SettingTemp /= 10;
  }
  //将头部的所有0替换为空格
  i=7;
  while(UARTSendDataBuf[i]=='0')
  {
    UARTSendDataBuf[i++]=' ';
  }
  UARTSendDataBuf[11]=' ';UARTSendDataBuf[12]='T';UARTSendDataBuf[13]=':';
  UARTSendDataBuf[18]='.';
  UARTSendDataBuf[20]=' ';
#endif


  disableInterrupts();
  
  /* Infinite loop */
  while (1)
  {
#ifdef  WATCHDOG
    IWDG_ReloadCounter();   //喂狗
#endif
    
    if(WakeCount<=0)    //每WORK_TO_WAKE_RATIO次唤醒采集一次传感器数据并上传，之所以采用初值-1这里<=0后面++并%WORK_TO_WAKE_RATIO的方式，是为了在开机时能连采两次使传感器工作正常
    {
      DS18B20_Init();
      DS18B20_Write(0xCC);  //跳过ROM操作
      DS18B20_Write(0x44);  //温度转换
      
      //9位分辨率93.75ms采样时间情况下，进入睡眠128ms，等待18B20采样完成
      //注意！如果提采样高分辨率会延长采样时间，此时应延长睡眠时间使之大于采样时间，否则会出错
      enableInterrupts();
      AWU_Init(AWU_Timebase_128ms);
      AWU_ReInitCounter();
      AWU_Cmd(ENABLE);
      halt();
      disableInterrupts();
      
#ifdef  WATCHDOG
      IWDG_ReloadCounter();   //喂狗
#endif
  
      while(GPIO_ReadInputDataBit(SENSOR_DATA_PORT, SENSOR_DATA_PIN)==RESET);   //如18B20还未采样完成则等待
      
      DS18B20_Init();
      DS18B20_Write(0xCC);  //跳过ROM操作
      DS18B20_Write(0xBE);  //读取RAM
      Temprature=DS18B20_Read();    //低8位
      Temprature=Temprature | (DS18B20_Read()<<8);  //高8位
      TH=DS18B20_Read();
      TL=DS18B20_Read();
      Config=DS18B20_Read();
      DS18B20_Init();       //reset，终止读写其他寄存器
      
#ifndef UI_STRING
      //将采集到的温度数据发送到主机
      UARTSendDataBuf[0]=0xAA;      //4字节目的地址包头，0xAA 目的地址高位 目的地址低位 0x55，主机目的地址高低位均为0x00
      UARTSendDataBuf[1]=0x00;
      UARTSendDataBuf[2]=0x00;
      UARTSendDataBuf[3]=0x55;
      UARTSendDataBuf[4]=AddrHi;    //发送给主机本机地址以方便主机的寻址，地址高位，这里我们放空了地址高位，模块的3位地址高位默认下拉设置为0
      UARTSendDataBuf[5]=AddrLo;    //发送给主机本机地址低位
      UARTSendDataBuf[6]=(u8)(Temprature>>8);     //温度高位
      UARTSendDataBuf[7]=(u8)(Temprature&0x00FF); //温度低位
      UART_Send_Data(UARTSendDataBuf, 8);         //将数据发送到主机
#else
      //字符串格式
      UARTSendDataBuf[14]='+';
      if(Temprature & 0x8000)   //最高位为1，负的，取反加一
      {
        UARTSendDataBuf[14]='-';
        Temprature = (~Temprature)+1;
      }
      //小数部分
      UARTSendDataBuf[19]='0';
      if(Temprature & 0x000F)
      {
        UARTSendDataBuf[19]='5';
      }
      //整数部分
      SettingTemp=Temprature>>4;
      for(u8 i=0; i<3; i++)
      {
        UARTSendDataBuf[17-i] = '0' + (SettingTemp % 10);
        SettingTemp /= 10;
      }
      UART_Send_Data(UARTSendDataBuf, 21);         //将数据发送到主机
#endif
    }

#ifdef  WATCHDOG
    IWDG_ReloadCounter();   //喂狗
#endif
    
    WakeCount++;
    WakeCount = WakeCount % WORK_TO_WAKE_RATIO;   //每WORK_TO_WAKE_RATIO次唤醒采集一次传感器数据并上传（约WORK_TO_WAKE_RATIO*30秒采集上传一次）

    enableInterrupts();
    AWU_Init(AWU_Timebase_30s);   //30秒睡眠，注意！！实测睡眠时间误差较大
    AWU_ReInitCounter();
    AWU_Cmd(ENABLE);
    halt();
    disableInterrupts();
    
  }

}

/**
  * @brief  Delay.
  * @param  nCount
  * @retval None
  */
void Delay(uint16_t nCount)   //实测每个count对应6指令周期，2MHz下对应3us
{
    /* Decrement nCount value */
    while (nCount != 0)
    {
        nCount--;
    }
}

/**
  * @brief  18B20 Init.
  * @param  None
  * @retval None
  */
void DS18B20_Init(void)
{
  SENSOR_DATA_PORT->ODR &= (uint8_t)(~(SENSOR_DATA_PIN));   //拉低总线，输出低电平
  SENSOR_DATA_PORT->DDR |= SENSOR_DATA_PIN;
  DELAY_500US();DELAY_100US();    //拉低600us
  SENSOR_DATA_PORT->DDR &= (uint8_t)(~(SENSOR_DATA_PIN));   //释放总线，设置为输入
  DELAY_50US();       //稍作等待
  while(GPIO_ReadInputDataBit(SENSOR_DATA_PORT, SENSOR_DATA_PIN)==SET);     //等待18B20拉低总线
  while(GPIO_ReadInputDataBit(SENSOR_DATA_PORT, SENSOR_DATA_PIN)==RESET);   //等待18B20释放总线 
  DELAY_50US();    //等待RX整个过程完成
}

/**
  * @brief  18B20 Read.
  * @param  None
  * @retval None
  */
u8 DS18B20_Read(void)
{
  u8 Data;
  
  for(u8 i=0;i<8;i++)
  {
    Data >>= 1;
    
    SENSOR_DATA_PORT->ODR &= (uint8_t)(~(SENSOR_DATA_PIN));   //拉低总线，输出低电平
    SENSOR_DATA_PORT->DDR |= SENSOR_DATA_PIN;
    DELAY_1US();DELAY_1US();                 //延时2us
    SENSOR_DATA_PORT->DDR &= (uint8_t)(~(SENSOR_DATA_PIN));   //释放总线，设置为输入
    DELAY_10US(); DELAY_1US();DELAY_1US();   //等待12us
    
    if(GPIO_ReadInputDataBit(SENSOR_DATA_PORT, SENSOR_DATA_PIN)==SET)   //读取电平
    {
      Data |= 0x80;
    }
    
    DELAY_10US();DELAY_10US();DELAY_10US();DELAY_10US();DELAY_5US();    //等待45us
  }
  
  return Data;
}

/**
  * @brief  18B20 Write.
  * @param  None
  * @retval None
  */
void DS18B20_Write(u8 Data)
{
  for(u8 i=0;i<8;i++)
  {
    SENSOR_DATA_PORT->ODR &= (uint8_t)(~(SENSOR_DATA_PIN));   //拉低总线，输出低电平
    SENSOR_DATA_PORT->DDR |= SENSOR_DATA_PIN;
    DELAY_10US();DELAY_5US();     //等待15us
    
    if(Data & 0x01)     //如果写入1
    {
      SENSOR_DATA_PORT->DDR &= (uint8_t)(~(SENSOR_DATA_PIN));   //释放总线，设置为输入
    }
    DELAY_10US();DELAY_10US();DELAY_10US();DELAY_10US();DELAY_5US();     //等待45us
    
    SENSOR_DATA_PORT->DDR &= (uint8_t)(~(SENSOR_DATA_PIN));   //释放总线，设置为输入
    DELAY_5US();    //恢复时间
    
    Data >>= 1;
  }
}

/**
  * @brief  发送给模块数据或命令
  * @param  DataBuf - 发送给模块的数据，DataLength - 发送长度
  * @retval None
  */
void UART_Send_Data(u8 DataBuf[], u8 DataLength)
{
  u8 idx=0;
  
  USART_Cmd(ENABLE);    //使能串口
  
  //选择如下两种唤醒方式之一
  
  //第一种，IO口唤醒，将唤醒口拉高后等待3ms，开始发送串口信号
  //在发送完一个数据包后等待20ms以上发送下一个数据包，在所有串口信号发送完毕后拉低唤醒口
  GPIO_WriteBit(MODE0_PORT, MODE0_PIN, SET);    //唤醒串口
  Delay(0x400);         //等待3ms
  
  //第二种，串口直接唤醒，发送一字节0x55唤醒串口后等待10ms发送串口数据，在发送完一个数据包后等待20~150ms发送下一个数据包
  //如果超过150ms未发送数据包，则需要重新进行唤醒
  //USART_SendData8(0x55);    //发送唤醒字节
  //while((USART->SR & 0x80) == 0);      //等待发送缓存空
  //Delay(0xD00);         //等待10ms
  
  for(idx=0; idx<DataLength; idx++)
  {
    USART_SendData8(DataBuf[idx]);    //发送当前字符
    while((USART->SR & 0x80) == 0);   //等待发送缓存空
  }
  
  while((USART->SR & 0x40) == 0);       //等待发送完毕
  GPIO_WriteBit(MODE0_PORT, MODE0_PIN, RESET);    //让模块重新进入睡眠
  
  //USART_Cmd(DISABLE);
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

