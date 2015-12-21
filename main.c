


/* Includes ------------------------------------------------------------------*/
#include "stm8l10x.h"
#include "board.h"

#define USE_FULL_ASSERT

//#define WATCHDOG

#define WORK_TO_WAKE_RATIO  1  //�����뻽�Ѵ���������ÿWORK_TO_WAKE_RATIO�λ��ѣ�һ��Ϊ30s������һ�βɼ����ϴ�������ֵ1��32767

#define USE_SEC_ADDR    //����ʹ�õ�ַ���õڶ�����-�������õ�ַ�������IO�ڵ�ַ���÷�ʽ����ע�͵��˾�
//#define SETTING_PANID     4372  //PANID����Χ1��65535��ע�͵��˾�������
//#define SETTING_TX_POWER  19    //TX_POWER����Χ0~19��13.3.6����ܵ�21����ע�͵��˾�������
//#define SETTING_CHANNEL   8192  //CHANNEL����Χ2048��134215680��ע�͵��˾�������
//#define SETTING_POLL_RATE 3000  //POLL_RATE����Χ0��65535��ע�͵��˾�������

//#define UI_STRING   //����Ϊ�ַ����������

/* Private defines -----------------------------------------------------------*/
volatile u8 i;
u16 Temprature;
u8  TH,TL,Config;
u8  AddrHi,AddrLo;
u16 Addr,SettingTemp;
u8  UARTSendDataBuf[32];
s32 WakeCount=-1;   //����ͳ�ƻ��Ѵ�����ʵ��ÿWORK_TO_WAKE_RATIO�λ���һ�βɼ��ϴ�

/* Private function prototypes -----------------------------------------------*/
void Delay(uint16_t nCount);
void DS18B20_Init(void);
u8 DS18B20_Read(void);
void DS18B20_Write(u8 Data);
void UART_Send_Data(u8 DataBuf[], u8 DataLength);

/* Private functions ---------------------------------------------------------*/

void main(void)
{
  /*----------IO������----------*/
  GPIO_Init(ADDR_LOW_PORT, GPIO_Pin_All, GPIO_Mode_Out_PP_Low_Slow);        //8λ��ַ
  
  GPIO_Init(ADDR_HIGH_PORT0, ADDR_HIGH_PIN0, GPIO_Mode_Out_PP_Low_Slow);    //δʹ�ÿ�����Ϊ����͵�ƽ
  
  //P1.3/mode0 - ģ������˯�ߣ�stm8���˯��
  GPIO_Init( MODE0_PORT, MODE0_PIN, GPIO_Mode_Out_PP_Low_Slow);
  
  //P1.5/mode1 - ģ�����˯�ߣ�stm8����˯��
  //Zigbee͸��ģ�������stm8�Ļ����ź�Ϊ�ߵ�ƽ������10ms��ʼ���ʹ����ź�
  //����stm8�������ڲ�����ѡ����ֻ������Ϊ��������
  //������Zigbeeģ�鵥������stm8�������ʱ���������뽫���ܵ��³��������жϣ���������ؽ���Zigbeeģ����ڲ�����ģ��ʱ��Ϊ��������
  GPIO_Init( MODE1_PORT, MODE1_PIN, GPIO_Mode_In_FL_IT);
  EXTI_SetPinSensitivity(EXTI_Pin_3, EXTI_Trigger_Rising);
  
  GPIO_Init(SENSOR_DATA_PORT, SENSOR_DATA_PIN, GPIO_Mode_Out_PP_High_Fast);   //���������ݿ�����
  //��ЧΪ��������
  //SENSOR_DATA_PORT->ODR |= SENSOR_DATA_PIN;   //����ߵ�ƽ
  //SENSOR_DATA_PORT->CR1 |= SENSOR_DATA_PIN;   //�������
  //SENSOR_DATA_PORT->CR2 &= (uint8_t)(~(SENSOR_DATA_PIN));   //10MHz�������
  //SENSOR_DATA_PORT->DDR |= SENSOR_DATA_PIN;   //����Ϊ���
  
  //GPIO_Init(SENSOR_DATA_PORT, SENSOR_DATA_PIN, GPIO_Mode_In_PU_No_IT);   //���������ݿ������������ж�
  //��ЧΪ��������
  //SENSOR_DATA_PORT->ODR |= SENSOR_DATA_PIN;   //����ߵ�ƽ������������ʱ�޹ؽ�Ҫ
  //SENSOR_DATA_PORT->CR1 |= SENSOR_DATA_PIN;   //��������
  //SENSOR_DATA_PORT->CR2 &= (uint8_t)(~(SENSOR_DATA_PIN));   //���ж�
  //SENSOR_DATA_PORT->DDR &= (uint8_t)(~(SENSOR_DATA_PIN));   //����Ϊ����
  //�ɴ˿ɼ���������18B20�ĵ�������������л�ʱ��ֻ��Ҫ���һ������DDR�����Ը����л��������Ϳ��Ծ�ȷ����ʱ��
  
  
  /*----------ϵͳ�����������ڲ�ģ��ʹ��----------*/
  CLK_DeInit();
  CLK_PeripheralClockConfig(CLK_Peripheral_AWU, ENABLE);      //ʹ�ܻ���
  CLK_MasterPrescalerConfig(CLK_MasterPrescaler_HSIDiv8);     //ʱ��8��Ƶ��2MHz
  
  
  /*----------���ѳ�ʼ��----------*/
  AWU_DeInit();
  
  
  /*----------���ڳ�ʼ��----------*/
  CLK_PeripheralClockConfig(CLK_Peripheral_USART, ENABLE);   //ʹ�ܴ���
  GPIO_ExternalPullUpConfig(GPIOC,GPIO_Pin_2|GPIO_Pin_3, ENABLE);   //���ߵ�ƽ
  USART_DeInit();
  USART_Init(115200,                            //������115200
            USART_WordLength_8D,                //8λ����λ
            USART_StopBits_1,                   //1λֹͣλ
            USART_Parity_No,                    //��У��
            USART_Mode_Rx | USART_Mode_Tx);     //���պͷ���ʹ��
  //USART_ITConfig(USART_IT_TXE, ENABLE);         //ʹ�ܷ����ж�
  USART_Cmd(ENABLE);    //���ڿ�ʼ����
  
  
  /*----------���Ź���ʼ��----------*/
  //ע�⣡�����Ź����ι��ʱ�޽�1~2�룬���������е�Ƭ������ʱ�������Ϊ30��
  //������Ҫ��Option Byte�е�OPT4��Ĭ�ϵ�0x00��Ϊ0x01����ʹ����ʱ���Ź���ͣ
  //Option Byte�޷��ڳ������޸ģ�ֻ��ͨ����д�����STVP����дʱ��SWIMЭ���ⲿд��
  //����ڵ���ʱ���Ź������޷�ʵ��
  //��SWIM�ⲿ��дʱ�����к궨��WATCHDOG��ʹ���Ź���Ч
#ifdef  WATCHDOG
  IWDG_Enable();
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(IWDG_Prescaler_256);    //���Ź�ʱ����Ϊ���1724.63ms
  IWDG_SetReload(0xFF);
  IWDG_ReloadCounter();
#endif
  
  
  /*----------���½���Zigbee����͸��ģ���ַ������----------*/
  //������STM8L 96bitΨһ���к��е�8bit��Ϊ��ַ��λ����ַ��3bit�ſ�Ϊ0x000��ʵ��Ӧ���п��Բ��ò��뿪�صȷ�ʽ�����趨
  //ע�⣡�������IO�����õ�ַ��IO�ڵ�ƽ�ܴ�̶Ƚ�Ӱ���豸���ģ�Ϊ�������¿���
  //Zigbee����͸��ģ�������Ϊ�ڲ���������STM8L���ڽ�IO�����ڸߵ�ƽ���н��������ĵ������
  //���Zigbeeģ���ڿ�����1��ʱ����IO�����õ�ַ������ʱ��Ӧ��IO�����ڵ͵�ƽ�Խ��͹��ģ���ģ���ֲᣩ
  //����STM8L����Ӧ�ڿ���ʱ���õ�ַIO�ڣ�����0.5���IO�ڻָ��͵�ƽ
  //����ò������õ�ַ�ķ�ʽ��������Ӧ�������õ�ַ��Ȼ���ϵ磬����0.5��󽫿��ز�����0
  
  //������ô������õ�ַģʽ���򽫵�ַ����IO��ȫ��������д�봮�����õ�ֵַ������ģ�鼴������ɵ�ַ����
  
  AddrHi=0;   //��ַ��λ��0  ע�⣡��������ſ���ģ���ַ��λ���ÿڣ��ʵ�ַ��λΪ0�������Ϊ�Լ��ĳ�����ǵ����øõ�ַ��������ĳ��������ö�Ӧ�ĵ�ַIO
  FLASH_SetProgrammingTime(FLASH_ProgramTime_Standard);
  FLASH_Unlock(FLASH_MemType_Program);
  //AddrLo=FLASH_ReadByte(0x4926);    //����X-coordinator
  AddrLo=FLASH_ReadByte(0x4928);    //����Y-coordinator
  //AddrLo=(FLASH_ReadByte(0x4926)<<4) | (FLASH_ReadByte(0x4928)&0x0F);   //X-coordinator��Y-coordinator��ռ4bit
  FLASH_Lock(FLASH_MemType_Program);
  Addr = ((u16)AddrHi<<8) | AddrLo; //���ߵ�λ��ַ��ϳ�������ַ
  
  //�ȴ�ģ���ʼ�����������Ӧ������������
  //ע�⣡������
  for(u8 i=0; i<6; i++)
  {
    Delay(0xFFFF);    //ÿ��DelayԼ200ms������ʱ1�����ϣ�����Ҫ�������ȴ�ʱ�䣬��Ϊģ���ϵ�һ��ʱ��󴮿ڲſ�ʼ������������
  }
  
#ifndef USE_SEC_ADDR    //δ����ʹ�ô������÷����������IO�����÷���

  //GPIO_Write(ADDR_HIGH_PORT, AddrHi);   //ע�⣡��������ſ���ģ���ַ��λ���ÿڣ�����������������IO�ڣ����Ϊ�Լ��ĳ�����ǵ����ø�λ�ĵ�ַ���ڴ�����IO
  GPIO_Write(ADDR_LOW_PORT, AddrLo);    //д���8λ��ַ����3λ�ſ�
  for(u8 i=0; i<3; i++)
  {
    Delay(0xFFFF);    //ÿ��DelayԼ200ms������ʱ0.5�����ϣ���Ϊ��0.5��ʱ�����ַIO�ڣ�֮���ַIO��״̬��Ӱ���ַ����
  }
  //GPIO_Write(ADDR_HIGH_PORT, 0);
  GPIO_Write(ADDR_LOW_PORT, 0x00);    //����ַIO���û�0�Խ��͹���

#else   //�����˴������õ�ַ���������ô������õ�ַ

  //���ô������õ�ַ��ʽ���������е�ַIO��ʹ֮Ϊ0x0000
  //������IO�ڳ�ʼ��ʱ��Ϊ�����͵�ƽ��������ĵ�ƽ��ע����������κε�ַIO�ڣ��򴮿ڵ�ַ��Ч���Զ�����IO�������õĵ�ַ
  //GPIO_Write(ADDR_LOW_PORT, 0);   //��ַ��λд��͵�ƽ����ʼ����Ϊ�͵�ƽ������д��
  //GPIO_Write(ADDR_HIGH_PORT, 0);  //��ַ��λд��͵�ƽ����ʼ����Ϊ�͵�ƽ������д��
  
  //���½���ַת��Ϊ�������������ַ���������������������UNI_SEC_ADDR������������ֵ֮��Ŀո�
  UARTSendDataBuf[0]='U';UARTSendDataBuf[1]='N';UARTSendDataBuf[2]='I';UARTSendDataBuf[3]='_';UARTSendDataBuf[4]='S';
  UARTSendDataBuf[5]='E';UARTSendDataBuf[6]='C';UARTSendDataBuf[7]='_';UARTSendDataBuf[8]='A';UARTSendDataBuf[9]='D';
  UARTSendDataBuf[10]='D';UARTSendDataBuf[11]='R';UARTSendDataBuf[12]=' ';
  //Ȼ��13bit��ַת��Ϊ���ֽ��ַ�������0x0789ת��Ϊ��1929����0x0089ת��Ϊ��0137��������ֵ����UARTSendDataBuf[13]~UARTSendDataBuf[16]
  SettingTemp=Addr;
  for(u8 i=0; i<4; i++)
  {
    UARTSendDataBuf[16-i] = '0' + (SettingTemp % 10);
    SettingTemp /= 10;
  }
  //�������õ�ַ����
  UART_Send_Data(UARTSendDataBuf, 17);
  
  Delay(0x3000);    //�����ȴ�
  
#endif
  
  /*----------���½��������������ã�����35�д�ȡ����Ҫ����ֵ��#defineע�Ͳ�������Ӧֵ----------*/
  
#ifdef  SETTING_PANID   //PANID���ã�ע�����õ�PANIDҪ��������·����һ�²ſ���������
  assert_param(SETTING_PANID>=1 && SETTING_PANID<=65535);
  
  UARTSendDataBuf[0]='P';UARTSendDataBuf[1]='A';UARTSendDataBuf[2]='N';UARTSendDataBuf[3]='I';UARTSendDataBuf[4]='D';
  UARTSendDataBuf[5]=' ';
  //Ȼ�����ֽ�PANIDת��Ϊ���ֽ��ַ���������ֵ����UARTSendDataBuf[6]~UARTSendDataBuf[10]
  SettingTemp=SETTING_PANID;
  for(u8 i=0; i<5; i++)
  {
    UARTSendDataBuf[10-i] = '0' + (SettingTemp % 10);
    SettingTemp /= 10;
  }
  //������������
  UART_Send_Data(UARTSendDataBuf, 11);
  
  Delay(0x3000);    //�����ȴ�  
#endif  //SETTING_PANID
  
#ifdef  SETTING_TX_POWER   //TX_POWER���书������
  assert_param(SETTING_TX_POWER>=0 && SETTING_TX_POWER<=21);
  
  UARTSendDataBuf[0]='T';UARTSendDataBuf[1]='X';UARTSendDataBuf[2]='_';UARTSendDataBuf[3]='P';UARTSendDataBuf[4]='O';
  UARTSendDataBuf[5]='W';UARTSendDataBuf[6]='E';UARTSendDataBuf[7]='R';UARTSendDataBuf[8]=' ';
  //Ȼ��TX_POWERת��Ϊ���ֽ��ַ���������ֵ����UARTSendDataBuf[9]~UARTSendDataBuf[10]
  SettingTemp=SETTING_TX_POWER;
  for(u8 i=0; i<2; i++)
  {
    UARTSendDataBuf[10-i] = '0' + (SettingTemp % 10);
    SettingTemp /= 10;
  }
  //������������
  UART_Send_Data(UARTSendDataBuf, 11);
  
  Delay(0x3000);    //�����ȴ�  
#endif  //SETTING_TX_POWER
  
#ifdef  SETTING_CHANNEL   //CHANNEL���ã�ע�����õ�CHANNELҪ��������·����һ�²ſ���������
  assert_param(SETTING_CHANNEL>=2048 && SETTING_CHANNEL<=134215680);
  
  UARTSendDataBuf[0]='C';UARTSendDataBuf[1]='H';UARTSendDataBuf[2]='A';UARTSendDataBuf[3]='N';UARTSendDataBuf[4]='N';
  UARTSendDataBuf[5]='E';UARTSendDataBuf[6]='L';UARTSendDataBuf[7]=' ';
  //Ȼ��CHANNELת��Ϊʮ�ֽ��ַ���������ֵ����UARTSendDataBuf[8]~UARTSendDataBuf[17]
  SettingTemp=SETTING_CHANNEL;
  for(u8 i=0; i<10; i++)
  {
    UARTSendDataBuf[17-i] = '0' + (SettingTemp % 10);
    SettingTemp /= 10;
  }
  //������������
  UART_Send_Data(UARTSendDataBuf, 18);
  
  Delay(0x3000);    //�����ȴ�  
#endif  //SETTING_CHANNEL
  
#ifdef  SETTING_POLL_RATE   //POLL_RATE�ն˶��ڻ��Ѳ�ѯ����ʱ�� 
  assert_param(SETTING_POLL_RATE>=0 && SETTING_POLL_RATE<=65535);
  
  UARTSendDataBuf[0]='P';UARTSendDataBuf[1]='O';UARTSendDataBuf[2]='L';UARTSendDataBuf[3]='L';UARTSendDataBuf[4]='_';
  UARTSendDataBuf[5]='R';UARTSendDataBuf[6]='A';UARTSendDataBuf[7]='T';UARTSendDataBuf[8]='E';UARTSendDataBuf[9]=' ';
  //Ȼ��POLL_RATEת��Ϊ���ֽ��ַ���������ֵ����UARTSendDataBuf[10]~UARTSendDataBuf[14]
  SettingTemp=SETTING_POLL_RATE;
  for(u8 i=0; i<5; i++)
  {
    UARTSendDataBuf[14-i] = '0' + (SettingTemp % 10);
    SettingTemp /= 10;
  }
  //������������
  UART_Send_Data(UARTSendDataBuf, 15);
  
  Delay(0x3000);    //�����ȴ�  
#endif  //SETTING_POLL_RATE
  
  
  //д������ģ������PW_RESET 1
  UARTSendDataBuf[0]='P';UARTSendDataBuf[1]='W';UARTSendDataBuf[2]='_';UARTSendDataBuf[3]='R';UARTSendDataBuf[4]='E';
  UARTSendDataBuf[5]='S';UARTSendDataBuf[6]='E';UARTSendDataBuf[7]='T';UARTSendDataBuf[8]=' ';UARTSendDataBuf[9]='1';
  //������������
  UART_Send_Data(UARTSendDataBuf, 10);
  
  //�ȴ�ģ������������0.5��ʱ�����ַ����IO��Ϊ0x0000�����봮�����õ�ַģʽ����ȡ�ղ����Ǵ��������õĵ�ַ�������ڸõ�ַ
  for(u8 i=0; i<6; i++)
  {
    Delay(0xFFFF);    //ÿ��DelayԼ200ms������ʱ1�����ϣ�����Ҫ�������ȴ�ʱ�䣬��Ϊģ���ϵ�һ��ʱ��󴮿ڲſ�ʼ������������
  }
    
  
  //���½���18B20������������
  DS18B20_Init();
  DS18B20_Write(0xCC);  //����ROM����
  DS18B20_Write(0xBE);  //������
  //����TH��TLֵ�Ա�����ʱ����ԭ��д�벻���иı�
  Temprature=DS18B20_Read();    //��8λ
  Temprature=Temprature | (DS18B20_Read()<<8);  //��8λ
  TH=DS18B20_Read();
  TL=DS18B20_Read();
  Config=DS18B20_Read();
  
  DS18B20_Init();
  DS18B20_Write(0xCC);  //����ROM����
  DS18B20_Write(0x4E);  //д����
  DS18B20_Write(TH);    //ԭ��д�벻�ı�
  DS18B20_Write(TL);    //ԭ��д�벻�ı�
  DS18B20_Write(0x1F);  //���ýϵͷֱ����ܴ�����͹��� 1F��9λ��93.75ms 3F��10λ��187.5ms 5F��11λ��375ms 7F��12λ��750ms
    
#ifdef UI_STRING    //�ַ����������
  //��ʽ ID:1234 T:+025.0
  //0~3��AA XX XX 55��Ŀ�ĵ�ַ
  //4~6, 3�ֽ�ͷ
  //7~10, 4�ֽڵ�ַ
  //11~13, 3�ֽ���
  //14������λ
  //15~17������λ
  //18��С����
  //19��С��λ
  //20��1�ֽ�β
  UARTSendDataBuf[0]=0xAA;UARTSendDataBuf[1]=0x00;UARTSendDataBuf[2]=0x00;UARTSendDataBuf[3]=0x55;
  UARTSendDataBuf[4]='I';UARTSendDataBuf[5]='D';UARTSendDataBuf[6]=':';
  //��������ַת��Ϊʮ�ֽ��ַ���������ֵ����UARTSendDataBuf[10]~UARTSendDataBuf[19]
  SettingTemp=Addr;
  for(u8 i=0; i<4; i++)
  {
    UARTSendDataBuf[10-i] = '0' + (SettingTemp % 10);
    SettingTemp /= 10;
  }
  //��ͷ��������0�滻Ϊ�ո�
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
    IWDG_ReloadCounter();   //ι��
#endif
    
    if(WakeCount<=0)    //ÿWORK_TO_WAKE_RATIO�λ��Ѳɼ�һ�δ��������ݲ��ϴ���֮���Բ��ó�ֵ-1����<=0����++��%WORK_TO_WAKE_RATIO�ķ�ʽ����Ϊ���ڿ���ʱ����������ʹ��������������
    {
      DS18B20_Init();
      DS18B20_Write(0xCC);  //����ROM����
      DS18B20_Write(0x44);  //�¶�ת��
      
      //9λ�ֱ���93.75ms����ʱ������£�����˯��128ms���ȴ�18B20�������
      //ע�⣡���������߷ֱ��ʻ��ӳ�����ʱ�䣬��ʱӦ�ӳ�˯��ʱ��ʹ֮���ڲ���ʱ�䣬��������
      enableInterrupts();
      AWU_Init(AWU_Timebase_128ms);
      AWU_ReInitCounter();
      AWU_Cmd(ENABLE);
      halt();
      disableInterrupts();
      
#ifdef  WATCHDOG
      IWDG_ReloadCounter();   //ι��
#endif
  
      while(GPIO_ReadInputDataBit(SENSOR_DATA_PORT, SENSOR_DATA_PIN)==RESET);   //��18B20��δ���������ȴ�
      
      DS18B20_Init();
      DS18B20_Write(0xCC);  //����ROM����
      DS18B20_Write(0xBE);  //��ȡRAM
      Temprature=DS18B20_Read();    //��8λ
      Temprature=Temprature | (DS18B20_Read()<<8);  //��8λ
      TH=DS18B20_Read();
      TL=DS18B20_Read();
      Config=DS18B20_Read();
      DS18B20_Init();       //reset����ֹ��д�����Ĵ���
      
#ifndef UI_STRING
      //���ɼ������¶����ݷ��͵�����
      UARTSendDataBuf[0]=0xAA;      //4�ֽ�Ŀ�ĵ�ַ��ͷ��0xAA Ŀ�ĵ�ַ��λ Ŀ�ĵ�ַ��λ 0x55������Ŀ�ĵ�ַ�ߵ�λ��Ϊ0x00
      UARTSendDataBuf[1]=0x00;
      UARTSendDataBuf[2]=0x00;
      UARTSendDataBuf[3]=0x55;
      UARTSendDataBuf[4]=AddrHi;    //���͸�����������ַ�Է���������Ѱַ����ַ��λ���������Ƿſ��˵�ַ��λ��ģ���3λ��ַ��λĬ����������Ϊ0
      UARTSendDataBuf[5]=AddrLo;    //���͸�����������ַ��λ
      UARTSendDataBuf[6]=(u8)(Temprature>>8);     //�¶ȸ�λ
      UARTSendDataBuf[7]=(u8)(Temprature&0x00FF); //�¶ȵ�λ
      UART_Send_Data(UARTSendDataBuf, 8);         //�����ݷ��͵�����
#else
      //�ַ�����ʽ
      UARTSendDataBuf[14]='+';
      if(Temprature & 0x8000)   //���λΪ1�����ģ�ȡ����һ
      {
        UARTSendDataBuf[14]='-';
        Temprature = (~Temprature)+1;
      }
      //С������
      UARTSendDataBuf[19]='0';
      if(Temprature & 0x000F)
      {
        UARTSendDataBuf[19]='5';
      }
      //��������
      SettingTemp=Temprature>>4;
      for(u8 i=0; i<3; i++)
      {
        UARTSendDataBuf[17-i] = '0' + (SettingTemp % 10);
        SettingTemp /= 10;
      }
      UART_Send_Data(UARTSendDataBuf, 21);         //�����ݷ��͵�����
#endif
    }

#ifdef  WATCHDOG
    IWDG_ReloadCounter();   //ι��
#endif
    
    WakeCount++;
    WakeCount = WakeCount % WORK_TO_WAKE_RATIO;   //ÿWORK_TO_WAKE_RATIO�λ��Ѳɼ�һ�δ��������ݲ��ϴ���ԼWORK_TO_WAKE_RATIO*30��ɼ��ϴ�һ�Σ�

    enableInterrupts();
    AWU_Init(AWU_Timebase_30s);   //30��˯�ߣ�ע�⣡��ʵ��˯��ʱ�����ϴ�
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
void Delay(uint16_t nCount)   //ʵ��ÿ��count��Ӧ6ָ�����ڣ�2MHz�¶�Ӧ3us
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
  SENSOR_DATA_PORT->ODR &= (uint8_t)(~(SENSOR_DATA_PIN));   //�������ߣ�����͵�ƽ
  SENSOR_DATA_PORT->DDR |= SENSOR_DATA_PIN;
  DELAY_500US();DELAY_100US();    //����600us
  SENSOR_DATA_PORT->DDR &= (uint8_t)(~(SENSOR_DATA_PIN));   //�ͷ����ߣ�����Ϊ����
  DELAY_50US();       //�����ȴ�
  while(GPIO_ReadInputDataBit(SENSOR_DATA_PORT, SENSOR_DATA_PIN)==SET);     //�ȴ�18B20��������
  while(GPIO_ReadInputDataBit(SENSOR_DATA_PORT, SENSOR_DATA_PIN)==RESET);   //�ȴ�18B20�ͷ����� 
  DELAY_50US();    //�ȴ�RX�����������
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
    
    SENSOR_DATA_PORT->ODR &= (uint8_t)(~(SENSOR_DATA_PIN));   //�������ߣ�����͵�ƽ
    SENSOR_DATA_PORT->DDR |= SENSOR_DATA_PIN;
    DELAY_1US();DELAY_1US();                 //��ʱ2us
    SENSOR_DATA_PORT->DDR &= (uint8_t)(~(SENSOR_DATA_PIN));   //�ͷ����ߣ�����Ϊ����
    DELAY_10US(); DELAY_1US();DELAY_1US();   //�ȴ�12us
    
    if(GPIO_ReadInputDataBit(SENSOR_DATA_PORT, SENSOR_DATA_PIN)==SET)   //��ȡ��ƽ
    {
      Data |= 0x80;
    }
    
    DELAY_10US();DELAY_10US();DELAY_10US();DELAY_10US();DELAY_5US();    //�ȴ�45us
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
    SENSOR_DATA_PORT->ODR &= (uint8_t)(~(SENSOR_DATA_PIN));   //�������ߣ�����͵�ƽ
    SENSOR_DATA_PORT->DDR |= SENSOR_DATA_PIN;
    DELAY_10US();DELAY_5US();     //�ȴ�15us
    
    if(Data & 0x01)     //���д��1
    {
      SENSOR_DATA_PORT->DDR &= (uint8_t)(~(SENSOR_DATA_PIN));   //�ͷ����ߣ�����Ϊ����
    }
    DELAY_10US();DELAY_10US();DELAY_10US();DELAY_10US();DELAY_5US();     //�ȴ�45us
    
    SENSOR_DATA_PORT->DDR &= (uint8_t)(~(SENSOR_DATA_PIN));   //�ͷ����ߣ�����Ϊ����
    DELAY_5US();    //�ָ�ʱ��
    
    Data >>= 1;
  }
}

/**
  * @brief  ���͸�ģ�����ݻ�����
  * @param  DataBuf - ���͸�ģ������ݣ�DataLength - ���ͳ���
  * @retval None
  */
void UART_Send_Data(u8 DataBuf[], u8 DataLength)
{
  u8 idx=0;
  
  USART_Cmd(ENABLE);    //ʹ�ܴ���
  
  //ѡ���������ֻ��ѷ�ʽ֮һ
  
  //��һ�֣�IO�ڻ��ѣ������ѿ����ߺ�ȴ�3ms����ʼ���ʹ����ź�
  //�ڷ�����һ�����ݰ���ȴ�20ms���Ϸ�����һ�����ݰ��������д����źŷ�����Ϻ����ͻ��ѿ�
  GPIO_WriteBit(MODE0_PORT, MODE0_PIN, SET);    //���Ѵ���
  Delay(0x400);         //�ȴ�3ms
  
  //�ڶ��֣�����ֱ�ӻ��ѣ�����һ�ֽ�0x55���Ѵ��ں�ȴ�10ms���ʹ������ݣ��ڷ�����һ�����ݰ���ȴ�20~150ms������һ�����ݰ�
  //�������150msδ�������ݰ�������Ҫ���½��л���
  //USART_SendData8(0x55);    //���ͻ����ֽ�
  //while((USART->SR & 0x80) == 0);      //�ȴ����ͻ����
  //Delay(0xD00);         //�ȴ�10ms
  
  for(idx=0; idx<DataLength; idx++)
  {
    USART_SendData8(DataBuf[idx]);    //���͵�ǰ�ַ�
    while((USART->SR & 0x80) == 0);   //�ȴ����ͻ����
  }
  
  while((USART->SR & 0x40) == 0);       //�ȴ��������
  GPIO_WriteBit(MODE0_PORT, MODE0_PIN, RESET);    //��ģ�����½���˯��
  
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

