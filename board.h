
#ifndef __BOARD_H
#define __BOARD_H


/* MACROs for SET, RESET or TOGGLE Output port */
/*
#define GPIO_PIN_0        0x01
#define GPIO_PIN_1        0x02
#define GPIO_PIN_2        0x04
#define GPIO_PIN_3        0x08
#define GPIO_PIN_4        0x10
#define GPIO_PIN_5        0x20
#define GPIO_PIN_6        0x40
#define GPIO_PIN_7        0x80
#define GPIO_PIN_LNIB     0x0F
#define GPIO_PIN_HNIB     0xF0
#define GPIO_PIN_ALL      0xFF
*/
  
#define ADDR_LOW_PORT		GPIOB
#define ADDR_LOW_PIN0           GPIO_PIN_0
#define ADDR_LOW_PIN1           GPIO_PIN_1
#define ADDR_LOW_PIN2           GPIO_PIN_2
#define ADDR_LOW_PIN3           GPIO_PIN_3
#define ADDR_LOW_PIN4           GPIO_PIN_4
#define ADDR_LOW_PIN5           GPIO_PIN_5
#define ADDR_LOW_PIN6           GPIO_PIN_6
#define ADDR_LOW_PIN7           GPIO_PIN_7

#define ADDR_HIGH_PORT0		GPIOC
#define ADDR_HIGH_PIN0          GPIO_Pin_0

#define ADDR_HIGH_PORT1		GPIOC
#define ADDR_HIGH_PIN1          GPIO_Pin_1

#define ADDR_HIGH_PORT2		GPIOD
#define ADDR_HIGH_PIN2          GPIO_Pin_0

#define MODE0_PORT              GPIOA
#define MODE0_PIN               GPIO_Pin_2

#define MODE1_PORT              GPIOA
#define MODE1_PIN               GPIO_Pin_3

#define SENSOR_DATA_PORT        GPIOC
#define SENSOR_DATA_PIN         GPIO_Pin_4


#define DELAY_1US()   nop();nop();
#define DELAY_5US()   DELAY_1US();DELAY_1US();DELAY_1US();DELAY_1US();DELAY_1US();
#define DELAY_10US()  DELAY_5US();DELAY_5US();
#define DELAY_50US()  DELAY_10US();DELAY_10US();DELAY_10US();DELAY_10US();DELAY_10US();
#define DELAY_100US() DELAY_50US();DELAY_50US();
#define DELAY_500US() DELAY_100US();DELAY_100US();DELAY_100US();DELAY_100US();DELAY_100US();


#endif
