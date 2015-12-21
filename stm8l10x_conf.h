

#ifndef __STM8L10x_CONF_H
#define __STM8L10x_CONF_H

/* Includes ------------------------------------------------------------------*/
#include "stm8l10x.h"



/* Uncomment the line below to expanse the "assert_param" macro in the 
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */ 

/* In the following line adjust the value of External High Speed oscillator (HSE)
   used in your application */
 #define HSE_VALUE ((uint32_t)16000000) /* Value of the External oscillator in Hz*/

/*** AUTO WAKE UP *************************************************************/
#define _AWU (1)

/*** BEEP *********************************************************************/
#define _BEEP (1)

/*** CLOCK CONTROLLER *********************************************************/
#define _CLK (1)

/*** COMPARATOR ***************************************************************/
#define _COMP (1)

/*** EXTERNAL INTERRUPTS ******************************************************/
#define _EXTI (1)

/*** FLASH/DATA EEPROM ********************************************************/
#define _FLASH (1)

/*** GENERAL PURPOSE IOs ******************************************************/
#define _GPIO (1)


/*** INTERRUPT CONTROLLER *****************************************************/
#define _ITC (1)

/*** INDEPENDANT WDG *********************************************************/
#define _IWDG (1)

/*** I2C **********************************************************************/
#define _I2C (1)

/*** OPTION BYTES *************************************************************/
#define _OPT (1)

/*** RESET CONTROLLER *********************************************************/
#define _RST (1)

/*** SPI ***********************************************************************/
#define _SPI (1)

/*** TIMERs *******************************************************************/
#define _TIM2 (1)
#define _TIM3 (1)
#define _TIM4 (1)
/*** IR remote control interface**************************************************/
#define _IRTIM (1)

/*** USART ********************************************************************/
#define _USART (1)

/*** WFE ********************************************************************/
#define _WFE (1)

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT



  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

/* Peripheral header file inclusion ******************************************/

#ifdef _AWU
#include "stm8l10x_awu.h"
#endif /* _AWU */

#ifdef _BEEP
#include "stm8l10x_beep.h"
#endif /* _BEEP */


#if defined(_FLASH) || defined(_OPT)
#include "stm8l10x_flash.h"
#endif /* _FLASH/OPT */

#ifdef _EXTI
#include "stm8l10x_exti.h"
#endif /* _EXTI */

#ifdef _GPIO
#include "stm8l10x_gpio.h"
#endif /* _GPIOx */

#ifdef _I2C
#include "stm8l10x_i2c.h"
#endif /* _I2C */

#ifdef _ITC
#include "stm8l10x_itc.h"
#endif /* _ITC */

#ifdef _IWDG
#include "stm8l10x_iwdg.h"
#endif /* _IWDG */

#ifdef _CLK
#include "stm8l10x_clk.h"
#endif /* _CLK */

#ifdef _RST
#include "stm8l10x_rst.h"
#endif /* _RST */

#ifdef _SPI
#include "stm8l10x_spi.h"
#endif /* _SPI */

#ifdef _TIM2
#include "stm8l10x_tim2.h"
#endif /* _TIM2 */

#ifdef _TIM3
#include "stm8l10x_tim3.h"
#endif /*_TIM3 */

#ifdef _TIM4
#include "stm8l10x_tim4.h"
#endif /* _TIM4 */

#ifdef _IRTIM
#include "stm8l10x_irtim.h"
#endif /* _IRTIM */

#ifdef _USART
#include "stm8l10x_usart.h"
#endif /* _USART */

#ifdef _COMP
#include "stm8l10x_comp.h"
#endif /* _COMP */

#ifdef _WFE
#include "stm8l10x_wfe.h"
#endif /* _WFE */

#endif /* __STM8L10x_CONF_H */

