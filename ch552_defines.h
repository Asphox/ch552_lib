#ifndef CH552_DEFINES_H
#define CH552_DEFINES_H

#include <stdint.h>
#include <string.h>
#include "ch554.h"

// UTILS

#define PUT_BIT(byte,pos,value) (byte ^= (-(!!value)^(uint8_t)(byte)) & pos) 


//  PORTS DEFINES
#define PORT1_BASE_ADDR 0x90
#define PORT2_BASE_ADDR 0xA0
#define PORT3_BASE_ADDR 0xB0

//  SAFE MODE
#define SAFE_MOD_ON() SAFE_MOD = 0x55; SAFE_MOD = 0xAA
#define SAFE_MOD_OFF() SAFE_MOD = 0x00

//  FREQ

#define SYS_FREQ 24000000

void SYS_init()
{
  SAFE_MOD_ON();
  #if SYS_FREQ == 32000000
  	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x07;  // 32MHz
  #elif SYS_FREQ == 24000000
  	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x06;  // 24MHz
  #elif SYS_FREQ == 16000000
  	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x05;  // 16MHz
  #elif SYS_FREQ == 12000000
  	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x04;  // 12MHz
  #elif SYS_FREQ == 6000000
  	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x03;  // 6MHz
  #elif SYS_FREQ == 3000000
  	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x02;  // 3MHz
  #elif SYS_FREQ == 750000
  	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x01;  // 750KHz
  #elif SYS_FREQ == 187500
  	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x00;  // 187.5MHz
  #else
  	#warning SYS_FREQ invalids or not sets
  #endif
  SAFE_MOD_OFF();
}

//  TIMERS

#define TMR_MOD_COUNTER (1<<2)
#define TMR_MOD_13 0
#define TMR_MOD_16 1
#define TMR_MOD_8_RELOAD 2
#define TMR0_MOD_SPLIT 3

#define TMR0_RUN(b) (TR0 = b)
#define TMR0_ENABLE_IRQ(b) (ET0 = b)
#define TMR0_IRQ_FLAG_OVF() (TF0)
#define TMR0_MOD(mode) (TMOD |= mode)
#define TMR0_LOAD(value) (TL0 = (uint8_t)((uint16_t)(value & 0xFF))); (TH0 = (uint8_t)((uint16_t)(value>>8) & 0xFF))
#define TMR0_LOAD_L(value) TL0 = ((uint8_t)value)
#define TMR0_LOAD_H(value) TH0 = ((uint8_t)value)
#define TMR0_IRQ() __onTimer0() __interrupt(INT_NO_TMR0)

#define TMR1_RUN(b) (TR1 = b)
#define TMR1_ENABLE_IRQ(b) (ET1 = b)
#define TMR1_GET_FLAG_OVF() (TF1)
#define TMR1_MOD(mode) (TMOD |= (mode<<4))
#define TMR1_LOAD(value) (TL1 = (uint8_t)((uint16_t)(value & 0xFF))); (TH1 = (uint8_t)((uint16_t)(value>>8) & 0xFF))
#define TMR1_LOAD_L(value) TL1 = ((uint8_t)value)
#define TMR1_LOAD_H(value) TH1 = ((uint8_t)value)
#define TMR1_IRQ() __onTimer1() __interrupt(INT_NO_TMR1)


// PWMs
#define PWM_SET_DIV(div) (PWM_CK_SE = (uint8_t)div)
void PWM_ENABLE_IRQ(uint8_t b)
{
	if(b)
	{
		PWM_CTRL |= bPWM_IE_END;
	}
	IE_PWMX = b;
}
#define PWM_GET_FLAG_END() ((uint8_t)PWM_CTRL & bPWM_IF_END)
#define PWM_SET_FLAG_END(b) (PUT_BIT(PWM_CTRL,bPWM_IF_END,b))
#define PWM_IRQ() __onPWM() __interrupt(INT_NO_PWMX)

#define PWM1_RUN(b) (PUT_BIT(PWM_CTRL,bPWM1_OUT_EN,b)) 
#define PWM1_SET_POLAR(b) (PUT_BIT(PWM_CTRL,bPWM1_POLAR,b))
#define PWM1_SET_DC(dc) (PWM_DATA1 = (uint8_t)dc)

#define PWM2_RUN(b) (PUT_BIT(PWM_CTRL,bPWM2_OUT_EN,b))
#define PWM2_SET_POLAR(b) (PUT_BIT(PWM_CTRL,bPWM2_POLAR,b))
#define PWM2_SET_DC(dc) (PWM_DATA2 = (uint8_t)dc)

// UARTS

#define UART1_ENABLE_RX(b) (U1REN = b)
#define UART1_ENABLE_9b(b) (U1SM0 = b)
#define UART1_WRITE_9b(b) (U1TB8 = b)
#define UART1_READ_9b() (U1RB8)
#define UART1_ENABLE_IRQ(b) (IE_UART1 = b)


#define USB_IRQ() __onUSB() __interrupt(INT_NO_USB)

#endif //CH552_DEFINES_H
