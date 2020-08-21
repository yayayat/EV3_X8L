#pragma once

#include "stm32f0xx.h"
#include "xprintf.h" 

#define IWDG_START          0xCCCC
#define IWDG_WRITE_ACCESS   0x5555
#define IWDG_REFRESH        0xAAAA

#define BAUD 2400        //UART default baud rate

// extern volatile uint16_t dataSend;
extern volatile uint8_t send;
extern volatile uint8_t currentMode;

void sysInit(void);
void iwdgInit(void);
void uartWrite(uint8_t d);

