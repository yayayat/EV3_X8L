/***************************************************
The code is written vladi-v, zerg17, yayayat.
                -06.08.2020-
Code for the 8x line sensor from the constructor ev3.
                    ---
                    
                USART TX PA9
                USART RX PA10
                    --- 
***************************************************/

#include "system.h"
#include "protocol.h"

extern volatile uint16_t dataF[8];

void sendData(uint8_t mode, uint8_t len, uint8_t* data) {
    uint8_t st = 0xC0 | (len << 3) | mode;
    uartWrite(st);
    uint8_t crc = 0xFF ^ st;
    for (uint8_t i = 0; i < (1 << len); i++) {
        uartWrite(data[i]);
        crc ^= data[i];
    }
    uartWrite(crc);
}

int main(void){
    sysInit();
    for(uint16_t i=0; i<sizeof(protocol); i++){     //transmit protocol 
        uartWrite(protocol[i]);
    }
    iwdgInit();                                     //initialization watch dog

    while(1){
        if(USART1->ISR & USART_ISR_RXNE){           //if read data register not empty
            if (USART1->RDR == 0x04){
                break;
            }
        }
    }
    USART1->BRR = (F_CPU+57600/2)/57600;         //57600 boad
    USART1->CR1 |= USART_CR1_RXNEIE;                //interrupt enable
    USART1->CR3 = USART_CR3_EIE;                    //Error interrupt enable

    while (1)
    {
        while (!send);
        send=0;
        sendData(0,2,(uint8_t*) &dataF);
    }
}