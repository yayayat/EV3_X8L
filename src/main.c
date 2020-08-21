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

extern volatile uint16_t data[8];
// uint8_t* pp=data;

uint16_t testMessage[8]={
    0x0000,0x1000,0x2000,0x3000,0x4000,0x5000,0x6000,0x7000
};

void sendData(uint8_t mode, uint8_t length, uint8_t* data) {
    // uartWrite(0x00);                                            //SYNC
    uint8_t header = 0xC0 | (length << 3) | mode;               //header config 0b11LLLMMM    
                                                                //0b[DATA MODE][LENGTH][MODE]
    uint8_t crc = 0xFF ^ header;                                //calculating checksum
    uartWrite(header);                                          //header transmission
    for (uint8_t i = 0; i < (1 << length); i++) {
        uartWrite(data[i]);                                     //payload transmission
        crc ^= data[i];                                         //calculating checksum
    }
    uartWrite(crc);                                             //checksum transmission
}

int main(void){
    sysInit();
    for(uint16_t i=0; i<sizeof(protocol); i++){     //CMD & INFO transmission
        uartWrite(protocol[i]);
    }
    iwdgInit();                                     //initialize watch dog

    while(1){
        if(USART1->ISR & USART_ISR_RXNE){           //if rx data register not empty
            if (USART1->RDR == 0x04){
                break;
            }
        }
    }
    USART1->BRR = (F_CPU+57600/2)/57600;            //57600 boad
    USART1->CR1 |= USART_CR1_RXNEIE;                //interrupt enable
    USART1->CR3 = USART_CR3_EIE;                    //Error interrupt enable
    GPIOB->BSRR=GPIO_BSRR_BS_1;
    while (1)
    {
        uint8_t pp[16];
        while (!send);
        for(uint8_t i=0;i<4;i++)
            ((uint32_t*)pp)[i] = ((uint32_t*)data)[i];
        send=0;
        sendData(0,4, pp);
        // for(uint8_t i=0;i<8;i++)
        //     testMessage[i]+=0x10;
        // xprintf("%4d ",data[i]);
        // xprintf("\n");
        // for(volatile uint32_t i=0;i<50000;i++);
    }
}