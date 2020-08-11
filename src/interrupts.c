#include "system.h"

volatile uint8_t send;
volatile uint8_t curMode;

void USART1_IRQHandler(void){
    static uint8_t stat = 0;
    static uint8_t crc = 0;
    static uint8_t m = 0;

    if(USART1->ISR & USART_ISR_RXNE){               //if read data register not empty
        uint8_t d = USART1->RDR;                    //read dataRegister
        IWDG->KR = IWDG_REFRESH;                    //udate counter watch dog
        switch (stat) {
            case 0:
                if (d == 0x02) send = 1;
                if (d == 0x43) crc = 0xBC, stat = 1;
                break;
            case 1:
                m = d;
                crc ^= d;
                stat = 2;
                break;
            case 2:
                if (crc == d) curMode = m;
                stat = 0;
                break;
        }
    }

    if(USART1->ISR & USART_ISR_FE){                 //Framing error
        USART1->ICR = USART_ICR_FECF;               //clears the FE flag
    }
    if(USART1->ISR & USART_ISR_ORE){                //Overrun error    
        USART1->ICR = USART_ICR_ORECF;              //clears the ORE flag
    }
    if(USART1->ISR & USART_ISR_NE){                 //START bit Noise detection flag
        USART1->ICR = USART_ICR_NCF;                //clears the NF flag
    }
}