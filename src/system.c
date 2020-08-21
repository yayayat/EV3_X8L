#include "system.h"

// volatile int dataSend;
volatile int16_t data[8];
int16_t dataF[8];                                 //raw data, filtered data


void rccInit(){
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN |
                   RCC_AHBENR_GPIOBEN |
                   RCC_AHBENR_DMA1EN;                       //enable GPIOA, DMA1
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN |
                     RCC_APB2ENR_ADCEN;                     //enable USATR and ADC
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;                     //enablle TIM3
}

void gpioInit(){
    GPIOA->MODER |= GPIO_MODER_MODER10_1 |
                    GPIO_MODER_MODER9_1 | 0xFFFF;           //A10 and A9 alternate function mode
    GPIOA->AFR[1] |= 0x00000110;                            //A10 and A9 alternate function mode
    GPIOB->MODER |= GPIO_MODER_MODER1_0;
}

void uartWrite(uint8_t d){
    while(!(USART1->ISR & USART_ISR_TXE));                  //waiting for the data to be transferred
    USART1->TDR = d;                                        //write data to dataRegister
}

void uartInit(){
    USART1->BRR = (F_CPU+BAUD/2)/BAUD;                      //setting boad rate
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE;              //enable TX and RX
    USART1->CR1 |= USART_CR1_UE;                            //USART enable
    NVIC_SetPriority(USART1_IRQn, 1);                       //set priority uart interrupt
    NVIC_EnableIRQ(USART1_IRQn);                            //enable interrupt
}

    void adcInit(){
    if ((ADC1->CR & ADC_CR_ADEN) != 0)
        ADC1->CR |= ADC_CR_ADDIS;                           //If ADC enable then disable
    while ((ADC1->CR & ADC_CR_ADEN) != 0);                  //Wait until ADC disable
    ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;                        //Disable DMA
    ADC1->CR = ADC_CR_ADCAL;                                //Calibration ADC.  //Calibration can only be
    while(ADC1->CR & ADC_CR_ADCAL);                         //Wait calibration  //initiated when the ADC is disabled (when ADEN=0)
    ADC1->ISR |= ADC_ISR_ADRDY;                             //Clear the ADRDY bit in ADC_ISR register by programming this bit to 1

    ADC1->CR = ADC_CR_ADEN;                                 //ADC enable
    while(!(ADC1->ISR & ADC_ISR_ADRDY));                    //Wait then ADC ready
    ADC1->SMPR = 1;                                         //Programmable sampling time
    //ADC1->IER |= ADC_IER_EOCIE | ADC_IER_EOSEQIE;         //End of conversion interrupt enable
    ADC1->CFGR1 = ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG |
                  ADC_CFGR1_DISCEN | ADC_CFGR1_EXTEN_0 |    //Trigger detection on the rising edge | external event used to trigger the start of conversion (TRG3)(TIM3_TRGO)
                  ADC_CFGR1_EXTSEL_0 | ADC_CFGR1_EXTSEL_1;  //Channel selection
    ADC1->CHSELR = 0xFF;

    DMA1_Channel1->CPAR = ((uint32_t) &(ADC1->DR));
    DMA1_Channel1->CMAR = (uint32_t) data;
    DMA1_Channel1->CNDTR = 8;
    DMA1_Channel1->CCR |= DMA_CCR_MINC | DMA_CCR_MSIZE_0
                       | DMA_CCR_PSIZE_0
                       | DMA_CCR_TCIE | DMA_CCR_CIRC
                       | DMA_CCR_PL;
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    ADC1->CR |= ADC_CR_ADSTART;                             //Starting conversions
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void DMA1_Channel1_IRQHandler(void){
    static uint32_t accR[8],accA[8];                        //acc Reflected/Ambient
    static uint16_t dataRF[8],dataAF[8];                    //data Reflected/Ambient filtered
    static uint8_t backlightState=0;                        //TCRT5000 sensor LED state
    if(backlightState){
        for(uint8_t i = 0;i<8;i++){
            accR[i]+=data[i]-dataF[i];
            dataRF[i]=accR[i]/2048;
            dataF[i]=dataRF[i];
        }
    }
    else{
        for(uint8_t i = 0;i<8;i++){
            accA[i]+=data[i]-dataAF[i];
            dataAF[i]=accA[i]/2048;
            dataF[i]=dataRF[i]-dataAF[i];
        }
    }
    // backlightState^=1;
    // GPIOB->BSRR=backlightState?GPIO_BSRR_BS_1:GPIO_BSRR_BR_1;
    DMA1->IFCR = DMA_IFCR_CGIF1;
}

void tim3Init(){
    TIM3->CR1 = TIM_CR1_ARPE;                               //Auto-reload preload enable
    TIM3->CR2 = TIM_CR2_MMS_1;                              //The update event is selected as trigger output
    TIM3->PSC = 1-1;                                        //Prescaler
    TIM3->ARR = 640-1;                                      //Auto-reload register
    TIM3->CR1 |= TIM_CR1_CEN;                               //Counter enable
}

void iwdgInit(void){
    IWDG->KR = IWDG_START;                                  //start watch dog
    IWDG->KR = IWDG_WRITE_ACCESS;                           //enable access to the IWDG_PR, IWDG_RLR and IWDG_WINR registers
    IWDG->PR = IWDG_PR_PR_0 | IWDG_PR_PR_2;                 //set prescaler
    IWDG->RLR = 625 - 1;                                    //reload value / 2 sec
    IWDG->KR = IWDG_REFRESH;                                //udate counter watch dog
}

void sysInit(){
    xdev_out(uartWrite);
    rccInit();
    gpioInit();
    uartInit();
    adcInit();
    tim3Init();
}