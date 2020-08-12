#include "system.h"

uint16_t data;
volatile uint16_t dataSend;

void rccInit(){
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                      //enable GPIOA, GPIOB, 
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN |
                     RCC_APB2ENR_ADCEN;                     //enable USATR and ADC
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;                     //enablle TIM3
}

void gpioInit(){
    GPIOA->MODER |= GPIO_MODER_MODER10_1 |
                    GPIO_MODER_MODER9_1;                   //A10 and A9 alternate function mode	                
    GPIOA->AFR[1] |= 0x00000110;                            //10 and A9 alternate function mode	
}

void uartWrite(uint8_t d){
    while(!(USART1->ISR & USART_ISR_TXE));                  //waiting for the data to be transferred
    USART1->TDR = d;                                        //write data to dataRegister
}

void uartInit(){
    USART1->BRR = (F_CPU+BAUD/2)/BAUD;                      //2400 boad
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE;              //enable TX and RX
    USART1->CR1 |= USART_CR1_UE;                            //USART enable
    NVIC_SetPriority(USART1_IRQn, 1);                       //set priority uart interrupt 
    NVIC_EnableIRQ(USART1_IRQn);                            //enable interrupt
}

void adcInit(){
    if ((ADC1->CR & ADC_CR_ADEN) != 0) 
        ADC1->CR |= ADC_CR_ADDIS;                            //If ADC enable then disable 
    while ((ADC1->CR & ADC_CR_ADEN) != 0);                   //Wait until ADC disable
    ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;                         //Disable DMA
    ADC1->CR = ADC_CR_ADCAL;                                 //Calibration ADC.  //Calibration can only be
    while(ADC1->CR & ADC_CR_ADCAL);                          //Wait calibration  //initiated when the ADC is disabled (when ADEN=0)
    ADC1->ISR |= ADC_ISR_ADRDY;                              //Clear the ADRDY bit in ADC_ISR register by programming this bit to 1
    
    ADC1->CR = ADC_CR_ADEN;                                  //ADC enable
    while(!(ADC1->ISR & ADC_ISR_ADRDY));                     //Wait then ADC ready 
    ADC1->SMPR = 0;                                          //Programmable sampling time
    ADC1->IER |= ADC_IER_EOCIE;                              //End of conversion interrupt enable
    ADC1->CFGR1 = ADC_CFGR1_EXTEN_0 | ADC_CFGR1_EXTSEL_1 |   //Trigger detection on the rising edge | external event used to trigger the start of conversion (TRG3)(TIM3_TRGO)
                  ADC_CFGR1_EXTSEL_0;                        //Channel selection 
    ADC1->CHSELR = 0b100000;                                 
    
    ADC1->CR |= ADC_CR_ADSTART;                              //Starting conversions

    NVIC_EnableIRQ(ADC1_IRQn);                               //Enable NVIC
}

void ADC1_IRQHandler(void){
    static uint16_t mina, maxa=500;

    if(ADC1->ISR & ADC_ISR_EOC){                             //If transfer complete
        ADC1->ISR |= ADC_ISR_EOC;                            //Transfer complete clear   
        uint16_t a=ADC1->DR;                                 //read data register
        maxa--;
        mina++;
        if(a>maxa)maxa=a;
        if(a<mina)mina=a;
        dataSend = maxa - mina;
    }
}

void tim3Init(){
    TIM3->CR1 = TIM_CR1_ARPE;                                //Auto-reload preload enable
    TIM3->CR2 = TIM_CR2_MMS_1;                               //The update event is selected as trigger output
    TIM3->PSC = 50;                                          //Prescaler 
    TIM3->ARR = 181-1;                                       //Auto-reload register
    TIM3->CR1 |= TIM_CR1_CEN;                                //Counter enable
}

void iwdgInit(void){
    IWDG->KR = IWDG_START;                                   //start watch dog
    IWDG->KR = IWDG_WRITE_ACCESS;                            //enable access to the IWDG_PR, IWDG_RLR and IWDG_WINR registers 
    IWDG->PR = IWDG_PR_PR_0 | IWDG_PR_PR_2;                  //set prescaler
    IWDG->RLR = 625 - 1;                                     //reload value / 2 sec
    IWDG->KR = IWDG_REFRESH;                                 //udate counter watch dog
}

void sysInit(){
    rccInit();
    gpioInit();
    uartInit();
    adcInit();
    tim3Init();
}

