
 // PROJECT C/ASM Mix Example
// AYUSHI THAKKAR

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include<stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include<strings.h>
#include "tm4c123gh6pm.h"
#include<ctype.h>
char str[81];
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define MEAS_LR      (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 2*4)))
#define HIGHSIDE_R   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 0*4)))
#define MEAS_C       (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define LOWSIDE_R    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))
#define INTEGRATOR   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 6*4)))
char str[81];
int maxchar = 80;
char field_offset[10];
char field_type[10];
int i,j;
int raw;
float instantvolt, iirvolt;
int field_count=0;
bool timeMode = false;
uint32_t frequency = 0;
uint32_t time = 0;
bool freqUpdate = false;
bool cap = false;
bool timeUpdate = false;
char number[12]={46,48,49,50,51,52,53,54,55,56,57,64};
char alpha[]={65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,95,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122};
char delimeter[]={32,33,34,35,36,37,38,39,40,41,42,43,44,45,47,58,59,60,61,62,63,91,92,93,94,96,123,124,125,126};
 extern void ResetISR(void);// an interrupt to generate software reset

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO ports A,B,D,E AND F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB| SYSCTL_RCGC2_GPIOD| SYSCTL_RCGC2_GPIOE| SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOC;

    // Configure LEDS and MEAS_C
    GPIO_PORTF_DIR_R |= 0x1E;  // make port 1,2,3 AND 4 as an outputs
    GPIO_PORTF_DR2R_R |= 0x1E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= 0x1E;  // enable LEDS AND MEAS_C
    // Configure MEAS_LR pin
        GPIO_PORTB_DIR_R |= 0x04;  // make port 2 as an output
        GPIO_PORTB_DR2R_R |= 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)
        GPIO_PORTB_DEN_R |= 0x04;  // enable  MEAS_LR
    // Configure HIGHSIDE_R pin
                GPIO_PORTE_DIR_R |= 0x01;  // make port 0 as an output
                GPIO_PORTE_DR2R_R |= 0x01; // set drive strength to 2mA (not needed since default configuration -- for clarity)
                GPIO_PORTE_DEN_R |= 0x01;  // enable  MEAS_LR
    // Configure INTEGRATOR pin
                GPIO_PORTD_DIR_R |= 0x40;  // make port 6  as an output
                GPIO_PORTD_DR2R_R |= 0x40; // set drive strength to 2mA (not needed since default configuration -- for clarity)
                GPIO_PORTD_DEN_R |= 0x40;  // enable  LOWSIDE_R and INTEGRATOR pins

     //configure LOWSIDE_R pin
                GPIO_PORTA_DIR_R |= 0x10;  // make port 4 as an output
                GPIO_PORTA_DR2R_R |= 0x10; // set drive strength to 2mA (not needed since default configuration -- for clarity)
                GPIO_PORTA_DEN_R |= 0x10;  // enable  LOWSIDE_R and INTEGRATOR pins

    // Configure UART0 pins
        SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
        GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
        GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
        GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

        // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
        UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
        UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
        UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
        UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
        UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
        UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

        // Configure SSI2 pins for SPI configuration
            SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;           // turn-on SSI2 clocking
            GPIO_PORTB_DIR_R |= 0x90;                        // make bits 4 and 7 outputs
            GPIO_PORTB_DR2R_R |= 0x90;                       // set drive strength to 2mA
            GPIO_PORTB_AFSEL_R |= 0x90;                      // select alternative functions for MOSI, SCLK pins
            GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK; // map alt fns to SSI2
            GPIO_PORTB_DEN_R |= 0x90;                        // enable digital operation on TX, CLK pins

            // Configure the SSI2 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
            SSI2_CR1_R &= ~SSI_CR1_SSE;                      // turn off SSI2 to allow re-configuration
            SSI2_CR1_R = 0;                                  // select master mode
            SSI2_CC_R = 0;                                   // select system clock as the clock source
            SSI2_CPSR_R = 40;                                // set bit rate to 1 MHz (if SR=0 in CR0)
            SSI2_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_8; // set SR=0, mode 3 (SPH=1, SPO=1), 8-bit
            SSI2_CR1_R |= SSI_CR1_SSE;

            // Configure AN0 as an analog input
            SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
            GPIO_PORTE_AFSEL_R |= 0x08;                      // select alternative functions for AN0 (PE3)
            GPIO_PORTE_DEN_R &= ~0x08;                       // turn off digital operation on pin PE3
            GPIO_PORTE_AMSEL_R |= 0x08;                      // turn on analog operation on pin PE3
            ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
            ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
            ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
            ADC0_SSMUX3_R = 0;                               // set first sample to AN0
            ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
            ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

            // configure analog pin

             SYSCTL_RCGCACMP_R = SYSCTL_RCGCACMP_R0;         //enable analog comparator clock
             GPIO_PORTC_DEN_R&=~0XC0;                        //enable comparator pins pc6 and pc7
             GPIO_PORTC_AMSEL_R |=0XC0;
              //unlocking pin PF0 to use comparator output pin
              GPIO_PORTF_LOCK_R=GPIO_LOCK_KEY;
              GPIO_PORTF_CR_R=GPIO_LOCK_LOCKED;
              GPIO_PORTF_AFSEL_R=0X01;
              //Configuring the PMCn fields in the GPIOPCTL register to assign the analog comparator output signals to the appropriate pins
              GPIO_PORTF_PCTL_R&= ~ GPIO_PCTL_PF0_M;
              GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF0_C0O;
              GPIO_PORTF_DIR_R |= 0x01;
              GPIO_PORTF_DEN_R |= 0X01;
              COMP_ACREFCTL_R=0X0000020E;                          //Configure the internal voltage reference to 2.4 V
              COMP_ACCTL0_R=0X402;                                 //The output of the comparator is inverted prior to being processed by hardware.
              waitMicrosecond(10);                                 //10 microseond delay
              // Configure FREQ_IN for frequency counter
                  GPIO_PORTC_AFSEL_R |= 0x10;                      // select alternative functions for FREQ_IN pin
                  GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC4_M;           // map alt fns to FREQ_IN
                  GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_WT0CCP0;
                  GPIO_PORTC_DEN_R |= 0x10;                        // enable bit  for digital input
              // Configure Wide Timer 0 as counter
               setTimerMode();
             }

int readAdc0Ss3()
        {
            ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
            while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
            return ADC0_SSFIFO3_R;                           // get single result from the FIFO
        }

void setTimerMode()
{
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0;     // turn-on timer
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER0_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER0_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER0_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
    WTIMER0_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER0_TAV_R = 0;                               // zero counter for first period
    NVIC_EN2_R |= 1 << (INT_WTIMER0A-16-64);         // turn-on interrupt 120 (WTIMER0A)
}
void Timer1Isr()
{

        frequency = WTIMER5_TAV_R;                   // read counter input
        WTIMER5_TAV_R = 0;                           // reset counter for next period
        freqUpdate = true;                           // set update flag
        GREEN_LED ^= 1;                              // status

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}
// Period timer service publishing latest time measurements every positive edge
void WideTimer0Isr()
{
        time = WTIMER0_TAV_R;                        // read counter input
        WTIMER0_TAV_R = 0;                           // zero counter for next edge
        time /= 40;                                  // scale to us units
        timeUpdate = true;                           // set update flag
        GREEN_LED ^= 1;                              // status
        WTIMER0_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}
// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}
// turn  on greenled for 500000 microsecond
void Green_led()
{
          GREEN_LED = 1;
          waitMicrosecond(500000);
          GREEN_LED = 0;
          waitMicrosecond(500000);
}
void volt_uart()
{
    char c;
    int count=0;
            putsUart0("Enter String: ");
            while(1)
            {
     loop1: c=getcUart0();
            if(c==8)                                        //comparing if char is backspace command
            {
            GREEN_LED = 1;
            waitMicrosecond(100000);
            GREEN_LED = 0;
                   if (count>0)
                   {
                       count --;
                   }
            }
           else if(c == 13)                                 //comparing if char is carriage return
                   {
                    str[count]=0;
                    GREEN_LED = 1;
                    waitMicrosecond(1000000);
                    GREEN_LED = 0;
                    break;
                   }
           else if(c<32)                                   //comparing if char is space
                   {
               goto loop1;
                  }
           else
                   {
                    str[count++]=c;
                    if(count == maxchar)
                    {
                      str[count]=0;
                      GREEN_LED = 1;
                      waitMicrosecond(1000000);
                      GREEN_LED = 0;
                      break;
                    }
                    }
}
}
//comparing string and converting delimeters to null
void comparestr()
{
    int count=strlen(str);
    for(i = 0; str[i] != '\0'; ++i)
                    {
                        if (( (str[i] >= 32 && str[i] <= 45) || (str[i] >= 58 && str[i] <= 64) || (str[i] >= 91 && str[i] <= 94)|| (str[i] >= 123 && str[i] <= 126) || (str[i] ==47) || (str[i] ==96)) )
                        {
                           str[i]=0;
                        }
                    }
                    putsUart0("\n");

   for(i=0;i<count;i++)
   {
      if (str[i] == '\0')
{
      if( (str[i+1] >= 65&& str[i+1] <= 90) || (str[i+1] >= 97 && str[i+1] <= 122)|| (str[i] ==95))
          {
          field_offset[field_count]=i+1;  //setting field offset
          field_type[field_count]='a';    //setting field type to alpha
          field_count=field_count+1;      //setting field count
            }

        else if((str[i+1] >= 48&& str[i+1] <= 57)|| (str[i+1]==46))
                    {
                           field_offset[field_count]=i+1;     //setting field offset
                           field_type[field_count]='n';       //setting field type to numeric
                           field_count=field_count+1;         //setting field count
}
}
}
}
void parsestring()
{
 for (i=0;i<field_count;i++)
    {
     // TURNING MEAS_LR ON OR OFF
    if(strcasecmp((str+field_offset[i]),"io")==0)
            {
                 if(strcasecmp((str+field_offset[i+1]),"meas_lr")==0)
                 {
                     putsUart0("measure_lr_string");
                         if(strcasecmp((str+field_offset[i+2]),"on")==0)
                                 {
                                 MEAS_LR=1;
                                 BLUE_LED=1;
                                 }
                         else if(strcasecmp((str+field_offset[i+2]),"off")==0)
                                 { MEAS_LR=0;
                                 RED_LED=1;
                                 }
                   }
                 // TURNING MEAS_C ON OR OFF
          else if(strcasecmp((str+field_offset[i+1]),"meas_c")==0)
                {
                  putsUart0("measure_c_string");
              if(strcasecmp((str+field_offset[i+2]),"on")==0)
                {
                 MEAS_C=1;
                 BLUE_LED=1;
                }
                 else if(strcasecmp((str+field_offset[i+2]),"off")==0)
                 {
                 MEAS_C=0;
                 RED_LED=1;
                 }
              // TURNING HIGHSIDE_R ON OR OFF                    }
          else if(strcasecmp((str+field_offset[i+1]),"highsider")==0)
                 {
                  putsUart0("highside_r_string");
               if(strcasecmp((str+field_offset[i+2]),"on")==0)
                 {
                 HIGHSIDE_R=1;
                 BLUE_LED=1;
                 }
                 else if(strcasecmp((str+field_offset[i+2]),"off")==0)
               {
                 HIGHSIDE_R=0;
                 RED_LED=1;
               }
               }
              // TURNING LOWSIDE_R ON OR OFF
                 else if(strcasecmp((str+field_offset[i+1]),"lowside_r")==0)
               {
                putsUart0("lowside_r_string");
               if(strcasecmp((str+field_offset[i+2]),"on")==0)
                {
                 LOWSIDE_R=1;
                 BLUE_LED=1;
                }
                 else if(strcasecmp((str+field_offset[i+2]),"off")==0)
                {
                 LOWSIDE_R=0;
                 RED_LED=1;
                }
                }
              // TURNING INTEGRATOR ON OR OFF
                 else if(strcasecmp((str+field_offset[i+1]),"integrator")==0)
               {

              if(strcasecmp((str+field_offset[i+2]),"on")==0)
                {
                  putsUart0("integrator_string");
                INTEGRATOR=1;
                BLUE_LED=1;
                }
                else if(strcasecmp((str+field_offset[i+2]),"off")==0)
                { INTEGRATOR=0;
                 RED_LED=1;
                }
                }
                }
              }
              }
              }
void case_sensetive()
{
    //CONVERTING CHARACTERS FROM UPPER CASE TO LOWER CASE IN STRING
    for(i=0;i<=strlen(str);i++)
                    {
                    if(str[i]>=65&&str[i]<=90)
                     str[i]=str[i]+32;
                    }
      putsUart0("\n");
       putsUart0(str);
 }
void iscmd()

{
    // COMPARING IF THE ENTERED VALUE IS FOR INDUCTANCE, RESISTANCE OR CAPACITANCE
    if(strcasecmp((str+field_offset[0]),"inductance")==0)
    {
        putsUart0("Entered string value is for inductance \n");
                  BLUE_LED = 1;
                     waitMicrosecond(500000);
                     BLUE_LED = 0;
                     waitMicrosecond(500000);
    }
    else if(strcasecmp((str+field_offset[0]),"resistance")==0)
        {
                         putsUart0("Entered string value is for resistance \n");
                         RED_LED = 1;
                         waitMicrosecond(500000);
                         RED_LED = 0;
                         waitMicrosecond(500000);
        }
    else if(strcasecmp((str+field_offset[0]),"capacitance")==0)
            {
                             putsUart0("Entered string value is for capacitance \n");
                             GREEN_LED = 1;
                             waitMicrosecond(500000);
                             GREEN_LED = 0;
                             waitMicrosecond(500000);
            }
            }
 void sixthstep()
 {
     //MEASURING INSTANT VOLTAGE
     float instantvolt;
         char str5[10];
         char str6[10];
         for (i=0;i<field_count;i++)
             {
             if(strcasecmp((str+field_offset[i]),"io")==0)
                     {
                          if(strcasecmp((str+field_offset[i+1]),"meas")==0)
                          {

                                  if(strcasecmp((str+field_offset[i+2]),"voltage")==0)
                                          {
                                      putsUart0("measure_voltage");
                                      MEAS_LR=0;
                                      MEAS_C=1;
                                      INTEGRATOR=1;
                                      while(1)
                                           {
                                               raw = readAdc0Ss3();
                                               instantvolt = ((raw / 4096.0 * 3.3)-0.021);    //FORMULA TO CALCULATE INSTANT VOLTAGE WOTH OFFSET 0.021
                                               putsUart0("\r\n");
                                               sprintf(str5, "%u", raw);                      //CONVERTING NUMERIC VALUE OF STRING TO STR
                                               putsUart0(str5);
                                               putsUart0("\r\n");
                                               sprintf(str6, "%f", instantvolt);              //CONVERTING NUMERIC VALUE OF INSTANT VOLTAGE TO STRING
                                               putsUart0(str6);
                                               putsUart0("\r\n");
                                               waitMicrosecond(500000);
                                           }
                                              }
                                              }
                                            }
                                           }
 }
 void sixthbstep()
 {
     //MEASURING IIR VOLTAGE
       char str7[10];
       char str8[10];
       float alpha = 0.99;
       int firstUpdate = true;
              for (i=0;i<field_count;i++)
                  {
                  if(strcasecmp((str+field_offset[i]),"io")==0)
                          {
                               if(strcasecmp((str+field_offset[i+1]),"meas")==0)
                               {

                                       if(strcasecmp((str+field_offset[i+2]),"iirvoltage")==0)
                                               {
                                           putsUart0("measure_iirvoltage");


                                               RED_LED=1;

                                       while(1)
                                                {
                                           raw = readAdc0Ss3();
                                           instantvolt = ((raw / 4096.0 * 3.3)-0.021) ;  //CALCULATING INSTANT VOLTAGE
                                            if (firstUpdate)
                                                   {
                                                       iirvolt = instantvolt;
                                                       firstUpdate = false;
                                                   }
                                                   else
                                                       iirvolt = iirvolt * alpha + instantvolt * (1-alpha); //CALCULATING IIR VOLTAGE

                                                   // display raw ADC value and VOLTAGE
                                                   sprintf(str7, "%u", raw);
                                                   putsUart0("\r\n");
                                                   putsUart0(str7);
                                                   sprintf(str7, "%f", instantvolt);
                                                   putsUart0("\r\n");
                                                   putsUart0(str8);
                                                   sprintf(str8, "%f", iirvolt);
                                                   putsUart0("\r\n");
                                                   putsUart0(str8);
                                                   waitMicrosecond(500000);
                                               }
                                                     }
                               }}}}
 void testmode()
 {
       // MEASURING TIME
     for (i=0;i<field_count;i++)
                       {
         if(strcasecmp((str+field_offset[i]),"io")==0)
                                   {
                                        if(strcasecmp((str+field_offset[i+1]),"meas")==0)
                                        {

                                                if(strcasecmp((str+field_offset[i+2]),"time")==0)
                                                        {
                                           putsUart0("measure_time");
                                           LOWSIDE_R=1;
                                           INTEGRATOR=1;
                                           waitMicrosecond(1000000);
                                           LOWSIDE_R=0;
                                           WTIMER0_CTL_R |= TIMER_CTL_TAEN;      //ENABLING TIMER
                                           HIGHSIDE_R=1;
                                              while (1)
                                               {

                                                   if (timeUpdate)               //CHECKING IF TIME IS UPDATING OR NOT
                                                   {
                                                       WTIMER0_ICR_R = TIMER_ICR_CAECINT;  //GPTM Timer A Capture Mode Event Interrupt Clear
                                                       timeUpdate = false;
                                                       sprintf(str, "%7lu", time);
                                                       putsUart0(str);
                                                   }

                                                   }
                                                   }
                                               }
                                               }
 }
 }

 void testmode8th()
 // MEASURING RESISTOR
  {
     char res[100];
      for (i=0;i<field_count;i++)
                        {
          if(strcasecmp((str+field_offset[i]),"res")==0)
                                    {
                                         putsUart0("measure_resistor");
                                           LOWSIDE_R=1;

                                            INTEGRATOR=1;
                                            waitMicrosecond(1000000);
                                            LOWSIDE_R=0;
                                            WTIMER0_CTL_R |= TIMER_CTL_TAEN; //ENABLING TIMER
                                            MEAS_LR=1;
                                               while (1)
                                                {

                                                    if (timeUpdate)
                                                    {
                                                       float time2 = time/1.12;
                                                        timeUpdate = false;
                                                        sprintf(res, "%f", time2);
                                                        putsUart0("\r \n");
                                                        putsUart0(res);
                                                        WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;  //TURNING OFF TIMER
                                                        break;
                                                     }
                                                   }
                                                    }
                                                }
                                                }
// FUNCTION TO CALCULATE RESISTOR VALUE IN STEP11(AUTO MODE)
 void res11()
 {
                                                char res[100];
                                                 putsUart0("measure_resistor");
                                                LOWSIDE_R=1;
                                                 INTEGRATOR=1;
                                                 waitMicrosecond(1000000);
                                                 LOWSIDE_R=0;
                                                 WTIMER0_CTL_R |= TIMER_CTL_TAEN;      //ENABLING TIMER
                                                 MEAS_LR=1;
                                                    while (1)
                                                     {
                                                        if (timeUpdate)
                                                         {
                                                             time = time/1.12;
                                                             timeUpdate = false;
                                                             sprintf(res, "%d", time);
                                                             putsUart0("\r \n");
                                                             putsUart0(res);
                                                             ResetISR();                //CALLING SOFTWARE RESET
                                                              break;
                                                         }

                                                         }
                                                         }
void testmode9th()
  {
    //CALCULATING CAPACITOR VALUE
    for (i=0;i<field_count;i++)
                        {
          if(strcasecmp((str+field_offset[i]),"cap")==0)
                                    {
                                        putsUart0("measure_capacitor");
                                         LOWSIDE_R=1;
                                            MEAS_C=1;
                                            waitMicrosecond(10000);
                                            LOWSIDE_R=0;
                                            timeUpdate=false;
                                            WTIMER0_CTL_R |= TIMER_CTL_TAEN;   //TURNING ON TIMER
                                            WTIMER0_TAV_R=0;                  // zero counter for first period
                                            HIGHSIDE_R=1;
                                               while (1)
                                                {
                                                    if (timeUpdate)
                                                    {
                                                        timeUpdate = false;
                                                        float cap = time/143000.0;  //CALCULATING CAPACITANCE
                                                        sprintf(str, "%f", cap);
                                                        putsUart0(str);
                                                       ResetISR();                  //SOFTWARE RESET
                                                        break;

                                                    }
                                                }
                                                    }
                                                    }
                                                }
//FUNCTION TO CALCULATE CAPACITOR IN STEP11
 void cap11()
   {
                                             putsUart0("measure_capacitor");
                                             LOWSIDE_R=1;
                                             MEAS_C=1;
                                             waitMicrosecond(1000000);
                                             LOWSIDE_R=0;
                                             timeUpdate=false;
                                             WTIMER0_CTL_R |= TIMER_CTL_TAEN;             //TURNING ON TIMER
                                             WTIMER0_TAV_R=0;
                                             HIGHSIDE_R=1;
                                                  raw = readAdc0Ss3();
                                       float instant_voltage = ((raw / 4096.0) * 3.3);    //CALCULATING INSTANTANEOUS VOLTAGE

                                                while (1)
                                                 {
                                                 if (timeUpdate)
                                                     {
                                                         timeUpdate = false;
                                                         int temp = time;
                                                         float cap = temp/143000.0;       //CALCULATING CAPACITANCE
                                                         putsUart0("\r\n");
                                                         sprintf(str, "%f", cap);
                                                         putsUart0(str);
                                                         ResetISR();                  //SOFTWARE RESET
                                                         break;

                                                     }
                                                 }
                                                     }
//MEASURING ESR
void testmode10th()
   {
     uint16_t raw=0;
     float esr;
     float v;
     char str2[81];
       for (i=0;i<field_count;i++)
                         {
           if(strcasecmp((str+field_offset[i]),"esr")==0)
                                     {
                                             putsUart0("measure_esr");
                                             LOWSIDE_R=1;
                                             MEAS_LR=1;
                                             waitMicrosecond(1000000);
                                             raw = readAdc0Ss3();
                                              v = ((raw / 4096.0 * 3.3)-0.021);    //CALCULATING INSTANTANEOUS VOLTAGE
                                            putsUart0("measure_v is ");
                                             sprintf(str2,"%f",v);
                                             putsUart0(str2);
                                             esr =(96/v)-33;                       //CALCULATING ESR
                                             putsUart0(str);
                                             putsUart0("\r \n");
                                             WTIMER0_CTL_R |= TIMER_CTL_TAEN;      //ENABLING TIMER
                                             LOWSIDE_R=0;
                                             MEAS_LR=0;
                                             MEAS_C=1;
                                             waitMicrosecond(1000000);
                                                     }
                                                 }
                                                 }
//MEASURING INDUCTOR
void testinductor()
   {
     uint16_t raw=0;
     float esr;
     float v;
     char str2[81];
       for (i=0;i<field_count;i++)
                         {
           if(strcasecmp((str+field_offset[i]),"ind")==0)
                                     {
                                           HIGHSIDE_R=0;
                                           MEAS_C=0;
                                           INTEGRATOR=0;
                                             putsUart0("measure_inductor");
                                             LOWSIDE_R=1;
                                             MEAS_LR=1;
                                            waitMicrosecond(200000);
                                             raw = readAdc0Ss3();
                                            v = ((raw / 4096.0 * 3.3)-0.021);     //INSTANTANEOUS VOLTAGE
                                            esr =(87/v)-33;                       //MEASURING ESR
                                            MEAS_LR=0;
                                            waitMicrosecond(200000);
                                            WTIMER0_CTL_R |= TIMER_CTL_TAEN;      //TURNING ON TIMER
                                            MEAS_LR=1;
                                            sprintf(str,"%f",esr);
                                             putsUart0("value of Esr is ");
                                             putsUart0(str);
                                             while(1){
                                              if (timeUpdate)
                                                     {
                                                         timeUpdate=false;
                                                         putsUart0("\r \n");
                                                         float l= ((33+esr)*time)/1.58;
                                                         putsUart0("\r \n");
                                                         sprintf(str, "%f",l);
                                                         putsUart0("value of inductor is");
                                                         putsUart0(str);
                                                         ResetISR();
                                                         break;


                                                     }

                                                     }
                                                     }
                                                 }
                                                 }
//FUNCTION TO CALCULATE INDUCTOR IN STEP11
void ind11()
{
         uint16_t raw=0;
         float esr;
         float v;
        // char str2[81];
                                               HIGHSIDE_R=0;
                                               MEAS_C=0;
                                               INTEGRATOR=0;
                                                 putsUart0("measure_inductor");
                                                 LOWSIDE_R=1;
                                                 MEAS_LR=1;
                                                waitMicrosecond(200000);
                                                 raw = readAdc0Ss3();
                                                v = ((raw / 4096.0 * 3.3)-0.021);     //INSTANTANEOUS VOLTAGE
                                                esr =(87/v)-33;
                                                MEAS_LR=0;
                                                waitMicrosecond(200000);
                                                WTIMER0_CTL_R |= TIMER_CTL_TAEN;       //TURNING ON TIMER
                                                MEAS_LR=1;
                                                while(1){
                                                  if (timeUpdate)
                                                         {
                                                             timeUpdate=false;
                                                             float l= ((33+esr)*time)/1.58;   //CALCULATING INDUCTANCE
                                                             putsUart0("\r \n");
                                                             sprintf(str, "%f",l);
                                                             putsUart0("value of inductor is");
                                                             putsUart0("\r \n");
                                                             putsUart0(str);
                                                             ResetISR();                  //SOFTWARE RESET
                                                             break;
                                                         }
                                                         }
                                                         }
//DETECTING WHETHER WE HAVE INDUCTOR,RESISTOR OR CAPACITOR
void auto_check()
 {
     for (i=0;i<field_count;i++)
                             {
               if(strcasecmp((str+field_offset[i]),"autocheck")==0)
                                         {
    // uint16_t i=0;
     //int set=0;
     cap = false;
    //float voltage[10];
     //float instant_voltage=0;
     WTIMER0_TAV_R = 0;                                          // zero counter for first period
     timeUpdate=false;
     MEAS_C=1;
     LOWSIDE_R=1;
     waitMicrosecond(10000);
     WTIMER0_CTL_R |= TIMER_CTL_TAEN;                            //TURNING ON TIMER
     LOWSIDE_R=0;
     HIGHSIDE_R=1;
     putsUart0("\r\n");
     timeUpdate=false;
     int k = 0;
    while(k <= 70)                                               //CHECKING IF TIME IS GETTING UPDATE OR NOT FOR 6 SECONDS
        {if(timeUpdate)
     {   timeUpdate=false;
         putsUart0("cap");
         WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;                      //DISABLING TIMER
         HIGHSIDE_R=0;
         MEAS_C = 0;
         cap11();
         cap=true;                                              // CAP IS TAKEN AS ANY VARIABLE TO SET OR UNSET FLAG
         ResetISR();
         break;

     }
        else
        {
            waitMicrosecond(200000);
            k++;
        }
        }
       if(cap ==false)                                        // CAP IS TAKEN AS ANY VARIABLE TO SET OR UNSET FLAG
       {
         MEAS_C=1;
         INTEGRATOR=1;
         HIGHSIDE_R=0;
         LOWSIDE_R=1;
         waitMicrosecond(500000);
         MEAS_C=0;
         LOWSIDE_R=0;
         WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;                     //DISABLING THE TIMER
         timeUpdate=false;
         LOWSIDE_R=0;
         WTIMER0_CTL_R |= TIMER_CTL_TAEN;                      //ENABLING THE TIMER
         MEAS_LR=1;
         WTIMER0_TAV_R = 0;                                    // zero counter for first period
         while(1)
         {
         if(timeUpdate)
             {
             sprintf(str, "%d",time);
                         putsUart0("time is");
                         putsUart0(str);
                          putsUart0("\r \n");
             if(time<=45)                                      //CHECKING IF INDUCTOR IS TRIGGERING FOR THE TIME LESS THAN OR EQUAL TO 45
             {
             timeUpdate=false;

              putsUart0("ind");
              WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;               //DISABLING THE TIMER
              MEAS_LR=0;
                                     LOWSIDE_R=0;
                                     INTEGRATOR=0;
                                     MEAS_LR=0;
                                     WTIMER0_TAV_R = 0;       // zero counter for first period
              ind11();
              ResetISR();
              break;
             }

         else
         {
             putsUart0("res");
         WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;                    //DISABLING THE TIMER
         timeUpdate=false;
         MEAS_LR=0;
         INTEGRATOR=0;
         res11();
         //ResetISR();
         break;
         }
             }
        // ResetISR();
             }
         }
     }
 }
 }
//RESET FUNCTION
void reset()
{
    for (i=0;i<field_count;i++)
                                {
                  if(strcasecmp((str+field_offset[i]),"reset")==0)
                                            {
                      ResetISR();
                      //TURNING ALL PINS OFF
                      HIGHSIDE_R=0;
                      MEAS_LR=0;
                      LOWSIDE_R=0;
                      MEAS_C=0;
                      INTEGRATOR=0;
                                            }
                                }
    }
int main(void)
{

    // Initialize hardware
    while(1)
    {
    initHw();
   // setTimerMode();
    Green_led();
    volt_uart();
    comparestr();
     iscmd();
    parsestring();
    sixthstep();
    sixthbstep();
   testmode();
   testmode8th();
   testmode9th();
   testmode10th();
   testinductor();
   auto_check();
   reset();
    }
 }

