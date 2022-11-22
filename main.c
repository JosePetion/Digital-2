/*--------------------------------------------------------------------
 *----------------PROTECTO 3------------------------------------------
 *  JOSE PABLO PETION RIVAS
 *  Carne: 201151
---------------------------------------------------------------------*/
//-----------------Estado 1 -------Comunicacion Uart
//-----------------Estado 2 -------Contadores manales y timer
//-----------------Estado 3 -------Animación con diferentes t.
//-----------------Estado 4 -------Contador con ADC

/*---------------------------------------------------------------------
 *----------------BIBLIOTECAS------------------------------------------
---------------------------------------------------------------------*/

#include<stdint.h>
#include<stdbool.h>
#include"inc/hw_memmap.h"
#include "driverlib/interrupt.h"
#include "inc/hw_types.h"
#include"inc/tm4c123gh6pm.h"
#include "driverlib/pin_map.h"
#include"driverlib/sysctl.h"
#include"driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/timer.h"
/*--------------------------------------------------------------------
 *------------------VARIABLES-----------------------------------------
---------------------------------------------------------------------*/
#define FS 40000
#define IN_MIN 0                  // Valor minimo de entrada del potenciometro
#define IN_MAX 4095               // Valor máximo de entrada del potenciometro
#define OUT_MIN 0                 // Valor minimo de salida
#define OUT_MAX 99                // Valor máximo de salida

/*--------------------------------------------------------------------
 *------------------VARIABLES-----------------------------------------
---------------------------------------------------------------------*/
int status;
int i;
uint32_t muestra, lectura;
uint32_t ui32Period, periodo;
uint8_t read, unidad, decena;                                               //Lec. UART
uint8_t LED1_STS=0, LED2_STS=0, LED3_STS=0, LED4_STS=0;     //Sts. LEDs
uint8_t contador=0, c_disp=0, trigger=0, ref = 0, value=0;
char display[16] = {0b01111110, 0b00110000, 0b01101101, 0b01111001, 0b00110011, 0b01011011, 0b01011111,
                    0b01110000, 0b01111111, 0b01111011, 0b01110111, 0b00011111, 0b01001110, 0b00111101,
                    0b01001111, 0b01000111};
char animacion0[10] = {0b01000000, 0b00100000, 0b00010000, 0b00001000, 0b00000100, 0b00000010, 0b00000001,
                       0b11111111, 0b11111110, 0b00000000};
char animacion1[10] = {0x01, 0x02, 0x04, 0x08, 0x00, 0x09, 0x0F, 0x09, 0x0F, 0};
/*--------------------------------------------------------------------
 *------------------Declaracion de funciones--------------------------
---------------------------------------------------------------------*/

void UARTsend(const uint8_t *bufferTX);
void UARTReadIntHandler(void);
void PORTAIntHandler(void);
void ObtenerMuestra();
void separacion(void);
unsigned short map(uint32_t val, uint32_t in_min, uint32_t in_max,
            unsigned short out_min, unsigned short out_max);
/*--------------------------------------------------------------------
 * --------------------------------VOID MAIN--------------------------
 --------------------------------------------------------------------*/
int main(void) {

    //Establecer reloj del microcontrolador
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    //COMUNICACIÓN UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // The ADC0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    //Habilitar periférico GPIO F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    //Habilitar pines de salida y entrada
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7);
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    //timer0
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ui32Period = (SysCtlClockGet()) / 2;
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    //timer1
    TimerConfigure(TIMER1_BASE, TIMER_CFG_A_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, (SysCtlClockGet())/FS);
    TimerControlTrigger(TIMER1_BASE, TIMER_A, true);
    TimerEnable(TIMER1_BASE, TIMER_A);
    //PUERTOS ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 1);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH2|ADC_CTL_IE|ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntEnable(ADC0_BASE, 3);
    ADCIntRegister(ADC0_BASE, 3, ObtenerMuestra);
    IntEnable(INT_ADC0SS3);
    //Configuracion de la consola
    UARTStdioConfig(0, 115200, 16000000);
    //PUERTOS DEL UART
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
     (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    //---------------------------------INTERRUPCIONES DE DIPSWITCH-------------------------------------------------
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    //GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_2|GPIO_INT_PIN_3);
    //---------------------------------Interrupciones de switch----------------------------------------------------
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6|GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_6|GPIO_INT_PIN_7);
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_INT_PIN_6|GPIO_INT_PIN_7, GPIO_LOW_LEVEL);
    IntPrioritySet(INT_GPIOA, 0);
    IntRegister(INT_GPIOA, PORTAIntHandler);
    IntEnable(INT_GPIOA);
    //-------------------------------------------------------------------------------------------------------------
    // MENU DESPLEGADO
    //Interrupción del UART
    IntEnable(INT_UART0);
    UARTFIFODisable(UART0_BASE);
    UARTIntEnable(UART0_BASE, UART_INT_RX);
    IntMasterEnable();
    TimerEnable(TIMER0_BASE, TIMER_A);  //Habilitamos Timer0

    UARTsend("Intrucciones:\n");
    UARTsend("Control Comando\n");
    UARTsend("LED1       a   \n");
    UARTsend("LED2       b   \n");
    UARTsend("LED3       d   \n");
    UARTsend("LED4       d   \n");
    UARTsend("DISPLAY   1-9  \n");
    UARTsend("\n Send: \n");

    while(true){
        status = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3);
        //Si el botón está presionado enciende el LED y si no, lo apaga.
        switch (status) {
        //---------------------------------------------------------------------------------Estado 1---------------------------------------------------------------------------
        case 0:
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
            //CONTROL POR UART ESTADO 1
                switch (read){
                    case '0':
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[0]);
                        break;
                    case '1':
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[1]);
                        break;
                    case '2':
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[2]);
                        break;
                    case '3':
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[3]);
                        break;
                    case '4':
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[4]);
                        break;
                    case '5':
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[5]);
                        break;
                    case '6':
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[6]);
                        break;
                    case '7':
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[7]);
                        break;
                    case '8':
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[8]);
                        break;
                    case '9':
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[9]);
                        break;
                    case 'A':
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[10]);
                        break;
                    case 'B':
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[11]);
                        break;
                    case 'C':
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[12]);
                        break;
                    case 'D':
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[13]);
                        break;
                    case 'E':
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[14]);
                        break;
                    case 'F':
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[15]);
                        break;
                    case '.':
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0x80);
                        break;
                    case 'a':
                        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, LED1_STS);
                        break;
                    case 'b':
                        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, LED2_STS);
                        break;
                    case 'c':
                        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, LED3_STS);
                        break;
                    case 'd':
                        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, LED4_STS);
                        break;
                    default:
                        break;
                }
            break;
        //---------------------------------------------------------------------------------Estado 2---------------------------------------------------------------------------
        case 4:
            if(contador==16){contador=0;}
            if(contador==255){contador=15;}
            if(c_disp>16){c_disp=0;}
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 4);
            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, contador);
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[c_disp]);
            break;
        //---------------------------------------------------------------------------------Estado 3---------------------------------------------------------------------------
        case 8:
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 8);
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, animacion0[ref]);
            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, animacion1[ref]);
            if(ref>9 && trigger==0){
                trigger=1;
                ref=0;
            }
            if(ref>9 && trigger==1){
                trigger=2;
                ref=0;
            }
            if(ref>9 && trigger==2){
                trigger=0;
                ref=0;
            }

            break;
        case 12:
            //---------------------------------------------------------------------------------Estado 4---------------------------------------------------------------------------
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 2);
            separacion();
            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, decena);
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[unidad]);
            break;
        }
}
}

//-----------------Funcion para enviar un string completo---------------
void UARTsend(const uint8_t *bufferTX){
    uint8_t C=0;
    while(*bufferTX!=C){
        UARTCharPut(UART0_BASE, *bufferTX++);
    }
}
//---------------------------Interrupcion de Handler--------------------
void UARTReadIntHandler(void){
    UARTIntClear(UART0_BASE, UART_INT_RX);
    read = UARTCharGet(UART0_BASE);
    UARTCharPut(UART0_BASE, read);
    UARTCharPut(UART0_BASE, 10);

    //revisa el status para saber que funcion del uart cumplir
    switch (status){
            //Accion del estado 1
            case 0:
            switch(read){
                case 'a':
                    LED1_STS=~LED1_STS;
                    break;
                case 'b':
                    LED2_STS=~LED2_STS;
                    break;
                case 'c':
                    LED3_STS=~LED3_STS;
                    break;
                case 'd':
                    LED4_STS=~LED4_STS;
                    break;
                default:
                    break;
            }
            default:
                break;
    }
}
//---------------------Interrupcion de puerto A -------------------------
void PORTAIntHandler(void){
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_6|GPIO_INT_PIN_7);
    int boton = GPIOIntStatus(GPIO_PORTA_BASE, true);

switch(status){

    case 0:               //ACCION DE BOTONES CON EL ESTADO 1
    switch (boton){
                    case 64:
                        if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6)==0){
                            while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6)==0){}
                            UARTprintf("\nOBJ.  STS\n");
                            UARTprintf("LED1  %d\n", LED1_STS&0b1);
                            UARTprintf("LED2  %d\n", LED2_STS&0b1);
                            UARTprintf("LED3  %d\n", LED3_STS&0b1);
                            UARTprintf("LED4  %d\n", LED4_STS&0b1);
                            UARTprintf("Display ");
                            UARTCharPut(UART0_BASE, read);
                        }
                        break;
                    case 128:
                        if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7)==0){

                        while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7)==0){}
                            UARTprintf("\n OBJ.  STS\n");
                            UARTprintf("\nLED1 %d\n", LED1_STS&0b1);
                            UARTprintf("LED2 %d\n", LED2_STS&0b1);
                            UARTprintf("LED3 %d\n", LED3_STS&0b1);
                            UARTprintf("LED4 %d\n", LED4_STS&0b1);
                            UARTprintf("Display ");
                            UARTCharPut(UART0_BASE, read);
                        }
                        break;

            }
    break;                  //FIN

    case 4:                 //ACCION DE BOTONES CON EL ESTADO 2
    switch (boton){
                    case 64:
                        if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6)==0){
                            while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6)==0){}
                            contador++;
                        }
                        break;
                    case 128:
                        if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7)==0){

                            while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7)==0){}
                            contador--;
                        }
                        break;

            }
    break;                  //FIN
//        default:
//            break;
    }
}

//**************************************************************************************************************
// Handler de la interrupciÃ³n del TIMER 0 - Recordar modificar el archivo tm4c123ght6pm_startup_css.c
//**************************************************************************************************************
void Timer0IntHandler(void)
{
    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Read the current state of the GPIO pin and
    // write back the opposite state
    switch (status){
        case 4:
            c_disp++;
            if(c_disp>15){c_disp=0;}
            UARTprintf("Contador LEDS: %d\n", contador);
            UARTprintf("Contador Disp: %d\n", c_disp);
            break;
        case 8:
            value++;
            switch (trigger){
            case 0:
                if (value==5){
                    ref++;
                    value=0;
                }
                break;
            case 1:
                if (value==3){
                    ref++;
                    value=0;
                }
                break;
            case 2:
                if (value==1){
                    ref++;
                    value=0;
                }
                break;
            default:
                break;
            }
        break;
    }
}

//---------------------------------------Interrupcion del ADC-----------------------------------
void ObtenerMuestra(){
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, &muestra);
    lectura=map(muestra, IN_MIN,IN_MAX, OUT_MIN, OUT_MAX);

}
//--------------------------------------Interpolación--------------------------------------------

unsigned short map(uint32_t x, uint32_t x0, uint32_t x1,
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}

//----------------------------------------Valores------------------------------------------------
void separacion(void){
    decena = lectura/10;
    unidad = lectura%10;
}
