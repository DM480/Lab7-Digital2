#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"


// Llamado para saber si existe un error en la libreria

#ifdef DEBUG
void
_error_(char *pcFilename, uint32_t ui32Line) {
    while(1);
}
#endif

//Variables
uint32_t LED = GPIO_PIN_1;
unsigned char non = 'x';
unsigned char no  = 'y';
char var = 0;
char BAN = 0;

//Mencion externa de funciones
extern void Timer0IntHandler(void);
extern void UARTIntHandler(void);

int main(void) {
    SysCtlClockSet(SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ | SYSCTL_USE_PLL | SYSCTL_SYSDIV_5); // Configuración de reloj 40 MHz

    // PUERTO F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    IntMasterEnable(); // Interrupciones generales

    // Configuración Timer0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/2 - 1); // COnfiguración de 32 bits
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0IntHandler);
    IntEnable(INT_TIMER0A);  // Habilitacion del Sub timer A
    TimerEnable(TIMER0_BASE, TIMER_A);

    // Configuración UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Habilitación de los GPIO correspondientes al módulo UART
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);  // Habilitación de módulo UART0
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    GPIOPinConfigure(0x00000001); // RX
    GPIOPinConfigure(0x00000401); // TX
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    UARTFIFOEnable(UART0_BASE);
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    UARTIntClear(UART0_BASE, UART_INT_RX | UART_INT_RT);
    UARTIntRegister(UART0_BASE,UARTIntHandler);
    UARTEnable(UART0_BASE);

    // Salida en pines 1, 2 y 3 para LEDS
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);

    while(1);
}

void Timer0IntHandler(void) {
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    switch(var){
    case 0:
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, LED);
        var++;
        break;
    case 1:
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
        var = 0;
        break;
    }
}

void UARTIntHandler(void) {
    UARTIntClear(UART0_BASE, UART_INT_RX | UART_INT_RT);
    if (UARTCharsAvail(UART0_BASE)){
        non = UARTCharGet(UART0_BASE);
    }
    if(no != non) {
        IntEnable(INT_TIMER0A);
        if(non == 'r') {
            LED = GPIO_PIN_1;
            no = non;
            non = 'x';
        }
        else if(non == 'b') {
            LED = GPIO_PIN_2;
            no = non;
            non = 'x';
        }
        else if(non == 'g') {
            LED = GPIO_PIN_3;
            no = non;
            non = 'x';
        }
        else{
        }
    }
    else{
        switch(BAN){
            case 0:
                IntDisable(INT_TIMER0A);
                BAN++;
                break;
            case 1:
                IntEnable(INT_TIMER0A);
                BAN = 0;
                break;
        }
    }
}
