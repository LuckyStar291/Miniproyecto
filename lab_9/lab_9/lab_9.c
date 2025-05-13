#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "inc/hw_nvic.h"
#include "utils/uartstdio.h"
#include "driverlib/timer.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"

// ==============================
//        Variables globales
// ==============================
uint32_t ui32SysClock;
char data[32];
uint32_t load;
uint32_t g_ui32Flags = 0;
uint32_t g_ui32States = 0;
uint32_t count = 0;

// ==============================
//       Prototipos Funciones
// ==============================
static void configureSysClock(void);
static void configGPIO(void);
static void configUART(void);
//static void configInterrupt(void); // No usada aún
//static void configTimer1(void);    // No usada aún
void ProcesarCadena(void);
//static void config_adc(void);     // No usada aún
uint32_t ReadADC(void);
void ButtonISR(void);               // Declaración adelantada
void configPwm(void);

int main(void) {
    configureSysClock();
    configGPIO();
    configUART();
    //configTimer1();
    UARTprintf("antes del config\r\n");
    load = 50000;

    configPwm();

    UARTprintf("\033[2J\n\nUART Listo\r\n");

    while (1) {
        ProcesarCadena();
    }
}

// ===============================
//      CONFIGURACIONES
// ===============================
static void configureSysClock(void) {
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                       SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_PLL |
                                       SYSCTL_CFG_VCO_240), 120000000);
}

static void configGPIO(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)) {}

    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, 0x03);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, 0x3C);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, 0x1F);

    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, 0x03);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, 0x03, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

static void configUART(void) {
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, ui32SysClock, 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    UARTEnable(UART0_BASE);
    UARTStdioConfig(0, 115200, ui32SysClock);
}

// ISR para botón
void ButtonISR(void) {
    GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0);
}

// Interrupciones (comentado si no se usa)
static void configInterrupt(void) {
    GPIOIntDisable(GPIO_PORTJ_BASE, GPIO_PIN_0);
    GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0);
    GPIOIntRegister(GPIO_PORTJ_BASE, ButtonISR);
    GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_PIN_0);
    MAP_IntEnable(INT_GPIOJ);
    MAP_IntMasterEnable();
}

void ProcesarCadena(void) {
    if (UARTCharsAvail(UART0_BASE)) {
        UARTgets(data, sizeof(data));
        UARTprintf("data recibida: %s\n", data);
        
        char *pos;
        if ((pos = strchr(data, '\n')) != NULL) *pos = '\0';
        if ((pos = strchr(data, '\r')) != NULL) *pos = '\0';

        if (strcmp(data, "LA") == 0) {
            GPIOPinWrite(GPIO_PORTN_BASE, 0x0C, 0x04);
            GPIOPinWrite(GPIO_PORTN_BASE, 0x30, 0x10);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, load * 0.6);  
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, load * 0.9);  
            UARTprintf("LA...\n");

        } else if (strcmp(data, "LS") == 0) {
            UARTprintf("LS...\n");
            UARTprintf("load = %d\n", load);
            GPIOPinWrite(GPIO_PORTN_BASE, 0x0C, 0x08);
            GPIOPinWrite(GPIO_PORTN_BASE, 0x30, 0x10);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, load * 0.7);  
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, load * 0.7); 
            SysCtlDelay(120000000 / 3 * 2);

        } else if (strcmp(data, "RA") == 0) {
            UARTprintf("RA...\n");
            GPIOPinWrite(GPIO_PORTN_BASE, 0x0C, 0x04);
            GPIOPinWrite(GPIO_PORTN_BASE, 0x30, 0x10);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, load * 0.9);  
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, load * 0.7);
            SysCtlDelay(120000000 / 3 * 2);
        } else if (strcmp(data, "RS") == 0) {
            UARTprintf("RS...\n");
            GPIOPinWrite(GPIO_PORTN_BASE, 0x0C, 0x04);
            GPIOPinWrite(GPIO_PORTN_BASE, 0x30, 0x20);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, load * 0.7);  
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, load * 0.7);
            SysCtlDelay(120000000 / 3 * 2);
        } 

        else if (strcmp(data, "SA") == 0) {
            UARTprintf("SA...\n");
            GPIOPinWrite(GPIO_PORTN_BASE, 0x0C, 0x04);
            GPIOPinWrite(GPIO_PORTN_BASE, 0x30, 0x10);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, load * 0.6);  
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, load * 0.6);
            SysCtlDelay(120000000 / 3 * 2);
        } 
        else if (strcmp(data, "SS") == 0) {
            UARTprintf("SS...\n");
            GPIOPinWrite(GPIO_PORTN_BASE, 0x0C, 0x00);
            GPIOPinWrite(GPIO_PORTN_BASE, 0x30, 0x00);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, load * 0.7);  
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, load * 0.7);
            SysCtlDelay(120000000 / 3 * 2);
        } 
        else if (strcmp(data, "r") == 0) {
            UARTprintf("RA...\n");
            GPIOPinWrite(GPIO_PORTN_BASE, 0x0C, 0x08);
            GPIOPinWrite(GPIO_PORTN_BASE, 0x30, 0x20);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, load * 0.7);  
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, load * 0.7);
            SysCtlDelay(120000000 / 3 * 2);
        }
        else if (strcmp(data, "P") == 0) {
            UARTprintf("P...\n");
            UARTprintf("load = %d\n", load);
            GPIOPinWrite(GPIO_PORTN_BASE, 0x0C, 0x04);
            GPIOPinWrite(GPIO_PORTN_BASE, 0x30, 0x20);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, load * 0.6);  
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, load * 0.6);
            SysCtlDelay(120000000 / 3 * 2);
        } 
        else{
            UARTprintf("%s desde la Tiva\n", data);
        }
    }
}

// ===============================
// FUNCIONES PWM (placeholder)
// ===============================

// Configura un generador PWM para dos salidas (canales A y B)
void configPwm(void){
    // 1. Habilitar los módulos PWM y GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);  // Cambia según tu pinout
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    // 2. Configurar los pines alternativos para PWM
    GPIOPinConfigure(GPIO_PK4_M0PWM6);
    GPIOPinConfigure(GPIO_PK5_M0PWM7);
    GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_4 | GPIO_PIN_5);
// 3. Configurar el generador

    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, load);
    
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, load*0.1);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, load*0.1);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));

    // Habilitar salidas
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
    PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true );
    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true );

}


// ===============================
// FUNCIONES ADICIONALES
// ===============================
uint32_t ReadADC(void) {
    return 0; // Placeholder
}
