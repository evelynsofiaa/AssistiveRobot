
// Configuraci�n del microcontrolador
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2 // System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = XT_XT     // Oscillator Selection bits (XT oscillator (XT))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

#include <xc.h>
#include <stdio.h>

#define _XTAL_FREQ 4000000          // 4 MHz crystal frequency

// PORT pins
#define RS      RC0      // RS pin LCD display
#define RW      RC1      // RW pin LCD display
#define EN      RC2      // EN pin LCD display

// Constants
#define Delay_LCD       5           // Delay used for LCD ENABLE pin
#define Delay_Shift     50          // Delay used to shift characters 

#define ClrScreen       0x01        // LCD clear display screen
#define ReturnHome      0x02        // LCD return home
#define DecCursor       0x04        // LCD decrement cursor (shift cursor to left)
#define IncCursor       0x06        // LCD increment cursor (shift cursor to right)
#define ShiftRight      0x05        // Shift display right
#define ShiftLeft       0x07        // Shift display left
#define DispOFFCurOFF   0x08        // Display OFF, cursor OFF
#define DispOFFCurON    0x0A        // Display OFF, cursor ON
#define DispONCurOFF    0x0C        // Display ON, cursor OFF
#define DispONCurBk     0x0E        // Display ON cursor Blinking
#define DispOFFCurBk    0x0F        // Display OFF cursor Blinking
#define ShiftCurLeft    0x10        // Shift cursor position to left
#define ShiftCurRight   0x14        // Shift cursor position to right
#define ShiftDispLeft   0x18        // Shift entire display to the left
#define ShiftDispRight  0x1C        // Shift entire display to the right
#define FirstLine       0x80        // Cursor at the beginning of the first line
#define SecondLine      0xC0        // Cursor at the beginning of the second line
#define TwoLines57Mat   0x38        // Two lines, 5x7 matrix
char dataT[10];
char dataL[10];

//define inputs
#define X_axis_1 RA1  // Pin RA1 para la señal digital del eje X arriba del comparador
#define X_axis_0 RA2  // Pin RA2 para la señal digital del eje X abajo del comparador
#define Y_axis_1 RA3 // Pin RA3 para la señal digital del eje Y arriba del comparador
#define Y_axis_0 RA4  // Pin RA4 para la señal digital del eje Y abajo del comparador

// Define outputs (L298N control pins)
// Primer L298N (Motores lado derecho)
#define IN1 RB0  // Controla la dirección del motor derecho 1
#define IN2 RB1  // Controla la dirección del motor derecho 1
#define ENA RB2  // PWM para controlar la velocidad del motor derecho

// Segundo L298N (Motores lado izquierdo)
#define IN3 RB3  // Controla la dirección del motor izquierdo 1
#define IN4 RB4  // Controla la dirección del motor izquierdo 1
#define ENB RB5  // PWM para controlar la velocidad del motor izquierdo


// Initialize ports
void Init_Ports() {
    // Configuración de puertos
    TRISA1 = 1; // Eje X derecha como entrada
    TRISA2 = 1; // Eje X izquierda como entrada
    TRISA3 = 1; // Eje Y arriba como entrada
    TRISA4 = 1; // Eje Y abajo como entrada
    TRISB0 = 0; // IN1 como salida
    TRISB1 = 0; // IN2 como salida
    TRISB2 = 0; // ENA como salida (PWM)
    TRISB3 = 0; // IN3 como salida
    TRISB4 = 0; // IN4 como salida
    TRISB5 = 0; // ENB como salida (PWM)
}

void Lcd_CmdWrite(unsigned char c) {
    PORTD = c;           // Place ASCII character on LCD data bus
    RS = 0;             // For sending command, RS = 0
    RW = 0;             // RW is always grounded
    EN = 1;             // Send the data now
    __delay_ms(Delay_LCD); // Wait for line to stabilize
    EN = 0;             // Ready, all sent 
}

void Lcd_DataWrite(unsigned char d) {
    PORTD = d;           // Place ASCII character on LCD data bus
    RS = 1;             // For sending data, RS = 1
    RW = 0;             // RW is always grounded
    EN = 1;             // Send the data now
    __delay_ms(Delay_LCD); // Wait for line to stabilize
    EN = 0;             // Ready, all sent 
}

void Message_LCD(unsigned char *s) {
    while(*s) {
        Lcd_DataWrite((unsigned char) *s++);
    }
}

// Initialize LCD to position cursor and display correctly
void Init_LCD() {
    Lcd_CmdWrite(TwoLines57Mat);
    Lcd_CmdWrite(DispONCurOFF);
    Lcd_CmdWrite(ClrScreen);
    Lcd_CmdWrite(FirstLine);
}

void Shift_Characters() {
    for(int i = 0; i < 25; i++) {
        __delay_ms(Delay_Shift);
        Lcd_CmdWrite(ShiftDispLeft);
    }
}

int Position_Lcd_Cursor(int LineNum, int Offset) {
    int PosVal;
    if(LineNum == 1) {
        PosVal = FirstLine + Offset;
    } else {
        PosVal = SecondLine + Offset;
    }
    return PosVal;
}

void Init_Var() {
    PORTA = 1;
    PORTD = 0;
    PORTB =0;
    PORTC =0;
}

void Config_ADC() {
    //ADCON1 = 0x0E;  // VCFG1 = 0 (Vref- = Vss), VCFG0 = 0 (Vref+ = Vdd)
    //ADCON2 = 0xA9; 
    ADFM = 1;  // Result justification to the right
    PCFG3 = 1; // Configuring channels as analog or digital
    PCFG2 = 1;
    PCFG1 = 0;
    PCFG0 = 0;
    ADCS2 = 1; // ADC conversion clock
    ADCS1 = 0;
    ADCS0 = 0;
    ADON = 1;  // Enable ADC
    __delay_ms(100);
}

void main(void) {
    Init_Ports();
    Init_LCD();
    Config_ADC();
    Init_Var();

      // Bucle principal
    while (1) {
        // Leer el estado del joystick eje X (derecha)
        if (X_axis_1 == 1) {
            // Girar a la derecha
            IN1 = 1; IN2 = 0;  // Motor derecho hacia adelante
            IN3 = 0; IN4 = 1;  // Motor izquierdo hacia atrás
            ENA = 1;           // Habilitar motor derecho (puede ser PWM para velocidad)
            ENB = 1;           // Habilitar motor izquierdo (puede ser PWM para velocidad)

            Lcd_CmdWrite(ClrScreen);
            Message_LCD((unsigned char *)"Go right");
            __delay_ms(1000);
            Lcd_CmdWrite(ClrScreen);
        }
        
        // Leer el estado del joystick eje X (izquierda)
        else if (X_axis_0 == 1) {
            // Girar a la izquierda
            IN1 = 0; IN2 = 1;  // Motor derecho hacia atrás
            IN3 = 1; IN4 = 0;  // Motor izquierdo hacia adelante
            ENA = 1;           // Habilitar motor derecho
            ENB = 1;           // Habilitar motor izquierdo

            Lcd_CmdWrite(ClrScreen);
            Message_LCD((unsigned char *)"Go left");
            __delay_ms(1000);
            Lcd_CmdWrite(ClrScreen);
        }

        // Leer el estado del joystick eje Y (arriba)
        else if (Y_axis_1 == 1) {
            // Avanzar hacia adelante
            IN1 = 1; IN2 = 0;  // Motor derecho hacia adelante
            IN3 = 1; IN4 = 0;  // Motor izquierdo hacia adelante
            ENA = 1;           // Habilitar motor derecho
            ENB = 1;           // Habilitar motor izquierdo

            Lcd_CmdWrite(ClrScreen);
            Message_LCD((unsigned char *)"Go up");
            __delay_ms(1000);
            Lcd_CmdWrite(ClrScreen);
        }

        // Leer el estado del joystick eje Y (abajo)
        else if (Y_axis_0 == 1) {
            // Retroceder
            IN1 = 0; IN2 = 1;  // Motor derecho hacia atrás
            IN3 = 0; IN4 = 1;  // Motor izquierdo hacia atrás
            ENA = 1;           // Habilitar motor derecho
            ENB = 1;           // Habilitar motor izquierdo

            Lcd_CmdWrite(ClrScreen);
            Message_LCD((unsigned char *)"Go down");
            __delay_ms(1000);
            Lcd_CmdWrite(ClrScreen);
        }

        // Detener los motores si no hay movimiento
        else {
            ENA = 0; // Deshabilitar motor derecho
            ENB = 0; // Deshabilitar motor izquierdo

            Lcd_CmdWrite(ClrScreen);
            Message_LCD((unsigned char *)"Stop");
            __delay_ms(1000);
            Lcd_CmdWrite(ClrScreen);
        }
    }
}


