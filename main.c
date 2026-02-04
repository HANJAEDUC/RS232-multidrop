
// CPU: PIC18F26K40
// Clock: 64MHz Internal
// Target: RS232 Smart Bridge (ID <-> Protocol)
// HW:
//  - PC Link (UART2): TX=RC0(11), RX=RC1(12), EN=RC3/4
//  - Scaler Link (UART1): TX=RC6(17), RX=RC7(18)
//  - IDs: DIP SW (RB0-3)
//  - LEDs: LED0(RB4)=RX Event, LED1(RB5)=TX Event

#pragma config FEXTOSC = OFF
#pragma config RSTOSC = HFINTOSC_64MHZ
#pragma config CLKOUTEN = OFF
#pragma config CSWEN = ON
#pragma config FCMEN = ON
#pragma config MCLRE = EXTMCLR
#pragma config PWRTE = OFF
#pragma config LPBOREN = OFF
#pragma config BOREN = SBORDIS
#pragma config BORV = VBOR_245
#pragma config ZCD = OFF
#pragma config PPS1WAY = OFF 
#pragma config STVREN = ON
#pragma config DEBUG = OFF
#pragma config LVP = OFF     

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#define _XTAL_FREQ 64000000

// --- Hardware Mapping ---
#define ID0_PORT PORTBbits.RB0
#define ID1_PORT PORTBbits.RB1
#define ID2_PORT PORTBbits.RB2
#define ID3_PORT PORTBbits.RB3

// MAX3323 Control
#define MAX3323_SHDN_LAT LATCbits.LATC2
#define MAX3323_RX_EN_LAT LATCbits.LATC3 
#define MAX3323_TX_EN_LAT LATCbits.LATC4 

// LEDs
#define LED_IND0_LAT LATBbits.LATB4 
#define LED_IND1_LAT LATBbits.LATB5

// --- Globals ---
uint8_t MY_DEVICE_ID = 0;

// UART Buffers (Ring)
#define RING_BUF_SIZE 128
volatile uint8_t u2_rx_buf[RING_BUF_SIZE];
volatile uint8_t u2_rx_head=0, u2_rx_tail=0;

volatile uint8_t u1_rx_buf[RING_BUF_SIZE];
volatile uint8_t u1_rx_head=0, u1_rx_tail=0;

// Payload Assembly Buffers
#define PAYLOAD_MAX 64
uint8_t u2_payload[PAYLOAD_MAX]; // PC -> Scaler
uint8_t u2_len = 0;

uint8_t u1_payload[PAYLOAD_MAX]; // Scaler -> PC
uint8_t u1_len = 0;

// Timers (Software counters, 1ms tick)
uint16_t led0_timer = 0; // 0=Off, >0=On
uint16_t led1_timer = 0;
uint16_t u1_timeout = 0; // For detecting end of Scaler Packet

// State Machine for UART2
typedef enum {
    S2_WAIT_ID,
    S2_RX_DATA
} u2_state_t;
u2_state_t u2_state = S2_WAIT_ID;

// Prototypes
void SYSTEM_Initialize(void);
void Init_App(void);
void Process_UART2_RX(void);
void Process_UART1_RX(void);
void Send_To_UART1(uint8_t *data, uint8_t len);
void Send_To_UART2(uint8_t *data, uint8_t len);

void main(void) {
    SYSTEM_Initialize();
    Init_App(); // Enable Ints, Read ID

    while (1) {
        // 1. Timebase (Approx 1ms)
        // Blocking 1ms is okay for 9600bps (1 byte ~1ms)
        // Ring buffers catch data during delay.
        __delay_ms(1);

        // 2. LED Management
        if (led0_timer > 0) {
            LED_IND0_LAT = 1;
            led0_timer--;
        } else {
            LED_IND0_LAT = 0;
        }

        if (led1_timer > 0) {
            LED_IND1_LAT = 1;
            led1_timer--;
        } else {
            LED_IND1_LAT = 0;
        }
        
        // 3. UART1 RX Timeout Management
        // If we have data but no new data for 30ms, assume packet end
        if (u1_len > 0) {
            u1_timeout++;
            if (u1_timeout > 30) { 
                // --- Forward SCALER -> PC ---
                // Format: [ID][Data]
                
                // 1. Send ID
                Send_To_UART2(&MY_DEVICE_ID, 1);
                
                // 2. Send Data
                Send_To_UART2(u1_payload, u1_len);
                
                // 3. LED1 Trigger (2s)
                led1_timer = 2000;
                
                // Reset
                u1_len = 0;
                u1_timeout = 0;
            }
        }

        // 4. Process Inputs
        Process_UART2_RX();
        Process_UART1_RX();
    }
}

// --- Processor Logic ---

void Process_UART2_RX(void) {
    while (u2_rx_head != u2_rx_tail) {
        uint8_t b = u2_rx_buf[u2_rx_tail];
        u2_rx_tail = (u2_rx_tail + 1) % RING_BUF_SIZE;

        switch (u2_state) {
            case S2_WAIT_ID:
                if (b == MY_DEVICE_ID) {
                    u2_state = S2_RX_DATA;
                    u2_len = 0;
                }
                break;

            case S2_RX_DATA:
                if (u2_len < PAYLOAD_MAX) u2_payload[u2_len++] = b;
                
                if (b == 'X') {
                    // --- Forward PC -> SCALER ---
                    // Format: [Data...X] (ID Stripped)
                    
                    // 1. Visually Indicate Receive (LED0 2s)
                    led0_timer = 2000;
                    
                    // 2. Forward to UART1
                    Send_To_UART1(u2_payload, u2_len);
                    
                    // Reset
                    u2_state = S2_WAIT_ID;
                    u2_len = 0;
                }
                break;
        }
    }
}

void Process_UART1_RX(void) {
    while (u1_rx_head != u1_rx_tail) {
        uint8_t b = u1_rx_buf[u1_rx_tail];
        u1_rx_tail = (u1_rx_tail + 1) % RING_BUF_SIZE;

        // Just collect all data
        if (u1_len < PAYLOAD_MAX) {
            u1_payload[u1_len++] = b;
        }
        // Reset timeout because we got new data
        u1_timeout = 0; 
    }
}

void Send_To_UART1(uint8_t *data, uint8_t len) {
    for (int i=0; i<len; i++) {
        while(!PIR3bits.TX1IF); // Wait TX1
        TX1REG = data[i];
    }
}

void Send_To_UART2(uint8_t *data, uint8_t len) {
    // 1. Enable Driver
    MAX3323_TX_EN_LAT = 1;
    __delay_ms(15); // Wait Charge Pump
    
    // 2. Send
    for (int i=0; i<len; i++) {
        while(!PIR3bits.TX2IF); // Wait TX2
        TX2REG = data[i];
    }
    
    // 3. Wait Finish
    while(!TX2STAbits.TRMT);
    
    // 4. Disable Driver
    __delay_ms(2);
    MAX3323_TX_EN_LAT = 0;
}

// --- Init & Drivers ---

void Init_App(void) {
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    
    MAX3323_SHDN_LAT = 1;
    MAX3323_RX_EN_LAT = 1; // Always Enable RX
    MAX3323_TX_EN_LAT = 0;
    
    uint8_t id = 0;
    if (!ID0_PORT) id |= 1;
    if (!ID1_PORT) id |= 2;
    if (!ID2_PORT) id |= 4;
    if (!ID3_PORT) id |= 8;
    MY_DEVICE_ID = id;
}

void SYSTEM_Initialize(void) {
    // OSC
    OSCCON1 = 0x60; OSCFRQ = 0x08; // 64MHz
    
    // PIN
    LATA=0; LATB=0; LATC=0;
    TRISA=0xFF; 
    TRISB=0xCF; // RB4, RB5 Output
    TRISC=0x82; // RC1, RC7 Input
    
    ANSELA=0; ANSELB=0; ANSELC=0;
    
    // PPS
    PPSLOCK = 0x55; PPSLOCK = 0xAA; PPSLOCKbits.PPSLOCKED = 0;

    // UART2 (PC)
    RC0PPS = 0x0B; // TX2 -> RC0 (Pin 11)
    RX2PPS = 0x11; // RC1 -> RX2 Input (Pin 12)
    
    // UART1 (Scaler)
    RC6PPS = 0x09; // TX1 -> RC6 (Pin 17) - Check K40 Datasheet (TX1=0x09)
    RX1PPS = 0x17; // RC7 -> RX1 Input (Pin 18)

    PPSLOCK = 0x55; PPSLOCK = 0xAA; PPSLOCKbits.PPSLOCKED = 1;
    
    // UART Init
    // UART1: 9600
    BAUD1CON=0x08; TX1STA=0x24; RC1STA=0x90; 
    SP1BRGL=0x82; SP1BRGH=0x06;
    PIE3bits.RC1IE=1;

    // UART2: 9600
    BAUD2CON=0x08; TX2STA=0x24; RC2STA=0x90; 
    SP2BRGL=0x82; SP2BRGH=0x06;
    PIE3bits.RC2IE=1;
}

void __interrupt() ISR(void) {
    // UART2 RX (PC)
    if (PIR3bits.RC2IF && PIE3bits.RC2IE) {
        if(RC2STAbits.OERR) { RC2STAbits.CREN=0; RC2STAbits.CREN=1; }
        u2_rx_buf[u2_rx_head] = RC2REG;
        u2_rx_head = (u2_rx_head + 1) % RING_BUF_SIZE;
    }
    // UART1 RX (Scaler)
    if (PIR3bits.RC1IF && PIE3bits.RC1IE) {
        if(RC1STAbits.OERR) { RC1STAbits.CREN=0; RC1STAbits.CREN=1; }
        u1_rx_buf[u1_rx_head] = RC1REG;
        u1_rx_head = (u1_rx_head + 1) % RING_BUF_SIZE;
    }
}
