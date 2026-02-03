/*
 * File:   main.c
 * Author: Antigravity
 * Target: PIC18F26K40
 * Board: RS232_MultiDrop
 *
 * Description:
 *   - UART1 (To Scaler):
 *     - TX1: RC6 (Pin 17) -> RS232_UC_TO_SCALER (UC to Scaler TX Pin)
 *     - RX1: RC7 (Pin 18) -> RS232_SCALER_TO_UC (Scaler to UC RX Pin)
 *   - UART2 (To MAX3323):
 *     - RX2: RC1 (Pin 12) -> RS232_MAX3322_TO_UC (UC RX Pin)
 *     - TX2: RC0 (Pin 11) -> RS232_UC_TO_MAX3322 (UC TX Pin)
 *   - Inputs (ID SW): RB0(ID0), RB1(ID1), RB2(ID2), RB3(ID3)
 *   - Control Outputs (MAX3323):
 *     - SHDN: RC2 (High = Operation / Always On)
 *     - RX_EN: RC3 (High = Enable / Always Receive)
 *     - TX_EN: RC4 (High = Sending, Low = Idle)
 *   - LEDs:
 *     - LED_IND0: RB4
 *     - LED_IND1: RB5
 */

// Configuration Bits
#pragma config FEXTOSC =                                                       \
    OFF // External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC =                                                        \
    HFINTOSC_64MHZ // Power-up default value for COSC bits (HFINTOSC with HFFRQ
                   // = 64 MHz and CDIV = 1:1)
#pragma config CLKOUTEN =                                                      \
    OFF // Clock Out Enable bit (CLKOUT function is disabled)
#pragma config CSWEN =                                                         \
    ON // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN =                                                         \
    ON // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config MCLRE = EXTMCLR // Master Clear Enable bit (MCLR pin is MCLR)
#pragma config PWRTE =                                                         \
    OFF // Power-up Timer Enable bit (Power up timer disabled)
#pragma config LPBOREN =                                                       \
    OFF // Low-power BOR enable bit (Low power BOR is disabled)
#pragma config BOREN = SBORDIS // Brown-out Reset Enable bits (Brown-out Reset
                               // enabled , SBOREN bit is ignored)
#pragma config BORV =                                                          \
    VBOR_245 // Brown Out Reset Voltage selection bits (Brown-out Reset Voltage
             // (VBOR) set to 2.45V)
#pragma config ZCD = OFF    // ZCD Disable bit (ZCD module is disabled)
#pragma config PPS1WAY = ON // PPSLOCK bit One-Way Set Enable bit
#pragma config STVREN = ON  // Stack Full/Underflow Reset Enable bit
#pragma config DEBUG = OFF  // Debugger Enable bit
#pragma config XINST = OFF  // Extended Instruction Set Enable bit (Disabled)
#pragma config WDTE = OFF   // WDT Operating Mode (WDT Disabled)

#include <stdbool.h>
#include <stdint.h>
#include <xc.h>

#define _XTAL_FREQ 64000000

// --- Pin Definitions ---
// Inputs (DIP Switch ID)
#define ID0_PORT PORTBbits.RB0
#define ID1_PORT PORTBbits.RB1
#define ID2_PORT PORTBbits.RB2
#define ID3_PORT PORTBbits.RB3

// Outputs (MAX3323 Control)
// MAX3323: SHDN (High=Shutdown), REN (Low=Enable), TE (Low=Enable)
#define MAX3323_SHDN_LAT LATCbits.LATC2
#define MAX3323_RX_EN_LAT LATCbits.LATC3
#define MAX3323_TX_EN_LAT LATCbits.LATC4

// Outputs (LEDs)
#define LED_IND0_LAT LATBbits.LATB4
#define LED_IND1_LAT LATBbits.LATB5

// --- Globals ---
uint8_t MY_DEVICE_ID = 0;

// Ring Buffers for Interrupt RX
#define RING_BUF_SIZE 64

volatile uint8_t uart1_rx_buf[RING_BUF_SIZE];
volatile uint8_t uart1_rx_head = 0;
volatile uint8_t uart1_rx_tail = 0;

volatile uint8_t uart2_rx_buf[RING_BUF_SIZE];
volatile uint8_t uart2_rx_head = 0;
volatile uint8_t uart2_rx_tail = 0;

// --- Function Prototypes ---
void SYSTEM_Initialize(void);
void OSCILLATOR_Initialize(void);
void PMD_Initialize(void);
void PIN_MANAGER_Initialize(void);
void UART1_Initialize(void);
void UART2_Initialize(void);
uint8_t UART1_Read(void);
void UART1_Write(uint8_t data);
uint8_t UART2_Read(void);
void UART2_Write(uint8_t data);
bool UART1_is_rx_ready(void);
bool UART2_is_rx_ready(void);
uint8_t Read_Device_ID(void);

// --- Interrupt Service Routine ---
void __interrupt() INTERRUPT_InterruptManager(void) {
  // UART1 RX Interrupt
  if (PIR3bits.RC1IF == 1 && PIE3bits.RC1IE == 1) {
    // Read clear Interrupt Flag
    if (RC1STAbits.OERR) {
      RC1STAbits.CREN = 0;
      RC1STAbits.CREN = 1;
    }

    uint8_t data = RC1REG;

    // Store in Ring Buffer
    uint8_t next_head = (uart1_rx_head + 1) % RING_BUF_SIZE;
    if (next_head != uart1_rx_tail) { // Not full
      uart1_rx_buf[uart1_rx_head] = data;
      uart1_rx_head = next_head;
    }
    // Else: Drop data (Buffer Overflow)
  }

  // UART2 RX Interrupt
  if (PIR3bits.RC2IF == 1 && PIE3bits.RC2IE == 1) {
    if (RC2STAbits.OERR) {
      RC2STAbits.CREN = 0;
      RC2STAbits.CREN = 1;
    }

    uint8_t data = RC2REG;

    uint8_t next_head = (uart2_rx_head + 1) % RING_BUF_SIZE;
    if (next_head != uart2_rx_tail) {
      uart2_rx_buf[uart2_rx_head] = data;
      uart2_rx_head = next_head;
    }
  }
}

// --- Main Application ---
void main(void) {
  SYSTEM_Initialize();

  // Enable Global Interrupts & Peripheral Interrupts
  INTCONbits.GIE = 1;
  INTCONbits.PEIE = 1;

  // Note: K40 Family uses INTCON for GIE/PEIE.
  // GIEL/GIEH are usually for Priority Mode (IPEN=1).
  // Default is usually IPEN=0 (Legacy Mode), so GIE/PEIE are correct.

  // Stabilization Delay: Wait for power/switches to settle
  __delay_ms(100);

  // Read ID from DIP Switches
  MY_DEVICE_ID = Read_Device_ID();

  // Initial State for MAX3323
  MAX3323_SHDN_LAT = 1;  // High = Operation
  MAX3323_RX_EN_LAT = 1; // High = Receive Enable
  // DEBUG: Enable TX ALWAYS to rule out timing issues
  MAX3323_TX_EN_LAT = 1; // High = Sending (Always ON)

  // LED Test (Both ON)
  LED_IND0_LAT = 1;
  LED_IND1_LAT = 1;

  // Packet Parsing State for UART2 (Bus)
  typedef enum {
    STATE_WAIT_START_ID,
    STATE_RECEIVE_PAYLOAD,
    STATE_WAIT_END_ID
  } rx_state_t;

  rx_state_t rx_state = STATE_WAIT_START_ID;
#define PAYLOAD_BUF_SIZE 64
  uint8_t payload_buffer[PAYLOAD_BUF_SIZE];
  uint8_t payload_index = 0;

  while (1) {
    // --- 1. UART2 Processing (Bus -> Scaler) ---
    // [ID] [POWERX] Test Logic
    static uint8_t keyword_idx = 0;
    const char *keyword = "POWERX";
    static bool id_matched = false;

    if (UART2_is_rx_ready()) {
      uint8_t rx_byte = UART2_Read();

      if (!id_matched) {
        // Step 1: Wait for ID Match
        if (rx_byte == MY_DEVICE_ID) {
          id_matched = true;
          keyword_idx = 0; // Reset keyword check
        }
      } else {
        // Step 2: Check for "POWERX"
        if (rx_byte == keyword[keyword_idx]) {
          keyword_idx++;

          if (keyword[keyword_idx] == '\0') {
            // "POWERX" Match Found!
            id_matched = false; // Reset for next packet

            // Action 1: Blink LEDs 3 times
            for (int i = 0; i < 3; i++) {
              LED_IND0_LAT = 0;
              LED_IND1_LAT = 0;
              __delay_ms(200);
              LED_IND0_LAT = 1;
              LED_IND1_LAT = 1;
              __delay_ms(200);
            }

            // Action 2: Send "POWERX" back to TX2
            // TX Driver is already ON (Debug)
            
            const char *msg = "POWERX";
            while (*msg) {
               UART2_Write(*msg++);
            }

            // Wait for transmission to finish
            while (!TX2STAbits.TRMT);
          }
        } else {
          // Mismatch in keyword -> Reset
          id_matched = false;
          keyword_idx = 0;
          // Check if this byte was actually the ID (restart immediately)
          if (rx_byte == MY_DEVICE_ID) {
              id_matched = true;
          }
        }
      }
    }

    // --- 2. UART1 Processing (To be ignored for this test or keep as is) ---
    // Keeping simple pass-through just in case, or can be removed.
    // For this specific test request, we focus on UART2 loop.
    if (UART1_is_rx_ready()) {
        UART1_Read(); // Just clear buffer to prevent overflow
    }
  }
}

void SYSTEM_Initialize(void) {
  OSCILLATOR_Initialize();
  PMD_Initialize();
  PIN_MANAGER_Initialize();
  UART1_Initialize();
  UART2_Initialize();
}

void OSCILLATOR_Initialize(void) {
  OSCCON1 = 0x60;
  OSCFRQ = 0x08;
}

void PMD_Initialize(void) {}

void PIN_MANAGER_Initialize(void) {
  LATA = 0x00;
  LATB = 0x00;
  LATC = 0x00;

  TRISA = 0xFF;
  ANSELA = 0x00;

  TRISB = 0xCF;
  ANSELB = 0x00;

  // RC0(TX2), RC1(RX2), RC2(SHDN), RC3(RXEN), RC4(TXEN), RC5, RC6(TX1),
  // RC7(RX1)
  TRISC = 0x82;
  ANSELC = 0x00;

  PPSLOCK = 0x55;
  PPSLOCK = 0xAA;
  PPSLOCKbits.PPSLOCKED = 0;

  RC6PPS = 0x0F;
  RX1PPS = 0x17;

  RC0PPS = 0x11;
  RX2PPS = 0x11;

  PPSLOCK = 0x55;
  PPSLOCK = 0xAA;
  PPSLOCKbits.PPSLOCKED = 1;
}

void UART1_Initialize(void) {
  BAUD1CON = 0x08;
  TX1STA = 0x24;
  RC1STA = 0x90;
  SP1BRGL = 0x82;
  SP1BRGH = 0x06;

  // Enable Interrupts
  PIE3bits.RC1IE = 1;
}

void UART2_Initialize(void) {
  BAUD2CON = 0x08;
  TX2STA = 0x24;
  RC2STA = 0x90;
  SP2BRGL = 0x82;
  SP2BRGH = 0x06;

  // Enable Interrupts
  PIE3bits.RC2IE = 1;
}

// Updated Buffer Read Functions
uint8_t UART1_Read(void) {
  // Read from Ring Buffer
  if (uart1_rx_head == uart1_rx_tail)
    return 0; // Empty

  uint8_t data = uart1_rx_buf[uart1_rx_tail];
  uart1_rx_tail = (uart1_rx_tail + 1) % RING_BUF_SIZE;
  return data;
}

void UART1_Write(uint8_t data) {
  while (!PIR3bits.TX1IF)
    ;
  TX1REG = data;
}

uint8_t UART2_Read(void) {
  if (uart2_rx_head == uart2_rx_tail)
    return 0;

  uint8_t data = uart2_rx_buf[uart2_rx_tail];
  uart2_rx_tail = (uart2_rx_tail + 1) % RING_BUF_SIZE;
  return data;
}

void UART2_Write(uint8_t data) {
  while (!PIR3bits.TX2IF)
    ;
  TX2REG = data;
}

bool UART1_is_rx_ready(void) { return (uart1_rx_head != uart1_rx_tail); }

bool UART2_is_rx_ready(void) { return (uart2_rx_head != uart2_rx_tail); }

uint8_t Read_Device_ID(void) {
  uint8_t id = 0;
  if (!ID0_PORT)
    id |= 0x01;
  if (!ID1_PORT)
    id |= 0x02;
  if (!ID2_PORT)
    id |= 0x04;
  if (!ID3_PORT)
    id |= 0x08;
  return id;
}
