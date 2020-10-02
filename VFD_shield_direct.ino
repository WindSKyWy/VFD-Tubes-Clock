
/*
  VFD Arduino Shield direct access to tubes and LEDs.

  This program listens to incoming commands on the serial port (9600 baud). A command
  is processed when a CR (13) or LF (10) character arrives.

  - Command 'W': Write four bytes to display state memory area. Bit 0..7 of each byte
    correspond with pin 2..9. The next character is 'B' for binary, 'H' for hexadecimal,
    'D' for decimal. The following four bytes must be formatted in the indicated base.
    For example:

    WD 1 12 255 0

    WH FF 0 AA A8

    WB 1001001 00110110 1 10000000

  - Command 'D': Set dimming value for the LEDs. Valid values are 0..8. O means LEDs are
    off, 8 means LEDs are fully on. For example:

    D 2

  Notes:
  * Pins 2..9 are connected to the segments and dots of the tubes.
  * Pins A0..A3 are connected to the grids of the tubes.
  * Pin 10 is connected to the LEDs beneath the tubes.
  * Macros HIGH and LOW are always passed as arguments of type byte (uint8_t).

  Tested with:
  * Arduino 1.8.12

  Author:  Kiril Kirov, Instructables
  Date:    2020-04-18
  License: public domain

       pin 2
        ---
 pin 7 |   | pin 3
       |   |
 pin 8  ---
       |   | pin 4
 pin 6 |   |
        --- . pin 9
       pin 5
*/

// Tubes state (read-write)
//
// Must be accessed at interrupt level

static  byte  tubes_state[4][8] =
{
/* pin  2     3     4     5     6     7     8     9  */
    {  LOW,  LOW,  LOW,  LOW,  LOW,  LOW,  LOW,  LOW }, /* Tube 0 */
    {  LOW,  LOW,  LOW,  LOW,  LOW,  LOW,  LOW,  LOW }, /* Tube 1 */
    {  LOW,  LOW,  LOW,  LOW,  LOW,  LOW,  LOW,  LOW }, /* Tube 2 */
    {  LOW,  LOW,  LOW,  LOW,  LOW,  LOW,  LOW,  LOW }  /* Tube 3 */
};


// Variables accessed at interrupt level

static  byte    cur_tube      = 3;    // 0..3
static  byte    tube_toggle   = 0;
static  byte    led_pwm_step  = 0;    // 0..7
static  byte    led_pwm_off   = 2;    // 0..7, 8 means always on


void  isr_tubes ()
{
  byte   *seg_p;
  byte    pin;


  // Drive the tubes

  // Disable all grids and segments, then wait a bit. Doing so prevents the so-called ghosting
  // visual effect.

  // Clear pins as fast as possible
  PORTC &= ~0b00001111;  // Clear pin A[0..3]
  PORTD &= ~0b11111100;  // Clear pin 2..7
  PORTB &= ~0b00000011;  // Clear pin 8..9

  // The following function is compiled to one or more nested loops that run for the exact amount
  // of cycles
  __builtin_avr_delay_cycles(16*40);  // 40 us (at 16 MHz)

  // Select the next tube
  cur_tube++;
  cur_tube %= 4;

  seg_p = &tubes_state[cur_tube][0];
  for (pin = 2; pin < 10; pin++, seg_p++) digitalWrite(pin,*seg_p);

  // Enable the current tube
  digitalWrite(A0+cur_tube,HIGH);
}


void  isr_leds ()
{
  // Program LED PWM duty cycle
  if (led_pwm_step == led_pwm_off) digitalWrite(10,LOW);
  else
    if (led_pwm_step == 0) digitalWrite(10,HIGH);

  // PWM step counter: 0->1->2->...->7->0->1->...
  led_pwm_step++;
  if (led_pwm_step == 8) led_pwm_step = 0;
}


// Timer 1 interrupt service routine

ISR(TIMER1_COMPA_vect)
{
  // Drive tubes at 250 Hz
  tube_toggle ^= 1;
  if (tube_toggle) isr_tubes();

  // PWM for LEDs requires 500 Hz
  isr_leds();
}


void  setup ()
{
  Serial.begin(9600);

  pinMode(A0,OUTPUT);
  pinMode(A1,OUTPUT);
  pinMode(A2,OUTPUT);
  pinMode(A3,OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);

  // TCCR1A - Timer/Counter Control Register A: (default 00000000b)
  // * Bit 7..6: COM1A[1..0]: Compare Match Output A Mode
  // * Bit 5..4: COM1B[1..0]: Compare Match Output B Mode
  // * Bit 3..2: Reserved
  // * Bit 1..0: WGM1[1..0]: Waveform Generation Mode
  //
  // TCCR1B - Timer/Counter Control Register B: (default 00000000b)
  // * Bit    7: ICNC1: Input Capture Noise Canceler
  // * Bit    6: ICES1: Input Capture Edge Select
  // * Bit    5: Reserved
  // * Bit 4..3: WGM1[3..2]: Waveform Generation Mode
  // * Bit 2..0: CS1[2..0]: Clock Select
  //
  // TCCR1C - Timer/Counter Control Register C: (default 00000000b)
  // * Bit    7: FOC1A: Force Output Compare for Channel A
  // * Bit    6: FOC1B: Force Output Compare for Channel B
  // * Bit 5..0: Reserved
  //
  // TCNT1H and TCNT1L - Timer/Counter Register: (default 0000000000000000b)
  // * Bit 15..0: TCNT1[15..0]: timer/counter value
  //
  // OCR1AH and OCR1AL - Output Compare Register A: (default 0000000000000000b)
  // * Bit 15..0: OCR1A[15..0]: compare value A
  //
  // OCR1BH and OCR1BL - Output Compare Register B: (default 0000000000000000b)
  // * Bit 15..0: OCR1B[15..0]: compare value B
  //
  // ICR1H and ICR1L - Input Capture Register: (default 0000000000000000b)
  // * Bit 15..0: ICR1[15..0]: capture value B
  //
  // TIMSK1 - Timer/Counter Interrupt Mask Register: (default 00000000b)
  // * Bit 7..6: Reserved
  // * Bit    5: ICIE1: Input Capture Interrupt Enable
  // * Bit 4..3: Reserved
  // * Bit    2: OCIE1B: Output Compare B Match Interrupt Enable
  // * Bit    1: OCIE1A: Output Compare A Match Interrupt Enable
  // * Bit    0: TOIE1: Overflow Interrupt Enable
  //
  // TIFR1 - Timer/Counter Interrupt Flag Register: (default 00000000b)
  // * Bit 7..6: Reserved
  // * Bit    5: ICF1: Input Capture Flag
  // * Bit 4..3: Reserved
  // * Bit    2: OCF1B: Output Compare B Match Flag
  // * Bit    1: OCF1A: Output Compare A Match Flag
  // * Bit    0: TOV1: Overflow Flag
  //
  // Settings:
  // - WGM1[3..0]=0100b: CTC (top value OCR1A, overflow on MAX)
  //   Non-PWM mode:
  //   - COM1A[1..0]=00b: Normal port operation, OC1A disconnected
  //   - COM1B[1..0]=00b: Normal port operation, OC1B disconnected
  // - CS1[2..0]=100b: clk/256 speed => 62500 Hz
  //                   -> a non-zero value also enables the timer/counter
  // - OCIE1A=1: Generate interrupt on output compare A match

  cli();

  // We've observed on Arduino IDE 1.5.8 that TCCR1A is non-zero at this point. So let's play safe
  // and write all relevant timer registers.
  TCCR1A = 0b00000000;
  OCR1A  = 125-1;       // 500 Hz (62500/125)
  TCNT1  = 0;
  TIMSK1 = 0b00000010;
  TIFR1  = 0b00000000;
  TCCR1B = 0b00001100;  // Enable timer

  sei();
}


#define RCV_BUF_LEN   32

static  char  rcv_buf[RCV_BUF_LEN];  // Receive buffer
static  byte  rcv_si    = 0;         // Store index
static  byte  rcv_error = false;     // Receive error condition
static  byte  rcv_fi    = 0;         // Fetch index
static  char  rcv_fetch = 0;         // Last fetched character


void  keep_rcv_char ()
{
  if (rcv_fetch != 0) rcv_fi--;
}


char  fetch_rcv_char ()
{
  if (rcv_fi == rcv_si)
  {
    // Reached the end of the buffer
    rcv_fetch = 0;
  }
  else
  {
    // Fetch the next character
    rcv_fetch = rcv_buf[rcv_fi++];
  }

  return rcv_fetch;
}


static  word  parse_u8_res;


boolean  parse_bin_u8 ()
{
  byte    prev;
  char    c;
  boolean parsed;

  parsed       = false;
  parse_u8_res = 0;

  for (;;)
  {
    c = fetch_rcv_char();
    if ((c >= '0') && (c <= '1'))
    {
      prev = parse_u8_res;

      parse_u8_res = parse_u8_res * 2 + (c - '0');

      // Check for overflow
      if (parse_u8_res < prev) return false;

      // At least one digit has been parsed
      parsed = true;
    }
    else
    {
      keep_rcv_char();

      return parsed;
    }
  }
}


boolean  parse_hex_u8 ()
{
  byte    prev;
  byte    digit;
  char    c;
  boolean parsed;

  parsed       = false;
  parse_u8_res = 0;

  for (;;)
  {
    c = fetch_rcv_char();
    if ((c >= '0') && (c <= '9')) digit = c - '0'; else
    if ((c >= 'A') && (c <= 'F')) digit = c - 'A' + 10; else
    {
      keep_rcv_char();

      return parsed;
    }

    prev = parse_u8_res;

    parse_u8_res = parse_u8_res * 16 + digit;

    // Check for overflow
    if (parse_u8_res < prev) return false;

    // At least one digit has been parsed
    parsed = true;
  }
}


boolean  parse_dec_u8 ()
{
  byte    prev;
  char    c;
  boolean parsed;

  parsed       = false;
  parse_u8_res = 0;

  for (;;)
  {
    c = fetch_rcv_char();
    if ((c >= '0') && (c <= '9'))
    {
      prev = parse_u8_res;

      parse_u8_res = parse_u8_res * 10 + (c - '0');

      // Check for overflow
      if (parse_u8_res < prev) return false;

      // At least one digit has been parsed
      parsed = true;
    }
    else
    {
      keep_rcv_char();

      return parsed;
    }
  }
}


void  parse_spaces ()
{
  char  c;

  for (;;)
  {
    c = fetch_rcv_char();
    if (c != ' ') break;
  }

  keep_rcv_char();
}


boolean  process_rcv ()
{
  static  byte  rcvd_bytes[4];

  char    c;
  byte    n;
  byte    b;
  byte    mask;
  byte   *seg_p;

  c = fetch_rcv_char();
  if (c == 0)
  {
    // Empty line received
    return true;
  }
  else
  if (c == 'W')
  {
    c = fetch_rcv_char();
    if (c == 'B')
    {
      for (n = 0; n < 4; n++)
      {
        // Parse separator space characters
        parse_spaces();

        if (!parse_bin_u8()) return false;
        rcvd_bytes[n] = parse_u8_res;
      }
    }
    else
    if (c == 'H')
    {
      for (n = 0; n < 4; n++)
      {
        // Parse separator space characters
        parse_spaces();

        if (!parse_hex_u8()) return false;
        rcvd_bytes[n] = parse_u8_res;
      }
    }
    else
    if (c == 'D')
    {
      for (n = 0; n < 4; n++)
      {
        // Parse separator space characters
        parse_spaces();

        if (!parse_dec_u8()) return false;
        rcvd_bytes[n] = parse_u8_res;
      }
    }
    else
        return false;

    // End-of-line expected
    parse_spaces();
    c = fetch_rcv_char();
    if (c != 0) return false;

    cli();

    seg_p = &tubes_state[0][0];
    n = 0;
    for (;;)
    {
      b = 0;
      mask = rcvd_bytes[n];
      for (;;)
      {
        *seg_p = (mask & 1) ? HIGH : LOW;
        seg_p++;

        if (b == 7) break;
        b++;
        mask >>= 1;
      }

      if (n == 3) break;
      n++;
    }

    sei();

    return true;
  }
  else
  if (c == 'D')
  {
    byte    pwm_off;

    // Parse separator space characters
    parse_spaces();

    // Parse the PWM duty (0..8)
    if (!parse_dec_u8()) return false;
    if (parse_u8_res > 8) return false;
    pwm_off = parse_u8_res;

    // End-of-line expected
    parse_spaces();
    c = fetch_rcv_char();
    if (c != 0) return false;

    cli();
    led_pwm_off = pwm_off;
    sei();

    return true;
  }
  else
  {
    // Unknown command
    return false;
  }
}


// Receive characters from the serial port in non-blocking fashion.

void  rcv ()
{
  char  c;

  while (Serial.available() > 0)
  {
    c = Serial.read();
    if ((c == 13) || (c == 10))
    {
      if (rcv_error == false) process_rcv();

      // Reset the receive buffer
      rcv_si    = 0;
      rcv_error = false;
      rcv_fi    = 0;
    }
    else
    if ((c < 32) || (c > 126))
    {
      // Invalid character received
      rcv_error = true;
    }
    else
    if (rcv_si == RCV_BUF_LEN)
    {
      // Buffer overflow
      rcv_error = true;
    }
    else
    {
      // Store character in receive buffer
      rcv_buf[rcv_si] = c;
      rcv_si++;
    }
  }
}


void  loop ()
{
  rcv();
}
