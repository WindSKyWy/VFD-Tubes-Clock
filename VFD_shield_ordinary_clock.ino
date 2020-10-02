
/*
  VFD Arduino Shield as a clock.

  This program listens to incoming commands on the serial port (9600 baud). A command
  is processed when a CR (13) or LF (10) character arrives.

  - Command 'T': Set the date and time by means of a Unix epoch value. The command
    must be followed by a decimal value. For example:

    T 1431472660

    Sets the date and time to 2020-10-01 23:17:40.

    See also: http://www.epochconverter.com/

  - Command 'S': Set the date and time by specifying year, month, day, hour, minute and
    second. The values must be separated by one or more spaces. For example:

    S 2015 07 21 16 35 48

    Sets the date and time to 2020-10-01 16:35:48.

  - Command 'D': Set dimming value for the LEDs. Valid values are 0..8. O means LEDs are
    off, 8 means LEDs are fully on. For example:

    D 2

  Required library: http://www.pjrc.com/teensy/td_libs_Time.html

  Notes:
  * Pins 2..9 are connected to the segments and dots of the tubes.
  * Pins A0..A3 are connected to the grids of the tubes.
  * Pin 10 is connected to the LEDs beneath the tubes.
  * Macros HIGH and LOW are always passed as arguments of type byte (uint8_t).

  Tested with:
  * Arduino 1.8.12, Time 1.6 and Time 1.5

  Author:  Kiril Kirov, Instructables
  Date:    2020-04-18
  License: public domain

*/


#include <Time.h>
#include <TimeLib.h>


/*
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

static  const  byte  digit_seg_data[11*7] PROGMEM =
{
/* pin    2     3     4     5     6     7     8  */
        HIGH, HIGH, HIGH, HIGH, HIGH, HIGH,  LOW,    // Digit 0
         LOW, HIGH, HIGH,  LOW,  LOW,  LOW,  LOW,    // Digit 1
        HIGH, HIGH,  LOW, HIGH, HIGH,  LOW, HIGH,    // Digit 2
        HIGH, HIGH, HIGH, HIGH,  LOW,  LOW, HIGH,    // Digit 3
         LOW, HIGH, HIGH,  LOW,  LOW, HIGH, HIGH,    // Digit 4
        HIGH,  LOW, HIGH, HIGH,  LOW, HIGH, HIGH,    // Digit 5
        HIGH,  LOW, HIGH, HIGH, HIGH, HIGH, HIGH,    // Digit 6
        HIGH, HIGH, HIGH,  LOW,  LOW,  LOW,  LOW,    // Digit 7
        HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH,    // Digit 8
        HIGH, HIGH, HIGH, HIGH,  LOW, HIGH, HIGH,    // Digit 9
         LOW,  LOW,  LOW,  LOW,  LOW,  LOW, HIGH     // Hyphen
};


typedef  struct  _TUBE
{
  byte    digit;      // 0..9
  byte    dot;        // HIGH or LOW
}
  TUBE;


// Display state of the tubes (read-write)

static  TUBE    tube_list[4] =
{
  { 10,  LOW },
  { 10,  LOW },
  { 10,  LOW },
  { 10,  LOW }
};


// Variables accessed at interrupt level

static  byte    cur_tube      = 3;    // 0..3
static  byte    tube_toggle   = 0;
static  byte    led_pwm_step  = 0;    // 0..7
static  byte    led_pwm_off   = 2;    // 0..7, 8 means always on


void  isr_tubes ()
{
  const  byte  *digit_seg_p;
  byte          digit;
  byte          pin;


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

  digit = tube_list[cur_tube].digit;
  digit_seg_p = digit_seg_data + 7*digit;
  for (pin = 2; pin < 9; pin++, digit_seg_p++) digitalWrite(pin,pgm_read_byte(digit_seg_p));
  digitalWrite(pin,tube_list[cur_tube].dot);

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


void  set_tubes_val (int val)
{
  byte    tube;

  // Loop: tube 3->0
  tube = 3;
  for (;;)
  {
    // Least significant decimal digit
    tube_list[tube].digit = val % 10;

    // Loop control
    if (tube == 0) break;
    tube--;

    // Shift decimal value one digit to the right
    val /= 10;
  }
}


#define RCV_BUF_LEN   32

static  char  rcv_buf[RCV_BUF_LEN];  // Receive buffer
static  byte  rcv_si    = 0;         // Store index
static  byte  rcv_error = false;     // Receive error condition
static  byte  rcv_fi    = 0;         // Fetch index


void  keep_rcv_char ()
{
  rcv_fi--;
}


char  fetch_rcv_char ()
{
  if (rcv_fi < rcv_si)
  {
    // Fetch the next character
    return rcv_buf[rcv_fi++];
  }
  else
  {
    // Reached the end of the buffer
    return 0;
  }
}


static  word  parse_u16_res;


boolean  parse_u16 ()
{
  word    prev;
  char    c;
  boolean parsed;

  parsed        = false;
  parse_u16_res = 0;

  for (;;)
  {
    c = fetch_rcv_char();
    if ((c >= '0') && (c <= '9'))
    {
      prev = parse_u16_res;

      parse_u16_res = parse_u16_res * 10 + (c - '0');

      // Check for overflow
      if (parse_u16_res < prev) return false;
  
      // At least one digit has been parsed
      parsed = true;
    }
    else
    {
      if (c != 0) keep_rcv_char();

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

  if (c != 0) keep_rcv_char();
}


boolean  process_rcv ()
{
  char  c;

  c = fetch_rcv_char();
  if (c == 0)
  {
    // Empty line received
    return true;
  }
  else
  if (c == 'T')
  {
    time_t  t;

    // Parse separator space characters
    parse_spaces();

    // Fetch the decimal value; at least one digit is expected
    t = 0;
    for (;;)
    {
      c = fetch_rcv_char();

      if ((c >= '0') && (c <= '9'))
      {
        t = (t * 10) + (c - '0');
      }
      else
      {
        if (c != 0) keep_rcv_char();
        break;
      }
   }

    // Parse separator space characters
    parse_spaces();

    // End-of-line expected
    c = fetch_rcv_char();
    if (c != 0) return false;

    setTime(t);

    return true;
  }
  else
  if (c == 'S')
  {
    static  tmElements_t  elem;

    word          w;
    time_t        t;

    // Parse separator space characters
    parse_spaces();

    // Parse the year (1970..2225)
    if (!parse_u16()) return false;
    w = parse_u16_res - 1970;
    if (w > 255) return false;
    elem.Year = (uint8_t)w;

    // Parse separator space characters
    parse_spaces();

    // Parse the month (1..12)
    if (!parse_u16()) return false;
    w = parse_u16_res;
    if ((w < 1) || (w > 12)) return false;
    elem.Month = (uint8_t)w;

    // Parse separator space characters
    parse_spaces();

    // Parse the day (1..31)
    if (!parse_u16()) return false;
    w = parse_u16_res;
    if ((w < 1) || (w > 31)) return false;
    elem.Day = (uint8_t)w;

    // Parse separator space characters
    parse_spaces();

    // Parse the hour (0..23)
    if (!parse_u16()) return false;
    w = parse_u16_res;
    if (w > 23) return false;
    elem.Hour = (uint8_t)w;

    // Parse separator space characters
    parse_spaces();

    // Parse the minute (0..59)
    if (!parse_u16()) return false;
    w = parse_u16_res;
    if (w > 59) return false;
    elem.Minute = (uint8_t)w;

    // Parse separator space characters
    parse_spaces();

    // Parse the second (0..59)
    if (!parse_u16()) return false;
    w = parse_u16_res;
    if (w > 59) return false;
    elem.Second = (uint8_t)w;

    // Parse separator space characters
    parse_spaces();

    // End-of-line expected
    c = fetch_rcv_char();
    if (c != 0) return false;

    t = makeTime(elem);
    setTime(t);

    return true;
  }
  else
  if (c == 'D')
  {
    byte    pwm_off;

    // Parse separator space characters
    parse_spaces();

    // Parse the PWM duty (0..8)
    if (!parse_u16()) return false;
    if (parse_u16_res > 8) return false;
    pwm_off = (uint8_t)parse_u16_res;

    // Parse separator space characters
    parse_spaces();

    // End-of-line expected
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


void  no_time ()
{
  byte  tube;

  cli();

  for (tube = 0; tube < 4; tube++)
  {
    tube_list[tube].digit = 10;
    tube_list[tube].dot   = LOW;
  }

  sei();
}


void  display_time ()
{
  static  TimeElements    tm;

  time_t          t;
  unsigned long   ms;
  boolean         dot;
  byte            u;

  ms  = millis() % 1000;
  dot = (ms < 500) ? HIGH : LOW;

  t = now();
  breakTime(t,tm);

  cli();

  u = tm.Second;
  if ((u >= 50) && (u <= 54))
  {
    u = tm.Day;

    tube_list[1].digit = (u % 10);
    tube_list[1].dot = dot;

    tube_list[0].digit = (u / 10);
    tube_list[0].dot = dot;

    u = tm.Month;

    tube_list[3].digit = (u % 10);
    tube_list[3].dot = dot;

    tube_list[2].digit = (u / 10);
    tube_list[2].dot = dot;
  }
  else
  {
    u = tm.Hour;

    tube_list[1].digit = (u % 10);
    tube_list[1].dot = dot;

    tube_list[0].digit = (u / 10);
    tube_list[0].dot = LOW;

    u = tm.Minute;

    tube_list[3].digit = (u % 10);
    tube_list[3].dot = LOW;

    tube_list[2].digit = (u / 10);
    tube_list[2].dot = LOW;
  }

  sei();
}


void  loop ()
{
  rcv();

  if (timeStatus() == timeNotSet)
  {
    no_time();
  }
  else
  {
    display_time();
  }
}
