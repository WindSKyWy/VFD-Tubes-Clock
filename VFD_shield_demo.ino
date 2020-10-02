
/*
  VFD Arduino Shield demo

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
*/


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

static  const  byte  digit_seg_data[10*7] PROGMEM =
{
/* pin    2     3     4     5     6     7     8  */
        HIGH, HIGH, HIGH, HIGH, HIGH, HIGH,  LOW,
         LOW, HIGH, HIGH,  LOW,  LOW,  LOW,  LOW,
        HIGH, HIGH,  LOW, HIGH, HIGH,  LOW, HIGH,
        HIGH, HIGH, HIGH, HIGH,  LOW,  LOW, HIGH,
         LOW, HIGH, HIGH,  LOW,  LOW, HIGH, HIGH,
        HIGH,  LOW, HIGH, HIGH,  LOW, HIGH, HIGH,
        HIGH,  LOW, HIGH, HIGH, HIGH, HIGH, HIGH,
        HIGH, HIGH, HIGH,  LOW,  LOW,  LOW,  LOW,
        HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH,
        HIGH, HIGH, HIGH, HIGH,  LOW, HIGH, HIGH
};


typedef  struct  _TUBE
{
  byte    digit;      // 0..9
  byte    dot;        // HIGH or LOW
}
  TUBE;


typedef  struct  _LED
{
  byte    pwm_off;    // 0..7, 8 means always on, 255 means repeat from start
  byte    wait_cnt;   // 0..65535
}
  LED;


// Display state of the tubes (read-write)

static  TUBE    tube_list[4] =
{
  { 1,  LOW },
  { 2,  LOW },
  { 3,  LOW },
  { 4, HIGH }
};


// LED PWM sequence data (read-only)

static  const  LED  led_seq[] PROGMEM =
{
  {   8, 200 },
  {   7,   4 },
  {   6,   4 },
  {   5,   3 },
  {   4,   3 },
  {   3,   2 },
  {   2,   3 },
  {   1,   3 },
  {   0,   4 },
  {   1,   3 },
  {   2,   3 },
  {   3,   2 },
  {   4,   3 },
  {   5,   3 },
  {   6,   4 },
  {   7,   4 },
  { 255,   0 }
};


// Variables accessed at interrupt level

static  byte  cur_tube      = 3;    // 0..3
static  byte  tube_toggle   = 0;
static  word  tube_wait_cnt = 0;
static  word  dot_wait_cnt  = 41;
static  byte  led_pwm_step  = 0;    // 0..7
static  byte  led_pwm_off   = 0;    // 0..7, 8 means always on
static  byte  led_seq_index = 0;
static  byte  led_wait_cnt  = 0;


void  isr_drive_leds ()
{
  // LED PWM

  if (led_pwm_step == led_pwm_off)
  {
    digitalWrite(10,LOW);
  }
  else
  {
    if (led_pwm_step == 0)
    {
      digitalWrite(10,HIGH);
    }
  }
}


void  isr_step_led_seq ()
{
  byte  pwm_off;

  // Fetch the PWM off position. If 255, start over from the beginning of the sequence data.
  for (;;)
  {
    pwm_off = pgm_read_byte(&led_seq[led_seq_index].pwm_off);
    if (pwm_off != 255) break;
    led_seq_index = 0;
  }

  led_pwm_off  = pwm_off;
  led_wait_cnt = pgm_read_byte(&led_seq[led_seq_index].wait_cnt);

  led_seq_index++;
}


void  isr_animate_leds ()
{
  led_pwm_step++;
  if (led_pwm_step == 8)
  {
    led_pwm_step = 0;

    // LED PWM sequence
    led_wait_cnt--;
    if (led_wait_cnt == 0) isr_step_led_seq();
  }
}


void  isr_drive_tubes ()
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


void  isr_animate_tubes ()
{
  byte    tube;
  byte    digit;

  tube_wait_cnt++;
  if (tube_wait_cnt == 83)
  {
    // Reset the wait counter
    tube_wait_cnt = 0;

    // Change the digits
    for (tube = 0; tube < 4; tube++)
    {
      digit = tube_list[tube].digit;
      if (digit == 9) digit = 0; else digit++;
      tube_list[tube].digit = digit;
    }
  }

  dot_wait_cnt++;
  if (dot_wait_cnt == 299)
  {
    // Reset the wait counter
    dot_wait_cnt = 0;

    // Animate the dot as an incrementing 4-bit binary number
    tube_list[3].dot ^= 1;
    if (tube_list[3].dot == LOW)
    {
      tube_list[2].dot ^= 1;
      if (tube_list[2].dot == LOW)
      {
        tube_list[1].dot ^= 1;
        if (tube_list[1].dot == LOW)
        {
          tube_list[0].dot ^= 1;
        }
      }
    }
  }
}


// Timer 1 interrupt service routine
//
// 500 Hz

ISR(TIMER1_COMPA_vect)
{
  // Drive tubes at 250 Hz
  tube_toggle ^= 1;
  if (tube_toggle)
  {
    isr_drive_tubes();
    isr_animate_tubes();
  }

  // PWM for LEDs requires 500 Hz
  isr_drive_leds();
  isr_animate_leds();
}


void  setup ()
{
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

  // Kick of the LED PWM sequence
  isr_step_led_seq();

  sei();
}


void  loop ()
{
}
