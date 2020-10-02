
/*
  VFD Arduino Shield as a volt meter.

  This program samples analog input A5 and displays a value between 0.00 and 5.00 on the tubes.

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
        HIGH, HIGH, HIGH, HIGH, HIGH, HIGH,  LOW,    // Digit 0
         LOW, HIGH, HIGH,  LOW,  LOW,  LOW,  LOW,    // Digit 1
        HIGH, HIGH,  LOW, HIGH, HIGH,  LOW, HIGH,    // Digit 2
        HIGH, HIGH, HIGH, HIGH,  LOW,  LOW, HIGH,    // Digit 3
         LOW, HIGH, HIGH,  LOW,  LOW, HIGH, HIGH,    // Digit 4
        HIGH,  LOW, HIGH, HIGH,  LOW, HIGH, HIGH,    // Digit 5
        HIGH,  LOW, HIGH, HIGH, HIGH, HIGH, HIGH,    // Digit 6
        HIGH, HIGH, HIGH,  LOW,  LOW,  LOW,  LOW,    // Digit 7
        HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH,    // Digit 8
        HIGH, HIGH, HIGH, HIGH,  LOW, HIGH, HIGH     // Digit 9
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
  { 0,  LOW },
  { 0, HIGH },
  { 0,  LOW },
  { 0,  LOW }
};


// Variables accessed at interrupt level

static  byte    cur_tube   = 3;        // 0..3
static  byte    isr_cnt    = 0;
static  boolean req_sample = true;     // Request sampling of the analog input


// Timer 1 interrupt service routine

ISR(TIMER1_COMPA_vect)
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


  // Request sampling of the analog input periodically
  isr_cnt++;
  if (isr_cnt == 125)
  {
    isr_cnt    = 0;
    req_sample = true;
  }
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

  // Turn on the LEDs
  digitalWrite(10,HIGH);

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
  OCR1A  = 250-1;       // 250 Hz (62500/250)
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


void  loop ()
{
  boolean req;

  cli();
  req = req_sample;
  req_sample = false;
  sei();

  if (req)
  {
    int adc_val;
    int voltage;

    adc_val = analogRead(A5);

    // Convert 0..1023 to 0..500 using 32-bit signed calculus
    voltage = (int)(((long)adc_val * 500L) / 1023L);
 
    cli();
    set_tubes_val(voltage);
    sei();
  }
}
