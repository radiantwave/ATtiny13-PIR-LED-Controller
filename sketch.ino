// Using MCUdude/MicroCore for an ATtiny13 @3.3V 600kHz

#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>  // Library for EEPROM access

// Pin Definitions
#define LED_PIN     PB0  // LED connected to PB0
#define MOTION_PIN  PB1  // Motion sensor connected to PB1
#define BUTTON_PIN  PB3  // Brightness change button connected to PB3

// ADC Channel 1 (PB2/Pin 7) inline

#define thresholdOnADC 900  // Threshold value for the light sensor

// Timer and status variables
unsigned long ledOnTime = 0;
const unsigned long ledDuration = 20000;

bool ledIsOn = false;  // Status of the LED

// PWM Duty Cycle values
const uint8_t pwmValues[] = {102, 153, 204, 255};
uint8_t currentPWMIndex = 0;

// EEPROM address for storing the PWM index
uint8_t EEMEM savedPWMIndex;

void setup() {
  // Disable interrupts to prevent any potential conflicts
  cli();

  // Increase OSCCAL value to calibrate the frequency
  OSCCAL = 0x69;

  // Timer 0 configuration for PWM on PB0
  TCCR0A = (1 << WGM00) | (1 << WGM01);  // Fast PWM mode
  TCCR0B = (1 << CS00);                  // No prescaling (prescaler = 1)

  // ADC configuration
  ADCSRA = (1 << ADEN) | (1 << ADPS1);  // Enable ADC, prescaler of 4 (600kHz / 4 = 150kHz)
  ADMUX = (1 << MUX0);                  // Select ADC1 (PB2) as input channel

  // Enable Pin Change Interrupts for PB1 (MOTION_PIN) and PB3 (BUTTON_PIN)
  GIMSK |= (1 << PCIE);  // Enable Pin Change Interrupts
  PCMSK |= (1 << MOTION_PIN) | (1 << BUTTON_PIN);  // Enable interrupts for PB1 and PB3

  // Set sleep mode to Power-down
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // Set LED_PIN as output and MOTION_PIN, BUTTON_PIN as input
  DDRB |= (1 << LED_PIN);         // PB0 as output
  DDRB &= ~(1 << MOTION_PIN);     // PB1 as input
  DDRB &= ~(1 << BUTTON_PIN);     // PB3 as input

  // Read the stored PWM index from EEPROM
  currentPWMIndex = eeprom_read_byte(&savedPWMIndex);
  // Ensure the index is within valid range
  if (currentPWMIndex > 3) {
    currentPWMIndex = 0;  // Set to default if stored value is invalid
  }
  // Set the initial PWM value
  OCR0A = pwmValues[currentPWMIndex];

  // Re-enable interrupts
  sei();
}

ISR(PCINT0_vect) {
  // Check if the button is pressed (pin is HIGH) and the LED is on
  if ((PINB & (1 << BUTTON_PIN)) && ledIsOn) {
    // Short delay for debouncing
    for (volatile uint16_t i = 0; i < 1000; i++);

    // Check again if the button is still pressed
    if (PINB & (1 << BUTTON_PIN)) {
      // Button was pressed and is stable
      currentPWMIndex = (currentPWMIndex + 1) % 4;  // Increase index and wrap around at 4
      OCR0A = pwmValues[currentPWMIndex];           // Set new PWM duty cycle
      // Save the new index to EEPROM
      eeprom_update_byte(&savedPWMIndex, currentPWMIndex);
    }
  }
}

// Reads the analog value from the ADC
inline uint16_t readADC() {
  ADCSRA |= (1 << ADSC);            // Start ADC conversion
  while (ADCSRA & (1 << ADSC));     // Wait for conversion to finish
  return ADC;
}

void loop() {
  // Read motion sensor
  bool motionDetected = PINB & (1 << MOTION_PIN);

  if (motionDetected) {
    if (!ledIsOn) {
      // Check ADC value to decide whether to turn on the LED
      uint16_t adcValue = readADC();
      if (adcValue < thresholdOnADC) {
        // ADC value is below threshold, turn on LED
        OCR0A = pwmValues[currentPWMIndex];  // Set duty cycle
        TCCR0A |= (1 << COM0A1);             // Enable PWM output on PB0

        ledIsOn = true;
        ledOnTime = millis();  // Save start time
      }
    } else {
      // LED is already on, reset timer
      ledOnTime = millis();  // Reset start time
    }
  } else if (!ledIsOn) {
    // No motion detected and LED is off, enter sleep mode
    cli();
    ADCSRA &= ~(1 << ADEN);     // Disable ADC
    sleep_enable();             // Enable sleep mode
    WDTCR &= ~(1 << WDTIE);     // Disable watchdog interrupt (needed for millis())
    // Disable Brown-out Detector (BOD) during sleep
    MCUCR |= (1 << BODS) | (1 << BODSE);
    MCUCR = (MCUCR & ~(1 << BODSE)) | (1 << BODS);
    sei();                      // Re-enable interrupts
    sleep_cpu();                // Enter sleep mode
    // Execution resumes here after waking up
    sleep_disable();            // Disable sleep mode after waking up
    WDTCR |= (1 << WDTIE);      // Re-enable watchdog interrupt (needed for millis())
    ADCSRA |= (1 << ADEN);      // Re-enable ADC
  }

  // Check if the LED should be turned off
  if (ledIsOn && (millis() - ledOnTime >= ledDuration)) {
    TCCR0A &= ~(1 << COM0A1);  // Disable PWM output on PB0
    ledIsOn = false;
  }
}
