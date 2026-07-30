#include <Arduino.h>
#include <avr/interrupt.h>

#include "config.hpp"
#include "motor_driver.hpp"
#include "encoder.hpp"


namespace
{
    volatile long encoderPosition = 0;
    volatile uint8_t lastEncoded = 0;

    /*
     * Standard 4x quadrature decode table, indexed by
     * (previousState << 2 | currentState), where each state is
     * (A << 1 | B).
     *
     * Valid single-step transitions map to +1 or -1. Anything else -
     * a double-step, a repeated state, or any transition that quadrature
     * signals can't legitimately produce on one edge - maps to 0 and is
     * ignored rather than corrupting the count. This makes the count
     * robust against noise glitches (e.g. PWM-induced EMI on the
     * encoder lines), which would otherwise show up as spurious
     * movement, particularly reverse movement, under motor load.
     */
    const int8_t transitionTable[16] =
    {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };
}

void initialiseEncoder()
{
    /*
     * INPUT_PULLUP is used because these Hall-effect encoders typically
     * have open-collector outputs and need a pull-up to read cleanly.
     * If your encoder module already has its own pull-ups (push-pull
     * output, or external resistors on the board), this is harmless.
     */
    pinMode(Config::ENCODER_A, INPUT_PULLUP);
    pinMode(Config::ENCODER_B, INPUT_PULLUP);

    lastEncoded =
        (digitalRead(Config::ENCODER_A) << 1) |
        digitalRead(Config::ENCODER_B);

    /*
     * ENCODER_A (pin 3) is a true external-interrupt pin (INT1) on the
     * ATmega328, so attachInterrupt() works normally here.
     */
    attachInterrupt(
        digitalPinToInterrupt(Config::ENCODER_A),
        readEncoder,
        CHANGE
    );

    /*
     * ENCODER_B (pin 6) is NOT an external-interrupt-capable pin on the
     * ATmega328 - only D2/D3 support attachInterrupt(). Calling
     * attachInterrupt() on it would silently do nothing, meaning B
     * transitions would never be caught and every count would rely on
     * A's edges alone with a possibly-stale B sample, which is exactly
     * the kind of race that produces spurious counts under noise.
     *
     * Pin 6 is PD6 = PCINT22, so it's wired into the AVR's pin-change
     * interrupt system directly instead.
     */
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT22); // PD6 = Arduino digital pin 6
}

long getEncoderPosition(){
  noInterrupts();
  long position = encoderPosition;
  interrupts();

  return position;
}

void setEncoderPosition(long position){
  noInterrupts();
  encoderPosition = position;
  lastEncoded =
      (digitalRead(Config::ENCODER_A) << 1) |
      digitalRead(Config::ENCODER_B);
  interrupts();
}

void readEncoder(){
  const uint8_t encoded =
      (digitalRead(Config::ENCODER_A) << 1) |
      digitalRead(Config::ENCODER_B);

  const uint8_t index = (lastEncoded << 2) | encoded;

  encoderPosition += transitionTable[index];

  lastEncoded = encoded;
}

/*
 * Pin-change interrupt vector for port D (pins 0-7), covering
 * ENCODER_B (pin 6). Only PCINT22 is enabled above, so this fires
 * solely on ENCODER_B transitions.
 */
ISR(PCINT2_vect)
{
    readEncoder();
}