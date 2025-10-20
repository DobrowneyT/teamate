// /**
//  * @file TeaMate_Tones.h
//  * @brief Hardware timer-based tone functions for TeaMate robot using Timer5
//  * @date 2025-03-25
//  */

// #ifndef TEAMATE_TONES_H
// #define TEAMATE_TONES_H

// // Globals for the timer (volatile because they're modified in ISR)
// volatile boolean isPlaying = false;
// volatile unsigned long toneDuration = 0;
// volatile unsigned long toneStartTime = 0;

// // Forward declarations to solve ordering issues
// void hardwareNoTone(uint8_t pin);

// /**
//  * @brief Hardware timer-based tone implementation using Timer5
//  * @param pin The buzzer pin
//  * @param frequency Frequency in Hz
//  * @param duration Duration in milliseconds
//  */
// void hardwareTone(uint8_t pin, unsigned int frequency, unsigned long duration) {
//   // Stop any currently playing tone
//   hardwareNoTone(pin);
  
//   // For Mega, we'll use Timer5 which is less commonly used by libraries
  
//   // Save the pin and duration
//   isPlaying = true;
//   toneDuration = duration;
//   toneStartTime = millis();
  
//   // Set pin as output
//   pinMode(pin, OUTPUT);
  
//   // Calculate the timer value for the desired frequency
//   // Timer5 is a 16-bit timer, so its top value is 65535
//   unsigned int timerValue;
  
//   // For higher precision at different frequencies, use appropriate prescaler
//   byte prescaler;
  
//   if (frequency < 100) {
//     prescaler = 256; // Use prescaler of 256 for very low frequencies
//     timerValue = (16000000UL / (256UL * frequency * 2UL)) - 1;
//   } else if (frequency < 500) {
//     prescaler = 64;  // Use prescaler of 64 for low frequencies
//     timerValue = (16000000UL / (64UL * frequency * 2UL)) - 1;
//   } else if (frequency < 2000) {
//     prescaler = 8;   // Use prescaler of 8 for mid frequencies
//     timerValue = (16000000UL / (8UL * frequency * 2UL)) - 1;
//   } else {
//     prescaler = 1;   // Use prescaler of 1 for high frequencies
//     timerValue = (16000000UL / (1UL * frequency * 2UL)) - 1;
//   }
  
//   // Make sure the timer value is within range
//   timerValue = constrain(timerValue, 1, 65535);
  
//   // Set up Timer5 for CTC mode (Clear Timer on Compare Match)
//   // WGM53:0 = 0100 for CTC mode (TOP=OCR5A)
//   TCCR5A = 0;                   // No output pins
//   TCCR5B = _BV(WGM52);          // CTC mode, TOP=OCR5A
  
//   // Set the compare value
//   OCR5A = timerValue;
  
//   // Configure timer interrupt to fire when timer reaches OCR5A
//   TIMSK5 = _BV(OCIE5A);         // Enable compare match interrupt
  
//   // Set prescaler and start the timer
//   if (prescaler == 1) {
//     TCCR5B |= _BV(CS50);        // No prescaler
//   } else if (prescaler == 8) {
//     TCCR5B |= _BV(CS51);        // Prescaler = 8
//   } else if (prescaler == 64) {
//     TCCR5B |= _BV(CS51) | _BV(CS50); // Prescaler = 64
//   } else if (prescaler == 256) {
//     TCCR5B |= _BV(CS52);        // Prescaler = 256
//   }
  
//   // Enable global interrupts
//   sei();
  
//   // If duration is specified, wait for it to complete
//   if (duration > 0) {
//     delay(duration);
//     hardwareNoTone(pin);
//   }
// }

// /**
//  * @brief Stop the hardware timer tone
//  * @param pin The buzzer pin
//  */
// void hardwareNoTone(uint8_t pin) {
//   // Stop Timer5
//   TCCR5A = 0;
//   TCCR5B = 0;
//   TIMSK5 = 0;
  
//   // Reset output pin
//   digitalWrite(pin, LOW);
//   isPlaying = false;
// }

// /**
//  * @brief Play the alternative "Tea Done" melody (sequence 8 from the test)
//  * This plays an ascending C-E-G melody
//  */
// void playTeaDoneMelody() {
//   // Use the constants from the main file
//   // First note - C6
//   hardwareTone(BUZZER_PIN, NOTE_C6, NOTE_DURATION_SHORT);
//   delay(NOTE_PAUSE);
  
//   // Second note - E6
//   hardwareTone(BUZZER_PIN, NOTE_E6, NOTE_DURATION_SHORT);
//   delay(NOTE_PAUSE);
  
//   // Final note - G6 
//   hardwareTone(BUZZER_PIN, NOTE_G6, NOTE_DURATION_LONG);
// }

// /**
//  * @brief Timer5 Compare A interrupt handler
//  * This toggles the pin state each time Timer5 reaches the compare value
//  */
// #ifdef __cplusplus
// extern "C" {
// #endif
// ISR(TIMER5_COMPA_vect) {
//   if (isPlaying) {
//     // Toggle the pin state for square wave
//     static boolean state = false;
//     digitalWrite(BUZZER_PIN, state = !state);
    
//     // Check if we need to stop the tone
//     if (toneDuration > 0 && (millis() - toneStartTime >= toneDuration)) {
//       hardwareNoTone(BUZZER_PIN);
//     }
//   }
// }
// #ifdef __cplusplus
// }
// #endif

// #endif /* TEAMATE_TONES_H */



/**
 * @file TeaMate_Tones.h
 * @brief Simple software-based tone functions for TeaMate robot
 * @date 2025-03-25
 */

#ifndef TEAMATE_TONES_H
#define TEAMATE_TONES_H

/**
 * @brief Software-based tone implementation
 * @param pin The buzzer pin
 * @param frequency Frequency in Hz
 * @param duration Duration in milliseconds
 */
void hardwareTone(uint8_t pin, unsigned int frequency, unsigned long duration) {
  // For very low frequencies, handle specially
  if (frequency < 100) {
    unsigned long period_ms = 1000 / frequency;
    unsigned long cycles = duration / period_ms;
    
    for (unsigned long i = 0; i < cycles; i++) {
      digitalWrite(pin, HIGH);
      delay(period_ms / 2);
      digitalWrite(pin, LOW);
      delay(period_ms / 2);
    }
    return;
  }
  
  // For higher frequencies, we need to use microsecond timing
  unsigned long period_us = 1000000 / frequency;
  unsigned long half_period_us = period_us / 2;
  unsigned long start_time = millis();
  unsigned long end_time = start_time + duration;
  
  pinMode(pin, OUTPUT);
  
  // Generate square wave for the specified duration
  while (millis() < end_time) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(half_period_us);
    digitalWrite(pin, LOW);
    delayMicroseconds(half_period_us);
    
    // // Allow time for motor updates every few cycles to maintain robot responsiveness
    // static int cycle_count = 0;
    // if (++cycle_count >= 10) {
    //   cycle_count = 0;
    //   update_all_motors();
    // }
  }
}

/**
 * @brief Stop the tone
 * @param pin The buzzer pin
 */
void hardwareNoTone(uint8_t pin) {
  digitalWrite(pin, LOW);
}

/**
 * @brief Play the Tea Done melody
 * This plays an ascending C-E-G melody
 */
void playTeaDoneMelody() {
  // First note - C6
  hardwareTone(BUZZER_PIN, NOTE_C6, NOTE_DURATION_SHORT);
  delay(NOTE_PAUSE);
  
  // Second note - E6
  hardwareTone(BUZZER_PIN, NOTE_E6, NOTE_DURATION_SHORT);
  delay(NOTE_PAUSE);
  
  // Final note - G6
  hardwareTone(BUZZER_PIN, NOTE_G6, NOTE_DURATION_LONG);
}

#endif /* TEAMATE_TONES_H */