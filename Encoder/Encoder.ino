int ENC_PIN_B = 2;
int ENC_PIN_A = 4;
int counts;
int samplingTime;
byte nextEncoderState[];
byte stopState[];

void setup() {
  // Setup encoder pins as inputs
  DDRB  &= ~(1 << ENC_PIN_B);
  DDRB  &= ~(1 << ENC_PIN_A);
  // Bias inputs
  PORTB |=  (1 << ENC_PIN_B); 
  PORTB |=  (1 << ENC_PIN_A);
  // Set global variable
  int counts = 0;
}

void loop() {
  enc_test();
}

void enc_test(void) {
// Test encoder outputs
  static int lastCount = 0;
  int encoder = enc_read();
  if (encoder) {
    counts += encoder;
    lastCount = counts;
  } 
}

int enc_read(void) {
// returns change in encoder state (-1: ccw, 0: no change, 1: cw) 
  int result = 0;
  static byte prevState = 0;
  static int bufferedCounts = 0;
  byte startState = State(); // Get current state
  delayMicroseconds(samplingTime); // Wait safety bounce time
  byte stopState = State(); // Get current state
  if ((startState == stopState) && (stopState != prevState)) { // check if the previous state was stable
    if (stopState == nextEncoderState[prevState]) {
      bufferedCounts++; 
    }
    else if (state == prevEncoderState[prevState]) {
      bufferedCounts--; 
    }
    prevState = state; // Record state for next pulse interpretation
    if (abs(bufferedCounts) == pulsesPerStep) {
      result = int(bufferedCounts / pulsesPerStep);
      bufferedCounts = 0;
    }
  }
  return(result);
}

byte State (void) {
  return(((PINB >> ENC_PIN_B) & 0x01) | (((PINB >> ENC_PIN_A) & 0x01) << 1));
}
