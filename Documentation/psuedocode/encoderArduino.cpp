#define ENCODER_0INTERRUPT_PIN 0 // pin that interrupts on both rising and falling of A and B channels of encoder
#define ENCODER_0A_PIN 1 
#define ENCODER_0B_PIN 2

#define ENCODER_1INTERRUPT_PIN 3
#define ENCODER_1A_PIN 4
#define ENCODER_1B_PIN 5

#define ENCODER_2INTERRUPT_PIN 6
#define ENCODER_2A_PIN  7
#define ENCODER_2B_PIN 8

 // A0,B0,A1,B1,A2,B2
const char encoder[6] = {ENCODER_0A_PIN,ENCODER_0B_PIN,ENCODER_1A_PIN,ENCODER_1B_PIN,ENCODER_2A_PIN,ENCODER_2B_PIN}

// declared as volatile as is recommended of variables that are accessed in a ISR (Interrupt service routine)
volatile int64_t encoderCounts[] = {0,0,0};

/*
 *	There are two ways to do this here. One would be to us the interrupt line that
 *	the motor drive creates and combines both A and B lines together. This has the
 *	advantage of freeing up 3 interrupt lines, but requires more RAM for keeping track
 *	of prev states and is logically harder to follow. Still would need the same amount of
 * 	digital pins just don't need to be interrupt  pins
 */
attachInterrupt(digitalPinToInterrupt(ENCODER_0INTERRUPT_PIN,encoder0_ISR, CHANGE);
attachInterrupt(digitalPinToInterrupt(ENCODER_1INTERRUPT_PIN,encoder1_ISR, CHANGE);
attachInterrupt(digitalPinToInterrupt(ENCODER_2INTERRUPT_PIN,encoder2_ISR, CHANGE);


