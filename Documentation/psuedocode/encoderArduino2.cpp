#define ENCODER_0INTERRUPT_PIN 18 // pin that interrupts on both rising and falling of A and B channels of encoder


#define ENCODER_1INTERRUPT_PIN 19

#define ENCODER_2INTERRUPT_PIN 20



// declared as volatile as is recommended of variables that are accessed in a ISR (Interrupt service routine)
volatile int64_t encoderCounts[] = {0,0,0};
char motor0Dir = '';
char motor1Dir = '';
char motor2Dir = '';

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



void encoder1_ISR()
{
	noInterrupts();
	if(motor0Dir == 'b')
	{
		encoderCounts[0] = encorderCounts[0] - 1;
	}
	
	{
		
	}
	
}
