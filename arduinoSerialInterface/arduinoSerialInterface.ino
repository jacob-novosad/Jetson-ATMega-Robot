#include <HardwareSerial.h>
#include <SimpleTimer.h>



#define ENCODER_0INTERRUPT_PIN 5// pin  18that interrupts on both rising and falling of A and B channels of encoder
#define ENCODER_1INTERRUPT_PIN 4 // pin 19 https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/ to see the encoder pin number
#define ENCODER_2INTERRUPT_PIN 3 // pin 20

#define FORWARD 0
#define BACKWARD 1
#define INFRARED_SENSOR_0 A0
#define INFRARED_SENSOR_1 A1
#define INFRARED_SENSOR_2 A2
#define INFRARED_SENSOR_3 A3
#define VELOCITY_TIME 10          //Every # miliseconds we update our rpm of wheels



volatile long encoderCounts[] = {
  0,0,0}; // variables accesed inside of an interrupt need to be volatile
bool motorDir[3] = {
  FORWARD,FORWARD,FORWARD};

const int motorPWMPins[3]            = {
  8,9,10};
const int motorDirPins[3]            = {
  29,28,27};
const int ultrasonicSensorTrigPins[] = {
  30,32,34,36,38,40};
const int ultrasonicSensorEchoPins[] = {
  31,33,35,37,39,41};
const int infraredSensorPins[] = {
  0,1,2,3};

double Kp = 0; // 12
double Ki = .1/60; // .1
//double Kd = 0;

double sum[3]        = {0,0,0};
double error[3]      = {0,0,0};
double setpoint[3]   = {0,0,0};
double pwmValue[3]   = {0,0,0};

unsigned long lastTime[3]   = {
  0,0,0};
unsigned long timeChange[3] = {
  0,0,0};

double velocityValues[3]   = {
  0,0,0};
float rpmValues[3]        = {
  0,0,0};
long pastEncoderValues[3]  = {
  0,0,0};
unsigned long pastTimes[3] = {
  0,0,0};// millis() works for up to 50days! we'll need an unsigned long for it


char rcv_buffer[64];  // holds commands recieved
char TXBuffer[64];    // temp storage for large data sent 
void motor(int,int,bool);
SimpleTimer rpmUpdateTimer;
void updateRPM();
double changeInEncoders;
double changeInRevolutions;
double changeInTimeSeconds;



void setup() {

  Serial.begin(115200);
  for(int i =0;i<6;i++)
  {
    pinMode(ultrasonicSensorTrigPins[i], OUTPUT);
    pinMode(ultrasonicSensorEchoPins[i], INPUT);
  }

  rpmUpdateTimer.setInterval(VELOCITY_TIME,updateRPM);
  //INFRARED SENSORS
  pinMode(INFRARED_SENSOR_0,INPUT);
  pinMode(INFRARED_SENSOR_1,INPUT);
  pinMode(INFRARED_SENSOR_2,INPUT);
  pinMode(INFRARED_SENSOR_3,INPUT);
  // Motor
  for(int i =0; i<3;i++)
  {
    pinMode(motorPWMPins[i], OUTPUT);
    pinMode(motorDirPins[i], OUTPUT); //LOW=CCW HIGH=CW
  }

  pinMode(18,INPUT_PULLUP);
  pinMode(19,INPUT_PULLUP);
  pinMode(20,INPUT_PULLUP);


  buffer_Flush(rcv_buffer);

  while (! Serial);

  attachInterrupt(ENCODER_0INTERRUPT_PIN,encoder0_ISR,CHANGE);
  attachInterrupt(ENCODER_1INTERRUPT_PIN,encoder1_ISR,CHANGE);
  attachInterrupt(ENCODER_2INTERRUPT_PIN,encoder2_ISR,CHANGE);


}

void loop() {
  // this is our timer called every millisecond defined by VELOCITY_TIME (at top) to update our rpm 
  rpmUpdateTimer.run();

  // determines if we have any serial commands and interpruts them
  receiveBytes();

  // proportional integral controller
  pi();

}

void updateRPM() {

  for( int i=0; i<3;i++)
  {
    // linear velocity velocityValues[i] = (((encoderCounts[i]-pastEncoderValues[i])*0.08567979964)/((millis()-pastTimes[i])*.001));
    changeInEncoders = encoderCounts[i] - pastEncoderValues[i];
    changeInTimeSeconds = ((millis()-pastTimes[i])*.001);// *.001 to convert to seconds
    changeInRevolutions = changeInEncoders/2249;

    rpmValues[i] = (changeInRevolutions/(changeInTimeSeconds))*60; // *60 to get Revolutions per MINUTE
    //if(changeInEncoders >0)
    //Serial.println(changeInEncoders);

    // update our values to be used next time around
    pastTimes[i]= millis();
    pastEncoderValues[i]=encoderCounts[i];
    //printDouble(rpmValues[0],90000000);

  }
}

void pi() {

  //delay(20);

  for(int i =0;i<3;i++)
  {
    // if pi not set to 0 for robot
    if(setpoint[i] != 0)
    {
      
      //updateRPM();
      timeChange[i] = (millis() - lastTime[i]);
      lastTime[i] = millis();

      sum[i] = (sum[i] +(error[i]*(double)timeChange[i]));

      //sum = sum+error;
      //Serial.println(sum[i]);
      pwmValue[i] = (Kp * error[i]) + (Ki*sum[i]); doc

      if(pwmValue[i] < 0){
        motor(i,pwmValue[i]*-1,0);
      }
      else{
        motor(i,pwmValue[i],1);
      }

//      
//      Serial.println("---------------------------");
//      Serial.println(i);
//      Serial.print("Error: ");
//      Serial.println(error[i]);
//      Serial.print("Revs PM Value: ");
       // Serial.println(rpmValues[i]);
      // Serial.print(",");
      //  Serial.println(millis());
//      Serial.print("PWM Value: ");
//      Serial.println(pwmValue[i]);
      error[i] = setpoint[i] -rpmValues[i];
      
    }
    else
    {
      error[i] = 0;
      motor(i,0,0);
    }
  }
}

void encoder0_ISR() // encoder0 interrupt service routine 
{
  noInterrupts();
  if(motorDir[0])
  {
    encoderCounts[0]++;

  }
  else
  {
    encoderCounts[0]--;
  }
  interrupts();
}
void encoder1_ISR()
{
  noInterrupts();
  if(motorDir[1])
  {
    encoderCounts[1]++;
  }
  else
  {
    encoderCounts[1]--;
  }
  interrupts();
}
void encoder2_ISR()
{
  noInterrupts();
  if(motorDir[2])
  {
    encoderCounts[2]++;
  }
  else
  {
    encoderCounts[2]--;
  }
  interrupts();
}

void motor(int motorNumber, int pwm, bool dir)
{
  //dir = !dir;                              // This is to ensure positive RPM is CCW
  motorDir[motorNumber] = dir;

  digitalWrite(motorDirPins[motorNumber],dir);
  // could input check here for less than 255
  analogWrite(motorPWMPins[motorNumber], pwm);
  //  Serial.print("Motor: ");
  //  Serial.print(motorNumber);
  //  Serial.print(" Speed: ");
  //  Serial.print(pwm);
  //  Serial.print(" Direction: ");
  //  Serial.println(dir);

}

void receiveBytes()
{
  static byte index = 0;
  char terminator = '\r'; // what tells us our command is done
  while(Serial.available() > 0)
  {
    rcv_buffer[index] = Serial.read(); // read in our serial commands
    if(rcv_buffer[index] == terminator) // main loop for processing our command
    {
      index = 0;
      parseCommand();
      buffer_Flush(rcv_buffer);
    }
    else
    {
      index++;
      if(index >= 64)
      {
        Serial.println("buffer overflow");
        index = 0;
        buffer_Flush(rcv_buffer);
      }
    }
  }

}

void buffer_Flush(char *ptr)
{
  for(int i = 0; i < 64; i++)
  {
    ptr[i] = 0;
  }
}


void parseCommand()
{
  char command = rcv_buffer[0]; // our first byte tells us the command char is equivalent to byte

  //uint16_t value = analogRead(INFRARED_SENSOR_0);
  //  double distance = get_IR(value);
  switch(command)
  {
  case 'E':
  case 'e':
    int encoderNum;
    buffer_Flush(TXBuffer);
    sscanf(&rcv_buffer[1], " %d \r",&encoderNum);
    //itoa(encoderCounts[encoderNum],TXBuffer,10);   // serial.print can not handle printing a 64bit int so we turn it  into a string
    Serial.println(encoderCounts[encoderNum]);
    break;
  case 'M':
  case 'm':
    int  motorNumber;
    int  motorPWM;
    int motorDirection;

    sscanf(&rcv_buffer[1], " %d %d %d \r",&motorNumber, &motorPWM, &motorDirection);
    motor(motorNumber,motorPWM,motorDirection);
    break;
  case 'u':
  case 'U':
    int ultrasonicNumber;
    long duration,cm,inches;
    sscanf(&rcv_buffer[1], " %d \r",&ultrasonicNumber);

    digitalWrite(ultrasonicSensorTrigPins[ultrasonicNumber], LOW);
    delayMicroseconds(5);
    digitalWrite(ultrasonicSensorTrigPins[ultrasonicNumber], HIGH);
    delayMicroseconds(10);
    digitalWrite(ultrasonicSensorTrigPins[ultrasonicNumber], LOW);

    duration = pulseIn(ultrasonicSensorEchoPins[ultrasonicNumber], HIGH);
    cm = (duration/2) / 29.1;
    inches = (duration/2) / 74; 

    Serial.print(inches);
    Serial.print("in, ");
    Serial.print(cm);
    Serial.print("cm");
    Serial.println();
    break;

  case 'i':
  case 'I':
    uint16_t value;
    int infraredNumber;
    double distance;

    sscanf(&rcv_buffer[1], " %d \r",&ultrasonicNumber);
    value = analogRead(ultrasonicNumber);
    distance = get_IR(value);

    Serial.print (distance);
    Serial.println (" cm");
    Serial.println ();
    break;
  case 'v':
  case 'V':

    int rpm0;
    int rpm1;
    int rpm2;
    sscanf(&rcv_buffer[1], "%d %d %d \r",&rpm0,&rpm1,&rpm2);
    //Serial.println(rpm0);
    //Serial.println(rpm1);
    //Serial.println(rpm2);
    
    
   for(int i = 0;i<3;i++)
   {
//      error[i] = 0;
      sum[i]   = 0;
    }

    setpoint[0] = (double)(rpm0/10);

    setpoint[1] = (double)(rpm1/10);

    setpoint[2] = (double)(rpm2/10);

 // default:
    //Serial.println("Error: Serial input incorrect");


  }
}

double get_IR(uint16_t value){
  if (value < 16)  value = 16;
  //return 4800.0 / (value - 1120.0);
  return 4800.0 / (value - 20.0);

}


//supporting function to print doubles precicely 
void printDouble( double val, unsigned int precision){
  // prints val with number of decimal places determine by precision
  // NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
  // example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

  Serial.print (int(val));  //prints the int part
  Serial.print("."); // print the decimal point
  unsigned int frac;
  if(val >= 0)
    frac = (val - int(val)) * precision;
  else
    frac = (int(val)- val ) * precision;
  Serial.println(frac,DEC) ;
} 



