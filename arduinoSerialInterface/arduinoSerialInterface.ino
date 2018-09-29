#include <HardwareSerial.h>
#include <SimpleTimer.h>
#define ENCODER_0INTERRUPT_PIN 5// pin that interrupts on both rising and falling of A and B channels of encoder
#define ENCODER_1INTERRUPT_PIN 4 // https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/ to see the encoder pin number
#define ENCODER_2INTERRUPT_PIN 3

#define FORWARD 0
#define BACKWARDS 1
#define INFRARED_SENSOR_0 A0
#define INFRARED_SENSOR_1 A1
#define INFRARED_SENSOR_2 A2
#define INFRARED_SENSOR_3 A3
#define VELOCITY_TIME 500
volatile long encoderCounts[] = {
  0,0,0}; // variables accesed inside of an interrupt need to be volatile
bool motorDir[3] = {
  FORWARD,FORWARD,FORWARD};

const int motorPWMPins[3]            = {
  8,9,10};
const int motorDirPins[3]            = {
  29,27,28};
const int ultrasonicSensorTrigPins[] = {
  30,32,34,36,38,40};
const int ultrasonicSensorEchoPins[] = {
  31,33,35,37,39,41};
const int infraredSensorPins[] = {0,1,2,3};
  
  
double velocityValues[]   = {0,0,0};
long pastEncoderValues[]  = {0,0,0};
unsigned long pastTimes[] = {0,0,0};// millis() works for up to 50days! we'll need an unsigned long for it


char rcv_buffer[64];  // holds commands recieved
char TXBuffer[64];    // temp storage for large data sent 
void motor(int,int,bool);
SimpleTimer velocityTimer;
void checkVelocity();



void setup() {

  Serial.begin(115200);
  for(int i =0;i<6;i++)
  {
    pinMode(ultrasonicSensorTrigPins[i], OUTPUT);
    pinMode(ultrasonicSensorEchoPins[i], INPUT);
  }

  velocityTimer.setInterval(VELOCITY_TIME,checkVelocity);
  //INFRARED SENSORS
  pinMode(INFRARED_SENSOR_0,INPUT);
  pinMode(INFRARED_SENSOR_1,INPUT);
  pinMode(INFRARED_SENSOR_2,INPUT);
  pinMode(INFRARED_SENSOR_3,INPUT);
  //Left Motor
  pinMode(motorPWMPins[0], OUTPUT);
  pinMode(motorDirPins[0], OUTPUT); //LOW=CCW HIGH=CW

  //Right Motor
  pinMode(motorPWMPins[1], OUTPUT);
  pinMode(motorDirPins[1], OUTPUT);

  //Rear Motor
  pinMode(motorPWMPins[2], OUTPUT);
  pinMode(motorDirPins[2], OUTPUT);
  buffer_Flush(rcv_buffer);
  pinMode(2,INPUT_PULLUP);
  pinMode(19,INPUT_PULLUP);
  pinMode(20,INPUT_PULLUP);
  while (! Serial);
  attachInterrupt(ENCODER_0INTERRUPT_PIN,encoder0_ISR,CHANGE);
  attachInterrupt(ENCODER_1INTERRUPT_PIN,encoder1_ISR,CHANGE);
  attachInterrupt(ENCODER_2INTERRUPT_PIN,encoder2_ISR,CHANGE);
  //  
}

void loop() {
  velocityTimer.run();

  //  while(encoderCounts[0] <= 2175 && test == false)
  //{
  //  motor(0,100,0);
  //  Serial.print("Encoder: ");
  // Serial.println(encoderCounts[0]);
  // }   
  // test = true;
  //  motor(0,0,1);
  receiveBytes();
  //checkVelocity();

}

void checkVelocity() {
  
  
  for( int i=0; i<3;i++)
  {
    velocityValues[i] = (((encoderCounts[i]-pastEncoderValues[i])*0.08567979964)/((millis()-pastTimes[i])*.001));
    pastTimes[i]= millis();
    pastEncoderValues[i]=encoderCounts[i];
    
//    Serial.print("Encoder ");
//    Serial.print(i);
//    Serial.print(" ");
//    Serial.println(velocityValues[i]);
  }
  //double tempRPM;
  //  Serial.println(encoderCounts[0]-pastEncoderValue);
  //  Serial.println((millis()-pastTime));
  //  velocity = (((encoderCounts[0]-pastEncoderValue)*0.08567979964)/((millis()-pastTime)* .001)*.001);
  //tempRPM = ((((encoderCounts[0]-pastEncoderValue)/2175)/((millis()-pastTime) * .001 ))*60);

  //pastTime = millis();
  //pastEncoderValue = encoderCounts[0];
  //  Serial.print("meters per  sec: ");
  // 
  //  Serial.println(velocity);
  //Serial.print("Revolutions per min: ");
  //Serial.println((tempRPM));


}

void encoder0_ISR() // encoder0 interrupt service routine 
{
  noInterrupts();
  if(!motorDir[0])
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
  if(!motorDir[1])
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
  if(!motorDir[2])
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
  char terminator = '\r';
  while(Serial.available() > 0)
  {
    rcv_buffer[index] = Serial.read();
    if(rcv_buffer[index] == terminator)
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
  char command = rcv_buffer[0];

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

  default:
    Serial.println("Error: Serial input incorrect");


  }
}

double get_IR(uint16_t value){
  if (value < 16)  value = 16;
  //return 4800.0 / (value - 1120.0);
  return 4800.0 / (value - 20.0);

}

