// this is the code that seems to work well with the ultrasonic sensor
//
// version 20141202-1

int pinI1=8;//define I1 interface
int pinI2=11;//define I2 interface 
int speedpinA=9;//enable motor A
int pinI3=12;//define I3 interface 
int pinI4=13;//define I4 interface 
int speedpinB=10;//enable motor B

int headPin=5; // head servo digital pin

// motor speed, adjust depending upon the number of volts sent into the motor shield.
// for instance, if 12v, 255 is really too fast. crank it down
int motorSpeed =55;//define the motorSpeed of motor

// to enable serial writes of debug messages, set debug=1
int debug=0; // 0=no 1=yes

#define echoPin 7 // Echo Pin
#define trigPin 6 // Trigger Pin
#define LEDPin 13 // Onboard LED

long duration, distance; // Duration used to calculate distance

const int analogSensorPin = 0;

// the sensor trigger distance in cm
int triggerDistanceCM = 8;
int triggerAnalogValue = 500;

int angle;
int pwm;
 
void setup()
{  
  pinMode(headPin, OUTPUT);

  pinMode(pinI1,OUTPUT);
  pinMode(pinI2,OUTPUT);
  pinMode(speedpinA,OUTPUT);
  pinMode(pinI3,OUTPUT);
  pinMode(pinI4,OUTPUT);
  pinMode(speedpinB,OUTPUT);
  if (debug>0) Serial.begin(9600);
 
  if (trigPin>0){
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(LEDPin, OUTPUT); // Use LED indicator (if required)
  }  
}
 
void loop(){
  if (trigPin>0 || analogSensorPin>0) {
    // if you have an active sensor, you'll always wind up in here
    if (trigPin>0){
      // an ultrasonic sensor is attached
      int cm=getDistance();
      if (debug>0) {
        Serial.print("cm:");
        Serial.println(cm);
      }
      if (cm<=triggerDistanceCM){
        // too close to an object, do something different like backup or turn left
        
        stop();
        drive("backward", 800);
        stop();
        drive("left", 800);
        stop();
        
        if (headPin>0){
         for (angle = 0; angle <= 140; angle += 5)  {
           servoPulse(headPin, angle);  }
         for (angle = 140; angle >= 0; angle -= 5)  {
           servoPulse(headPin, angle);  }
         delay(1000);
        }
        
      } else {
        // distance to object is above threshold, keep moving forward
        drive("forward", 1000);
      }
    }
    
    if (analogSensorPin>0){
      // an analog sensor such as a potentiometer is attached
      if (getAnalogResistance()<=triggerAnalogValue){
        // analog value is too high, do something different
        drive("left", 300);
      } else {
        // analog value is in acceptable range, just drive forward
        drive("forward", 1000);
      }
    } 
  } else {
    // this is what gets done every loop if there are no sensors (digital ultrasonic, analog pot, etc...)
    drive("forward", 1000);
  }
}

void drive(String dir, int t){
  if (debug>0){
      Serial.println(dir);
    } else {
    if (dir=="right"){
      analogWrite(speedpinA,motorSpeed);//input a simulation value to set the speed
      analogWrite(speedpinB,motorSpeed);
      digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
      digitalWrite(pinI3,LOW);
      digitalWrite(pinI2,LOW);//turn DC Motor A move anticlockwise
      digitalWrite(pinI1,HIGH);
    }
    
    if (dir=="left"){
      analogWrite(speedpinA,motorSpeed);//input a simulation value to set the speed
      analogWrite(speedpinB,motorSpeed);
      digitalWrite(pinI4,LOW);//turn DC Motor B move anticlockwise
      digitalWrite(pinI3,HIGH);
      digitalWrite(pinI2,HIGH);//turn DC Motor A move clockwise
      digitalWrite(pinI1,LOW);
    }
    
    if (dir=="forward"){
      analogWrite(speedpinA,motorSpeed);//input a simulation value to set the speed
      analogWrite(speedpinB,motorSpeed);
      digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
      digitalWrite(pinI3,LOW);
      digitalWrite(pinI2,HIGH);//turn DC Motor A move clockwise
      digitalWrite(pinI1,LOW);
    }
    
    if (dir=="backward"){
      analogWrite(speedpinA,motorSpeed);//input a simulation value to set the speed
      analogWrite(speedpinB,motorSpeed);
      digitalWrite(pinI4,LOW);//turn DC Motor B move anticlockwise
      digitalWrite(pinI3,HIGH);
      digitalWrite(pinI2,LOW);//turn DC Motor A move clockwise
      digitalWrite(pinI1,HIGH);
    }
  }
    
  delay(t);
  //if (debug==0) stop();
}

void stop(){
   digitalWrite(speedpinA,LOW);// Unenble the pin, to stop the motor. this should be done to avid damaging the motor. 
   digitalWrite(speedpinB,LOW);
   delay(300);
}

long msTocm(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

int getAnalogResistance(){
  int sensorValue = analogRead(A1);
  if (debug>0) Serial.println(sensorValue);
  delay(1);        // delay in between reads for stability
  return sensorValue;
}

int getDistance(){
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2); 
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
   
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
   
  //Calculate the distance (in cm) based on the speed of sound.
  distance = duration/58.2;
 
  if (debug>0) {
    Serial.print("cm");
    Serial.println();
  }
 
  //Delay 50ms before next reading.
  delay(50);
 
  return distance;
}

void servoPulse (int servo, int angle)
{
 pwm = (angle*11) + 500;      // Convert angle to microseconds
 digitalWrite(servo, HIGH);
 delayMicroseconds(pwm);
 digitalWrite(servo, LOW);
 delay(50);                   // Refresh cycle of servo
}
