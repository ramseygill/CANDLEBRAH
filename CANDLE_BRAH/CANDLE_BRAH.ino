/*------------------------------------------
 *              AMALGABOT
 *------------------------------------------
 * Author: Ramsey Gill
 * Website: http://amalgamatronics.tumblr.com/
 * Created: February 2012
 * Version: 1.3
 ------------------------------------------------------------------------------------
 * References:
 * MAKE Magazine, Volume 19, p. 77 - June 2009 Kris Magri, Jameco Electronics J-Bot Sketch, 
 *"Getting Started With Arduino" pg. 41,49
 -------------------------------------------------------------------------------------
 * modified code for use with motor sheilds(Arduino R3 and Sparkfun L298P based controllers)
 * Dependencies: AMALGABOT Uses an Arduino 2560
 * License: GPLv3 (PUBLIC DOMAIN)
 *(http://www.fsf.org/licensing/
 */
//////////////////LIBRARIES//////////////////////////////////////////////////////////
  
  #include <Servo.h> // Enables the Servo library
  #include <LiquidCrystal.h>  //Liquid Crystal library for screen
  
////////////RANGE FINDER STUFF///////////////////////////////////////////////////////
  
  const int BOUNDARY = 12;   // (in) Avoid objects closer than 10 inches.
  const int PINGPIN = 24;  // Ping ultrasonic sensor on pin D24.
  long duration, inches, cm; // Define variables for Ping sensor
  int centerDist, leftDist, rightDist, backDist; 
  
///////////////////SERVOS/////////////////////////////////////////////////////////////
  
  Servo SERVO; //Part of Servo.h Library
  
/////////////TRANSISTOR ON/OFFs///////////////////////////////////////////////////////
  
  const int SOUND = 22;      // Set high to play sound from recorder
  const int BACKLIGHT = 23;  // set high to turn on lcd backlight.
  
/////////////MOTOR SHEILD CONSTANTS///////////////////////////////////////////////////
  
  const int DIR_A = 12;     // Left motor direction Ch.A
  const int PWM_A = 3;      // Left motor PWM speed Ch.A
  const int DIR_B = 13;     // Right motor direction Ch.B
  const int PWM_B = 11;     // Right motor PWM speed Ch.B
  const int BRAKE_A = 9;    // Pull high to brake motor Ch.A
  const int BRAKE_B = 8;    // Pull high to brake motor Ch.B

///////////////////BUMP SWITCHES X4///////////////////////////////////////////////////

  const int BUMP_FR = 27; // Front Right Bumper Switch
  const int BUMP_FL = 26; // Front left Bumper Switch 
  const int BUMP_BR = 29; // back right bumper switch 
  const int BUMP_BL = 28; // back left bumper switch
  
  int val_FR = 0; // used to store the state of Front Right Bumper Switch
  int val_FL = 0;
  int val_BR = 0;
  int val_BL = 0;

///////////////////LCD PINS///////////////////////////////////////////////////////////
  
  LiquidCrystal lcd(30, 31, 32, 33, 34, 35); //Part of LiquidCrystal.h
  
//////////////////SETUP///////////////////////////////////////////////////////////////

void setup()                                                 
{
  pinMode(DIR_A, OUTPUT);  //Set control pins to be outputs
  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(BRAKE_A, OUTPUT);
  pinMode(BRAKE_B, OUTPUT);
  pinMode(SOUND, OUTPUT);
  pinMode(BACKLIGHT, OUTPUT);
  
  pinMode(BUMP_FR, INPUT); //set bump switch as an input
  pinMode(BUMP_FL, INPUT); //set bump switch as an input
  pinMode(BUMP_BR, INPUT); //set bump switch as an input
  pinMode(BUMP_BL, INPUT); //set bump switch as an input
  
  digitalWrite(SOUND, LOW); //not really nessisary but not a bad idea.
  digitalWrite(BACKLIGHT, LOW);
  
  //Part of Servo.h Library
  
  SERVO.attach(25);  //Add a servo on digital pin D25
  SERVO.write(93);  //Center sensor pan servo (adjusted to 93)
  
////////////////LCD BOOT SEQUENCE////////////////////////////////////////////////////
  
  //cosmetic flicker sequence and wait time(delays motion)
  lcd.begin(16, 2);  // set up the LCD's number of columns and rows:
  lcd.clear();
  lcd.print("    RISK-BOT");
  lcd.setCursor(0, 1);
  lcd.print("  INITIALIZING");
  digitalWrite(BACKLIGHT, HIGH);
  delay(500);
  digitalWrite(BACKLIGHT, LOW);
  delay(500);
  digitalWrite(BACKLIGHT, HIGH);
  delay(500);
  digitalWrite(BACKLIGHT, LOW);
  delay(500);
  digitalWrite(BACKLIGHT, HIGH);
  delay(500);
  digitalWrite(BACKLIGHT, LOW);
  delay(500); 
  digitalWrite(BACKLIGHT, HIGH); //lcd backlight stays on from this point
  delay(500);
  LookAround();
  lcd.clear();

///////////////////SERIAL INIT///////////////////////////////////////////////////
  
  Serial.begin(9600);  // initialize the serial communications:
  
//////////////MOTOR SPEED PWM CONSTANTS (180 KEEPS MOTORS BELOW 2 AMPS)//////////
  
  analogWrite(PWM_A, 255);  //motor speed
  analogWrite(PWM_B, 255);  //motor speed
}

/////////////////MAIN LOOP///////////////////////////////////////////////////////////////
void loop()                     

{    

  LookAhead();
  
  if(inches >= BOUNDARY) /* If the inches in front of an object is greater than or equal to BOUNDARY react*/
  {
    forward(); // All wheels forward
      //-----------------------------------------------------------------  
      val_FR = digitalRead(BUMP_FR); //read input value and store it
      if (val_FR == HIGH)
      {
        brake();
        detection();
        backward();
        turnLeft();
      }else
      //------------------------------------------------------------------
      val_FL = digitalRead(BUMP_FL); //read input value and store it
      if (val_FL == HIGH)
      {
        brake();
        detection();
        backward();
        turnRight();
      //------------------------------------------------------------------
      }else
    delay(90); // Wait 0.11 seconds (adjust for faster ping interval)
  } else // If not:

  {
    brake();
    detection();
    LookAround();
     //-----------------------------------------------------------------  
      val_BR = digitalRead(BUMP_BR); //read input value and store it
      if (val_BR == HIGH)
      {
        brake();
        detection();
        forward_timed();
        turnRight();
      }else
      //------------------------------------------------------------------
      val_BL = digitalRead(BUMP_BL); //read input value and store it
      if (val_BL == HIGH)
      {
        brake();
        detection();
        forward_timed();
        turnLeft();
      //------------------------------------------------------------------
      }else
    if(rightDist > leftDist) // If the right distance is greater than the left distance , turn right
    {
      turnRight();
    }else if (leftDist > rightDist) // If the left distance is greater than the right distance , turn left
    {
      turnLeft();
    }else if (leftDist&&rightDist&&centerDist < BOUNDARY) // If the left and right distance is smaller than BOUNDARY, back up
    {
      backward();
      
      LookAround();
    }
  }
}

////////////////END MAIN LOOP////////////////////////////////////////////////////////////

/////////////////FORWARD/////////////////////////////////////////////////////////////////
void forward()
{
  digitalWrite(BRAKE_A, LOW);
  digitalWrite(BRAKE_B, LOW);
  digitalWrite(DIR_A, HIGH);     // Left motor forward.             
  digitalWrite(DIR_B, HIGH);     // Right motor forward.
  lcd.clear();
  lcd.print("    FORWARD");
}  
/////////////////////////////////////////////////////////////////////////////////////////
void forward_timed()
{
  lcd.clear();
  lcd.print("    SCOOT");
  digitalWrite(BRAKE_A, LOW);
  digitalWrite(BRAKE_B, LOW);
  digitalWrite(DIR_A, HIGH);     // Left motor forward.             
  digitalWrite(DIR_B, HIGH);     // Right motor forward.
  delay(200); // scoot forward
  brake();
}  
/////////////////////////////////////////////////////////////////////////////////////////
void backward()
{
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("    BACK UP!");
  digitalWrite(BRAKE_A, LOW);
  digitalWrite(BRAKE_B, LOW);
  digitalWrite(DIR_A, LOW);     // Left motor backward.             
  digitalWrite(DIR_B, LOW);     // Right motor forward.
  delay(300); // time to backup
  brake();
}
/////////////////////////////////////////////////////////////////////////////////////////
void brake() //brakes both motor channels.
{
  digitalWrite(BRAKE_A, HIGH);
  digitalWrite(BRAKE_B, HIGH);
  lcd.clear();
  lcd.print("     BRAKE");
  delay(500);
}
////////////////////////////////////////////////////////////////////////////////////////
void turnRight() //(int duration)// Amount of turning is determined by duration argument (ms).
{
  lcd.clear();
  lcd.print("           RIGHT");
  lcd.setCursor(0, 1);
  lcd.print("            TURN");
  digitalWrite(BRAKE_A, LOW);
  digitalWrite(BRAKE_B, LOW);
  digitalWrite(DIR_A, LOW);     // Left motor forward.             
  digitalWrite(DIR_B, HIGH);    // Right motor backward.
  delay(500);                   // Turning time (ms).
}
//////////////////////////////////////////////////////////////////////////////////////////
void turnLeft()
{
  lcd.clear();
  lcd.print("LEFT");
  lcd.setCursor(0, 1);
  lcd.print("TURN");
  digitalWrite(BRAKE_A, LOW);
  digitalWrite(BRAKE_B, LOW);
  digitalWrite(DIR_A, HIGH);    // Right motor forward.             
  digitalWrite(DIR_B, LOW);     // Left motor backward.
  delay(500);                   // Turning time (ms).
}
//////////////////////////////////////////////////////////////////////////////////////////
void detection() // Pause play sound run LCD text sequence
{
  lcd.clear();
  lcd.print("    OBSTACLE");
  lcd.setCursor(0, 1);
  lcd.print("    DETECTED");
  digitalWrite(SOUND, HIGH);
  delay(50);
  digitalWrite(SOUND, LOW);
  delay(1100);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LookAhead() 
{
  SERVO.write(93); // angle to look forward (adjusted to 93)
  //delay(100); // wait 0.100 seconds (affects ping))) rate, decrease if bot is moving fast)
  GetDistance();
  
}
/////////////////////////////////////////////////////////////////////////////////////////
void LookAround()
{
  SERVO.write(180); // left 180° angle
  delay(600); // time to travel(may need to be adjusted for different servos)
  GetDistance(); //Left
  leftDist = inches; //get the left distance
  SERVO.write(2); // right look to the other side (adjusted)
  delay(1000); // time to travel(may need to be adjusted for different servos)
  GetDistance(); //Right
  rightDist = inches; // get the right distance
  SERVO.write(93); // center 90° angle(adjusted to 93)
  delay(500); // time to travel(may need to be adjusted for different servos)
  GetDistance(); //Center
  centerDist = inches; // store inches
  

  // Prints a line in the serial monitor
  Serial.print("RightDist: ");
  Serial.println(rightDist);
  Serial.print("LeftDist: ");
  Serial.println(leftDist);
  Serial.print("CenterDist: ");
  Serial.println(centerDist);
}
///////////////PING))) SUBROUTINE////////////////////////////////////////////////////////

// ping
// Take a distance reading from Ping ultrasonic rangefinder.
// from http://arduino.cc/en/Tutorial/Ping?from=Tutorial.UltrasoundSensor
 
unsigned long GetDistance()
{
  // The Ping is triggered by a HIGH pulse of 2 or more microseconds.
  // We give a short LOW pulse beforehand to ensure a clean HIGH pulse.
  pinMode(PINGPIN, OUTPUT);
  digitalWrite(PINGPIN, LOW);
  delayMicroseconds(2);
  digitalWrite(PINGPIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(PINGPIN, LOW);

  // The same pin is used to read the signal from the Ping: a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off an object.
  pinMode(PINGPIN, INPUT);
  duration = pulseIn(PINGPIN, HIGH);

  // Convert the time into a distance.
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
}
  long microsecondsToInches(long microseconds) // converts time to a distance
{
  return microseconds / 74 / 2;
}
  long microsecondsToCentimeters(long microseconds) // converts time to a distance
{
  return microseconds / 29 / 2;
}
////////////////////////////////////////////////////////////////////////////////////
