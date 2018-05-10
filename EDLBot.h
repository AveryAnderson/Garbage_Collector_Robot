/*
ECEN 2270
Motor Control Library

Avery Anderson, Mark Hinkle, Noa Margalit
*/

///// define pins
const int pinON = 4;         // connect pin 13 to ON/OFF switch, active HIGH

const int pinCW_Left = 7;    // connect pin 7 to clock-wise PMOS gate
const int pinCC_Left = 8;    // connect pin 8 to counter-clock-wise PMOS gate
const int pinSpeed_Left = 5; // connect pin 9 to speed reference

const int pinCC_Right = A1;    // connect pin 12 to counter-clock-wise PMOS gate
const int pinCW_Right = A0;    // connect pin 11 to clock-wise PMOS gate
const int pinSpeed_Right = 6; // connect pin 10 to speed reference

/* Calculations for converting feet to encoder pulses
const float pulse_dist = 0.53;
const float feet_to_mm = 304.8;
  
  therefore 2 feet is equivalent to 609.6 mm
  which isequivalent to 1150.189 encoder pulses   
  Note the actual number we found through experiment (850) is lower due to the inertia of the wheel
*/
const int two_feet = 850;
/*  Calculations for converting degrees to wheel rotations
 *  
 *  "Axel" Diameter: 26.36 cm in_edge, 30.71 cm out_edge
 *  "Axel" Radius:   13.18 cm,         15.355 cm
 *  Turning Area Circumference: 2 * pi * 14.2675 = 89.65 cm
 *  
 *  To rotate 180 degrees, each wheel must travel half of this circumference 
 *  Therefore rotating 180 degrees correlates to moving 44.8227 cm for each wheel
 *  
 *  After conversion to encoder pulses we see that this means 845.71 encoder pulses
 *  Note the actual number we found through experiment (740) is lower due to the inertia of the wheel
 */
const int rot_180_dist = 740;

///// encoder counter variable 
volatile int enc_count_Left = 0;   // "volatile" means the variable is not stored in a cache
volatile int enc_count_Right = 0;  // "volatile" means the variable is not stored in a cache

///// Function definitions of our ISR Handlers.  Each one simple increments the global counter
void count_Left(){
  enc_count_Left++;
}
void count_Right(){
  enc_count_Right++;
}

//This function will move the robot forward a set number of encoder pulses, at the written speed
int forward(const int distance, const int spd)   {
  analogWrite (pinSpeed_Left,  spd );
  analogWrite (pinSpeed_Right, spd );
  
  digitalWrite(pinCW_Left,  HIGH); // go clockwise
  digitalWrite(pinCW_Right, HIGH); // go clockwise
  digitalWrite(pinCC_Left,   LOW); // dont go counterclockwise
  digitalWrite(pinCC_Right,  LOW); // dont go counterclockwise

  //The left wheel is naturally faster than our right in our robot, to account for this we hard code
  //A ten unit increase in the speed we pass to our wheels
  int leftSpd = spd;
  int rightSpd = spd + 10;

  //In this while loop we control the movement of each wheel independantly to ensure we get to the desired distance
  do {
    
    //If the wheel has reached its max distance, turn it off. Else turn it on at the appropriate speed
    if((enc_count_Right < distance)){
      analogWrite (pinSpeed_Right, rightSpd );
    }
    else{
      analogWrite (pinSpeed_Right, 0 );
    }
    
    if((enc_count_Left < distance)){
    analogWrite (pinSpeed_Left,  leftSpd  );
    }
    else{
      analogWrite (pinSpeed_Left, 0 );
    }
    
    
  } while((enc_count_Right < distance) || (enc_count_Left < distance)); //wait until the proper distance has been reached by both wheels

  digitalWrite(pinCW_Left,  LOW); // dont go clockwise
  digitalWrite(pinCW_Right, LOW); // dont go clockwise 
  enc_count_Left  = 0;
  enc_count_Right = 0;
}

//This function will move the robot forward a set number of encoder pulses, at the written speed
int reverse(const int distance, const int spd)   {
  analogWrite (pinSpeed_Left,  spd);
  analogWrite (pinSpeed_Right, spd);
  
  digitalWrite(pinCW_Left,   LOW); // dont go clockwise
  digitalWrite(pinCW_Right,  LOW); // dont go clockwise
  digitalWrite(pinCC_Left,  HIGH); // go counterclockwise
  digitalWrite(pinCC_Right, HIGH); // go counterclockwise

  //The left wheel is naturally faster than our right in our robot, to account for this we hard code
  //A ten unit increase in the speed we pass to our wheels
  int leftSpd = spd;
  int rightSpd = spd + 10;//Increase the right speed artificially as the wheel is slower

  //In this while loop we control the movement of each wheel independantly to ensure we get to the desired distance
  do {

    //If the wheel has reached its max distance, turn it off. Else turn it on at the appropriate speed
    if((enc_count_Right < distance)){
      analogWrite (pinSpeed_Right, rightSpd );
    }
    else{
      analogWrite (pinSpeed_Right, 0 );
    }
    
    if((enc_count_Left < distance)){
    analogWrite (pinSpeed_Left,  leftSpd  );
    }
    else{
      analogWrite (pinSpeed_Left, 0 );
    }
    
  } while((enc_count_Right < distance) || (enc_count_Left < distance)); //wait until the proper distance has been reached by both wheels

  digitalWrite(pinCC_Left,  LOW); // dont go clockwise
  digitalWrite(pinCC_Right, LOW); // dont go clockwise
 
  enc_count_Left  = 0;
  enc_count_Right = 0;
}

//In this function we tell the wheel to turn by passing the number of encoder pulses that correspond to the turn amount we desire, and the speed with which we turn
int turn_left (const int deg, const int spd)  {
  analogWrite (pinSpeed_Left,  spd);
  analogWrite (pinSpeed_Right, spd);
  
  digitalWrite(pinCW_Right, HIGH); // go clockwise - right wheel
  digitalWrite(pinCC_Left,  HIGH); // go counterclockwise - left wheel
  digitalWrite(pinCC_Right,  LOW); // dont go counterclockwise - right wheel
  digitalWrite(pinCW_Left,   LOW); // dont go clockwise - left wheel

  enc_count_Left  = 0;
  enc_count_Right = 0;
  
  do {    
    } while((enc_count_Right < deg) && (enc_count_Left < deg)); //wait until the proper distance has been traveled

  digitalWrite(pinCW_Right, LOW); // stop going clockwise - right wheel
  digitalWrite(pinCC_Left,  LOW); // stop going counterclockwise - left wheel
  
  analogWrite (pinSpeed_Left,  0);
  analogWrite (pinSpeed_Right, 0);
  enc_count_Left  = 0;
  enc_count_Right = 0;
}

//In this function we tell the wheel to turn by passing the number of encoder pulses that correspond to the turn amount we desire, and the speed with which we turn
int turn_right(const int deg, const int spd)  {
  analogWrite (pinSpeed_Left,  spd);
  analogWrite (pinSpeed_Right, spd);
  
  digitalWrite(pinCW_Right,  LOW); // dont go clockwise - right wheel
  digitalWrite(pinCC_Left,   LOW); // dont go counterclockwise - left wheel
  digitalWrite(pinCC_Right, HIGH); // go counterclockwise - right wheel
  digitalWrite(pinCW_Left,  HIGH); // go clockwise - left wheel

  enc_count_Left  = 0;
  enc_count_Right = 0;

  do {    
    } while((enc_count_Right < deg) && (enc_count_Left < deg)); //wait until the proper distance has been traveled

  digitalWrite(pinCC_Right, LOW); // stop going counterclockwise - right wheel  
  digitalWrite(pinCW_Left,  LOW); // stop going clockwise - left wheel
  enc_count_Left  = 0;
  enc_count_Right = 0;
}

void demo(){
  digitalWrite(13,HIGH);                // turn LED on
  forward   (two_feet,    100);         // move forward 2 feet
  digitalWrite(13,LOW);                 // turn LED off

  delay(100);                           // wait 1 second
  
  turn_right(rot_180_dist, 50);         // rotate clockwise 180 degrees

  delay(100);
  
  digitalWrite(13,HIGH);                // turn LED on
  forward   (two_feet,    100);         // move forward 2 feet
  digitalWrite(13,LOW);                 // turn LED off

  delay(100);
  
  turn_left (rot_180_dist-20, 50);      // rotate counterclockwise 180 degrees
  
  
  digitalWrite(13,LOW);                 // turn LED off

  delay(1000);                          // wait 3 seconds
  
}

//This is the function for part A4 of the lab.  It turns the robot 360 degrees in each direction
void part_A_360(){
  //Turn right 360 degrees.  We proved 180 above, 360 is approximately 2* that value
  turn_right(500,100);
  delay(1000);
  //Turn left 360 degrees
  turn_left(1400,100);
  delay(1000);
}

//This is the function for the pulse display
void pulseCounter(){
  //Turn off both of the wheels
  digitalWrite(pinCW_Left,   LOW); // stop clockwise
  digitalWrite(pinCC_Left,   LOW); // stop counter-clockwise
  digitalWrite(pinCW_Right,  LOW); // stop clockwise
  digitalWrite(pinCC_Right,  LOW); // stop counter-clockwise
  delay(1000);                     // wait 1 second


  //Reset the counters
  enc_count_Left  = 0;
  enc_count_Right = 0;

  //Set both of the speed references to max speed of around 3.6 => half speed of 92
  analogWrite (pinSpeed_Right, 92); 
  analogWrite (pinSpeed_Left,  92); 

  //Set both wheels to turn Clockwise
  digitalWrite(pinCW_Left,  HIGH); 
  digitalWrite(pinCW_Right, HIGH); 
  delay(1000);

  //Print the wheel values
  Serial.print("LEFT: ");
  Serial.print(enc_count_Left);
  Serial.print("  RIGHT: ");
  Serial.print(enc_count_Right);
  Serial.print("  DIFF ");
  Serial.println((enc_count_Left - enc_count_Right));  
}

void botInit(){
  pinMode(pinON,           INPUT);
  pinMode(pinCW_Left,     OUTPUT);
  pinMode(pinCC_Left,     OUTPUT);
  pinMode(pinSpeed_Left,  OUTPUT);
  pinMode(pinCW_Right,    OUTPUT);
  pinMode(pinCC_Right,    OUTPUT);
  pinMode(pinSpeed_Right, OUTPUT);
  
  digitalWrite(pinCW_Left,   LOW); // stop clockwise
  digitalWrite(pinCC_Left,   LOW); // stop counter-clockwise
  analogWrite (pinSpeed_Left,  0); // set speed reference
  digitalWrite(pinCW_Right,  LOW); // stop clockwise
  digitalWrite(pinCC_Right,  LOW); // stop counter-clockwise
  analogWrite (pinSpeed_Right, 0); // set speed reference

  attachInterrupt(0, count_Left,  RISING); // pin 2
  attachInterrupt(1, count_Right, RISING); // pin 3
}
