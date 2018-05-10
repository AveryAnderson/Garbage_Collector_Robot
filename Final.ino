
/*
 * ECEN 2270
 * Final Project:
 * Garbage Collector Robot

Avery Anderson, Mark Hinkle, Noa Margalit
*/


/*The primary functions to use from this header file will be 
 * EncoderDistance-> distance divided by 0.53 meters
 * Speed is an integer between 0 and 184
 * 360 degree rotation is around 1400 pulses
 * 180 degree rotation is around  740 pulses
 *    forward(encoderNumber,speed)
 *    reverse(encoderNumber,speed)
 *    turn_left(encoderNumber,speed)
 *    turn_right(encoderNumber,speed)
 * There are other functions, but they were for a previous lab
 */
  #include "EDLBot.h"

 /* Communication with the PIXYCam is controlled by the SPI
  * and Pixy libraries which we include below
  */
  #include <SPI.h>  
  #include <Pixy.h>

/*
 * The servos will be controlled by the servo library
 */
  #include <Servo.h>

 /*  These numbers will define the "Good Region" parameters on the robot
  *  The DENOM shows what fraction of the screen is indicated, while
  *  UPPER and LOWER indicate the upper and lower bounds of the image
  *  Note DENOM needs to be an odd integer or the defined region will have
  *  zero width.
  */
  #define DENOM  5
  #define LOWER ((DENOM/2)*(PIXY_MAX_X/DENOM))
  #define UPPER (PIXY_MAX_X - LOWER)

  //We never want to rotate more than 6 degrees, or approximately 30 pulses
  #define SIX_DEGREES  40

  //Define the servo pins
  #define LEFT_SERVO   10
  #define RIGHT_SERVO  9

  //Define echo and trigger pins for the proximity sensor
  const int echo = A5;    // connect pin 12 to counter-clock-wise PMOS gate
  const int trig = A4;    // connect pin 11 to clock-wise PMOS gate

//Declare an integer outside of the scope of this function.
//This will control whether we pivot left or right while waiting 
//For a block in our hold while loop
volatile int pivot = 0;
  
// This is the main Pixy object 
Pixy pixy;

//This is the main Servo object
Servo leftArm;
Servo rightArm;

//The commands LIFO. We will be indexing this array using the commandIndex global variable
char commands[100] = {0};
int commandIndex = 0;

//This function adds a command to the LIFO.  If we overflow the lifo we just stop storing the new commands
//The recognized commands will be 'f' for forwards 'l' for left' and 'r' for right
void addCommand(char com){
  //If we still have room to store commands, AND we have a valid command, store it in the current commandIndex
  if((commandIndex < 100) && ((com == 'r') || (com == 'f') || (com == 'l'))){
    commands[commandIndex] = com;
    commandIndex ++;
  }
}

//This function pops items from the LIFO and does the opposite command until we arrive at the original space
//Undo every command in the array until I arrive at my original starting space
void undoCommands(){
  while(commandIndex > 0){
    commandIndex --;
    char currentCommand = commands[commandIndex];
    if(currentCommand == 'r'){
      turn_left(SIX_DEGREES,40);
    }
    else if (currentCommand == 'l'){
      turn_right(SIX_DEGREES,40);
    }
    else if(currentCommand == 'f'){
      reverse(200,80);
    }
  }
  
}

///// setup pins and initial values
void setup() {
  //Initialize Robot
  botInit();
  //Initialize PixyCam
  pixy.init();
  //Attach the two servo pins
  leftArm.attach(LEFT_SERVO);
  rightArm.attach(RIGHT_SERVO);

  //Initialize the pins used by the distance sensor
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  //Initialize the servos to be open
  openServos();
}

//This function will hold the processor until we find an object to track
void pan(){
  //While we are not detecting any objects, pivot back and forth. We will use a
  //global variable and the two modulus to alternate pivot directions
  do
  {
    if(pivot%2){
     turn_right(150,50);
    }
    else{
      turn_left(150,50);
    }
    delay(500);
    pivot++;
  } while(!(pixy.getBlocks()));
  delay(500);
}

//This function will hold the processor until we rotate the object into the 
//center of the frame.  If the detected object disappears, exit the loop and 
//return -1.  Otherwise return 1 when we find the object
int center(){  
   //Enter an infinite loop
   while(1){
      delay(200);
      //If we find that there are no blocks, there must have been a mistake return -1
      if(!pixy.getBlocks()){
        return 0;
      }
      
      //Otherwise begin processing data
      //Now that we have recognized at least one item, we want to center in on it
      //Name the first object in our list as the focus

      Block focus = pixy.blocks[0];
  
      //Now we want to rotate so the robot ends up with the object in the center of the frame
      //If the x value is below our lower bound, we need to turn left to bring it in focus
      if(focus.x < LOWER){        
        //If we just turned right, and now need to turn left, it means we overshot. In this instance it
        //is best to just move forwards anyway as our angle will never capture the object.
        if(commands[commandIndex - 1] == 'r'){
          return 1;
        }
        //Otherwise store in the LIFO that we just turned left, and turn to the left
        addCommand('l');
        turn_left(SIX_DEGREES,40);
      }
      //repeat the same process if we need to rotate to the right
      else if(focus.x > UPPER){
        if(commands[commandIndex - 1] == 'l'){
          return 1;
        }
        addCommand('r');
        turn_right(SIX_DEGREES,40);
      }
      //If we have still not entered an if loop, we must be lined up, return zero to indicate success
      else{
        return 1;
      }
    }
}

//This function checks the proximity sensor on the front of the robot in order to decide whether or not it
//is close enough to an object to try and grab it.
boolean inRange(){
  
  long duration, distance;
  digitalWrite(trig, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trig, HIGH);

  delayMicroseconds(10); // Added this line
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  distance = (duration/2) / 29.1;
  return (distance < 5)? true : false ;
}

//This program moves forward half a foot at a time, continually reasserting that it is focused on the object
//It will continue moving forward until it knows it has found an object
int approach(){
  commandIndex = 0;
  while(!inRange()){
    if(!center()) return 0;

    //If I am already in range and was misaligned, I would be able to see that now.  Check range again.
    if(inRange()){
      return 1;
    }
    
    //Store a command indicating that we are moving forward
    addCommand('f');
    forward(200,80);
    delay(20);
    center();
  }
  return 1;
}

/* The servos are controlled by a a pwm wave to set their open and closed states.  In order to drive them
 * we only need to force them open or force them closed.  These operations will be undertaken by the following
 * two functions * 
 */
void closeServos(){
  leftArm.writeMicroseconds(2250);
  rightArm.writeMicroseconds(450);
}
void openServos(){
  leftArm.writeMicroseconds(1250);
  rightArm.writeMicroseconds(1500);
}

void loop() {
  do {
    enc_count_Left  = 0;                // reset encoder counter to 0
    enc_count_Right = 0;                // reset encoder counter to 0
    openServos();                       // open the servo arms
    undoCommands();                     // set the command stack to zero
  } while (digitalRead(pinON) == LOW);  // wait for ON switch

//  while(1){
//    forward(1111,50);
//  }

  //Scan 180 in front of us in order to ensure we will find an object as fast as possible
  delay(100);
  pan(); 
  
  //Attempt to approach the object.  The robot will incrementally move forward, attempt to recenter, and repeat.
  if(approach()){
    //If we succeeded in getting to the object, close the servos
    closeServos();
    //Stop listening to the pixyCam, instead just drag the object back to the starting position.
    undoCommands();
  } 
}
