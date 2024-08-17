


#include <NewPing.h>

#define TRIGGER_PINL  A3  
#define ECHO_PINL     A0  

#define MAX_DISTANCE 100 

#define TRIGGER_PINF  A4  
#define ECHO_PINF     A1  

#define TRIGGER_PINR  A5  
#define ECHO_PINR     A2  





int dir;


#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4



float P = 0.7 ;
float D = 0.5 ;
float I = 0.4 ;
float oldErrorP ;
float totalError ;
int offset = 5 ;

int wall_threshold = 13 ;
int front_threshold = 7 ;

boolean frontwall ;
boolean leftwall ;
boolean rightwall ;
boolean first_turn ;
boolean rightWallFollow ;
boolean leftWallFollow ;





int en1 = 2 ;
int en2 = 3 ;

int en3 = 4 ;
int en4 = 5 ;

int enA = 10 ;
int enB = 11 ;

int baseSpeed = 10 ;

int RMS ;
int LMS ;

int LED = 13 ;
int led1 = 8 ;
int led2 = 9 ;



NewPing sonarLeft(TRIGGER_PINL, ECHO_PINL, MAX_DISTANCE); 
NewPing sonarRight(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);
NewPing sonarFront(TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE);

unsigned int pingSpeed = 30; 
unsigned long pingTimer;    


float oldLeftSensor, oldRightSensor, leftSensor, rightSensor, frontSensor, oldFrontSensor, lSensor, rSensor, fSensor;

//int TestNUM = 1  ;



void setup() {

  Serial.begin(115200);


  for (int i = 2; i <= 13; i++) 
    pinMode(i, OUTPUT);



  first_turn = false ;
  rightWallFollow = false ;
  leftWallFollow = false ;
 

}

void loop() {




  ReadSensors();

  walls();


  if ( first_turn == false ) {

    pid_start();

  }
  else if (leftWallFollow == true ) {

    PID(true) ;

  }
  else if (rightWallFollow == true ) {
    PID(false) ;
  }


  if (leftwall == true && rightwall == false && frontwall == true ) {

    PID(false) ;

    if ( first_turn == false ) {

      first_turn = true ;
      rightWallFollow = true ;
      
      digitalWrite(led2 , LOW );
      digitalWrite(led1 ,HIGH );
    }
  }
   if (leftwall == false && rightwall == true && frontwall == true ) {

    PID(true) ;

    if ( first_turn == false ) {


      first_turn = true ;
      leftWallFollow = true ;
      digitalWrite(LED , HIGH);
       
    }
  }
   if ( leftSensor == 0 || leftSensor > 100 && rightSensor == 0 || rightSensor > 100 && frontSensor == 0 || frontSensor > 100 ) {

    setDirection(STOP);
  }




  Serial.print(" Left : ");
  Serial.print(leftSensor);
  Serial.print(" cm ");
  Serial.print(" Right : ");
  Serial.print(rightSensor);
  Serial.print(" cm ");
  Serial.print(" Front : ");
  Serial.print(frontSensor);
  Serial.println(" cm ");



  Serial.print("error=");
  Serial.println(totalError);


}





void setDirection(int dir) {

  if ( dir == FORWARD ) {
    digitalWrite(en1, LOW);  
    digitalWrite(en2, HIGH);
    digitalWrite(en3, LOW); 
    digitalWrite(en4, HIGH);
  }
  else if ( dir == LEFT ) {
    digitalWrite(en1, HIGH);  
    digitalWrite(en2, LOW );
    digitalWrite(en3, LOW ); 
    digitalWrite(en4, HIGH);
  }
  else if ( dir == RIGHT ) {
    digitalWrite(en1, LOW);   
    digitalWrite(en2, HIGH);
    digitalWrite(en3, HIGH);  
    digitalWrite(en4, LOW);
  }
  else if ( dir == STOP ) {
    digitalWrite(en1, HIGH);  
    digitalWrite(en2, HIGH );
    digitalWrite(en3, HIGH );  
    digitalWrite(en4, HIGH);
  }
  else if ( dir == BACKWARD ) {
    digitalWrite(en1, HIGH );   
    digitalWrite(en2, LOW );
    digitalWrite(en3, HIGH );  
    digitalWrite(en4, LOW );
  }
}

void ReadSensors() {



  lSensor = sonarLeft.ping_cm(); 
  rSensor = sonarRight.ping_cm();
  fSensor = sonarFront.ping_cm();


  leftSensor = (lSensor + oldLeftSensor) / 2; 
  rightSensor = (rSensor + oldRightSensor) / 2;
  frontSensor = (fSensor + oldFrontSensor) / 2;


  oldLeftSensor = leftSensor; 
  oldRightSensor = rightSensor;
  oldFrontSensor = frontSensor;

}


void pid_start() {

  //ReadSensors()

  float errorP = leftSensor - rightSensor ;
  float errorD = errorP - oldErrorP;
  float errorI = (2.0 / 3.0) * errorI + errorP ;

  totalError = P * errorP + D * errorD + I * errorI ;

  oldErrorP = errorP ;


  RMS = baseSpeed + totalError ;
  LMS = baseSpeed - totalError ;



  if (RMS < 0) {

    RMS = map(RMS , 0 , -255, 0, 255);

    analogWrite(enA , RMS);
    analogWrite(enB , LMS);

    setDirection(RIGHT);

  }
  else if (LMS < 0) {
    LMS = map(LMS , 0 , -255, 0, 255);


    analogWrite(enA , RMS);
    analogWrite(enB , LMS);

    setDirection(LEFT);
  }
  else {

    analogWrite(enA , RMS);
    analogWrite(enB , LMS);

    setDirection(FORWARD);
  }



}




void PID( boolean left ) {

  if (left == true ) {

    float errorP = leftSensor - rightSensor - offset ;
    float errorD = errorP - oldErrorP;
    float errorI = (2.0 / 3) * errorI + errorP ;


    totalError = P * errorP + D * errorD + I * errorI ;

    oldErrorP = errorP ;


    RMS = baseSpeed + totalError ;
    LMS = baseSpeed - totalError ;

    

    if (RMS < 0) {

      RMS = map(RMS , 0 , -255, 0, 255);

      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(RIGHT);

    }
    else if (LMS < 0) {
      LMS = map(LMS , 0 , -255, 0, 255);


      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(LEFT);
    }
    else {

      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(FORWARD);
    }

  }
  else {

    float errorP = leftSensor - rightSensor + offset ;
    float errorD = errorP - oldErrorP;
    float errorI = (2.0 / 3) * errorI + errorP ;

    totalError = P * errorP + D * errorD + I * errorI ;

    oldErrorP = errorP ;


    RMS = baseSpeed + totalError ;
    LMS = baseSpeed - totalError ;

   

    if (RMS < 0) {

      RMS = map(RMS , 0 , -255, 0, 255);

      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(RIGHT);

    }
    else if (LMS < 0) {
      LMS = map(LMS , 0 , -255, 0, 255);


      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(LEFT);
    }
    else {

      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(FORWARD);
    }

  }

}
`

void walls() {


  if ( leftSensor < wall_threshold ) {
    leftwall = true ;
  }
  else {
    leftwall = false ;
  }


  if ( rightSensor < wall_threshold ) {
    rightwall = true ;
  }
  else {
    rightwall = false ;


  } if ( frontSensor < front_threshold ) {
    frontwall = true ;
  }
  else {
    frontwall = false ;
  }

}




void turnright() {


  LMS = baseSpeed ;

  RMS = LMS * rightSensor / ( rightSensor + 11 ) ;


}


void turnleft() {


  RMS = baseSpeed ;

  LMS = RMS * leftSensor / ( leftSensor + 11 ) ;

}






