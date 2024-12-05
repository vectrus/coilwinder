#include "doxygen.h"
#include "NexButton.h"
#include "NexCheckbox.h"
#include "NexConfig.h"
#include "NexCrop.h"
#include "NexDualStateButton.h"
#include "NexGauge.h"
#include "NexGpio.h"
#include "NexHardware.h"
#include "NexHotspot.h"
#include "NexNumber.h"
#include "NexObject.h"
#include "NexPage.h"
#include "NexPicture.h"
#include "NexProgressBar.h"
#include "NexRadio.h"
#include "NexRtc.h"
#include "NexScrolltext.h"
#include "NexSlider.h"
#include "NexText.h"
#include "NexTimer.h"
#include "Nextion.h"
#include "NexTouch.h"
#include "NexUpload.h"
#include "NexVariable.h"
#include "NexWaveform.h"


#include "LinkedList.h"

#include "State.h"
#include "StateMachine.h"

// End of declaring objects
char buffer[100] = {0};  // This is needed only if you are going to receive a text from the display. You can remove it otherwise.

const int continueButton = 19; // Continue button

const byte leftStop = 2;
const byte rightStop = 3;


const int enaSlide = 4; // enable
const int dirSlide = 5; // direction
const int pulSlide = 6; // step

const int enaSpindle = 7; // enable
const int dirSpindle = 8; // direction
const int pulSpindle = 9; // step



const int interval1 = 800;   // interval while coiling
const int interval2 = 350;   // interval for movent
const int interval3 = 8;   // Interval for homing
   // mudanças de estado do pulso
   
// Sitched vars and such   
boolean pulse = LOW; //estado do pulso
long leftStopActiveTime = millis();
long rightStopActiveTime = millis();
long leftStopActive = false;
long rightStopActive = false;
boolean changePendingLeft = 0;
boolean changePendingRight = 0;
volatile boolean continuePressed= false;
boolean homedR = 0; 
boolean homedL = 0; 
volatile boolean pauzed = 0;

// MachineState
volatile char *currentStatus = "Starting";



// Motorsettings
int stepsPerRevolutionSlideMotor = 3200;   // Steps per revolution slide
int stepsPerRevolutionSpindleMotor = 3200; // Steps per revolution spindle

// Coil specs
// Vars
boolean coilDirection = HIGH; //HIGH - CW
long numberofTurns = 1016; // In...uh  well.... turns
long numberofTurnsDone = 0; // In...uh  well.... turns ;-)
long prevNumberofTurnsDone = 0;
long wireThickness = 500000; // (0.003mm = 3000, 0.5 = 500000) In Millimeters X 1000000 to calculate wit integers rather than floats.
long diameter = 70; // Coil diameter in Millimeters
long length = 800; // Coil length in Millimeters
long lengthOfWireNeeded = numberofTurns * (2 * 3.14159265359) * diameter / 2; // (just to know, you never know...)

// Coil calculations
long totalsteps = numberofTurns * stepsPerRevolutionSpindleMotor;
int mmPerRevSlider = 8; // The pitch of the leadscrew
int stepsPerMM = stepsPerRevolutionSlideMotor / mmPerRevSlider;
int stepsPerResolutionSpindle = stepsPerRevolutionSpindleMotor / 60; 
long distanceToCoverPerResolutionMM = ((wireThickness) / 60); // Distance in MM the slider needs to make per resolution
long intermediate = stepsPerMM * distanceToCoverPerResolutionMM;
long stepsPerResolutionSlide = intermediate / 1000000;
    


// Nextion set default values
NexButton continue_btn = NexButton(0, 6, "continue_btn");  // Cont/Start btn tekst
NexButton moveRightBtn = NexButton(2, 3, "moveright");  // Cont/Start btn tekst
NexButton moveLeftBtn = NexButton(2, 2, "moveleft");  // Cont/Start btn tekst
NexButton moveForwardBtn = NexButton(2, 5, "moveforward");  // Cont/Start btn tekst
NexButton moveBackBtn = NexButton(2, 4, "moveback");  // Cont/Start btn tekst
NexButton stopBtn = NexButton(0, 4, "stop");  //stop button / for pauzing
NexText turnsset = NexText(0, 2, "numberofTurns");  // Textbox for turnsset
NexText machinestate = NexText(0, 2, "machinestate");  // Textbox for turnsset


NexTouch *nex_listen_list[] =
{
  &continue_btn,  // Button added start
  &moveRightBtn,
  &moveLeftBtn,
  &moveForwardBtn,
  &moveBackBtn,
  &stopBtn,
  //&minutesLess,  // Checkbox added airfilter mode
  //&fanSpeedSlider,  // Checkbox added airfilter mode
  //&timerSlider,  // Checkbox added airfilter mode
  //&page0,  // Page added as a touch event
  //&page1,  // Page added as a touch event
  //&page2,  // Page added as a touch event
  //&page3,  // Page added as a touch event
  //&page4,  // Page added as a touch event
  //&page5,  // Page added as a touch event
  NULL  // String terminated
};  // End of touch event list


// StateMachine
const int STATE_DELAY = 3;
int randomState = 0;
const int LED = 13;

StateMachine machine = StateMachine();

/*
 * Example of using a lambda (or anonymous function) callback
 * instead of providing the address of an existing function.
 * Also example of using the attribute executeOnce to execute
 * some part of the code only once per state.
 */
State* S0 = machine.addState([](){
  Serial.println("State 0, anonymous function");
  if(machine.executeOnce){
    homedL = 0; // Since we dont know where we are now.
    homedR = 0; // Since we dont know where we are now.
    Serial.println("Execute Once");
    digitalWrite(LED,!digitalRead(LED));
  }
});;

// Machine State
State* S1 = machine.addState(&state1); // Homing (right)
State* S2 = machine.addState(&state2); // Homing (left)
State* S3 = machine.addState(&state3); // Roll Right
State* S4 = machine.addState(&state4); // Slide right 
State* S5 = machine.addState(&state5); // Run coil


void setup()
{
    //Start Serial output for dev purposes;
    Serial.begin(115200);
    Serial3.begin(9600);

    
    /// Nextion stuff
    nexInit(115200); 
    
    // Assign Nextion Serial calls to their callbacks
    continue_btn.attachPop(continue_btnPopCallback, &continue_btn);  // Dual state button bt0 release
    moveRightBtn.attachPop(moveRightBtnPopCallback, &moveRightBtn);  // Move Slider right
    moveLeftBtn.attachPop(moveLeftBtnPopCallback, &moveLeftBtn);  // Move Slider Left
    moveForwardBtn.attachPop(moveForwardBtnPopCallback, &moveForwardBtn);  // Move spindle forward
    moveBackBtn.attachPop(moveBackBtnPopCallback, &moveBackBtn);  // Move Spindel right
    moveBackBtn.attachPop(moveBackBtnPopCallback, &moveBackBtn);  // Move Spindel right
    stopBtn.attachPop(stopBtnPopCallback, &stopBtn);  // Pauze/stop and goto manual

  
  
    pinMode(LED,OUTPUT);
    randomSeed(A0);
    /*
    * Example of a transition that uses a lambda
    * function, and sets the transition (first one)
    * to a random state.
    * 
    * Add only one transition(index=0)
    * that points to randomly selected states
    * Initially points to itself.
    */
    
   

    /*
    * The other way to define transitions.
    * (Looks cleaner)
    * Functions must be defined in the sketch
    */
    S0->addTransition(&transitionS0S1,S1);
    S1->addTransition(&transitionS1S2,S2);
    S2->addTransition(&transitionS2S3,S3);
    S3->addTransition(&transitionS3S4,S4);
    S4->addTransition(&transitionS4S5,S5);
    S5->addTransition(&transitionS5S4,S4);
    //S5->addTransition(&transitionS5S2,S2);

    
    // Enable pins for endstops
    // Endstops put a grnd on the inputs, normaly high
    pinMode(leftStop, INPUT_PULLUP);
    pinMode(rightStop, INPUT_PULLUP);
    
    //pinMode(continueButton, INPUT_PULLUP); // Continue button
    //attachInterrupt(digitalPinToInterrupt(continueButton), continue_btnPopCallback, FALLING); // Interupt continubutton
    attachInterrupt(digitalPinToInterrupt(leftStop), triggerEndStopLeft, FALLING); // Interupt for left
    attachInterrupt(digitalPinToInterrupt(rightStop), triggerEndStopRight, FALLING); // Interupt for right
    
    
    
    
    // Enable pins for SlideMotor
    pinMode(enaSlide, OUTPUT);
    pinMode(dirSlide, OUTPUT);
    pinMode(pulSlide, OUTPUT);
    digitalWrite(enaSlide, LOW); // turn on/off
    digitalWrite(dirSlide, HIGH); // direction low CW / high CCW 
    digitalWrite(pulSlide, HIGH); // do a pulse, not a step, a pulse! depending on the settings of the driver.

    // Enable pins for SpindleMotor
    pinMode(enaSpindle, OUTPUT);
    pinMode(dirSpindle, OUTPUT);
    pinMode(pulSpindle, OUTPUT);
    digitalWrite(enaSpindle, LOW);  // turn on/off
    digitalWrite(dirSpindle, HIGH); // direction low CW / high CCW 
    digitalWrite(pulSpindle, HIGH); // do a pulse
}
 
void loop(void)
{
  

    if(leftStopActive == true) { // If the right endstop has been made active by interupt
        handleEndStopLeft();
    }
    if(rightStopActive == true) { // If the right endstop has been made actove by interupt
        handleEndStopRight();
    }
    
    //runSlideMotor();
    machine.run();
   // delay(STATE_DELAY);
    
    
    updateNextionScreen(currentStatus);
    
    // Do nextion stuff
    Serial3.print("page0.turnsset.txt=\"");  
    Serial3.print(numberofTurns);
    Serial3.print(" \"");
    Serial3.write(0xff);  
    Serial3.write(0xff);
    Serial3.write(0xff);
    
    Serial3.print("page0.turnsdone.txt=\"");  
    Serial3.print(numberofTurnsDone);
    Serial3.print(" \"");
    Serial3.write(0xff);  
    Serial3.write(0xff);
    Serial3.write(0xff);
    
    nexLoop(nex_listen_list);  // Check for any touch event
}


void updateNextionScreen(char *currentStatus) {
    // Status
    
    
    Serial3.print("page0.machinestate.txt=\"");  
    Serial3.print(currentStatus);
    Serial3.print(" \"");
    Serial3.write(0xff);  
    Serial3.write(0xff);
    Serial3.write(0xff);

}


bool transitionS0S1(){
    currentStatus = "Press cont.";

  if(continuePressed == true) {
            continuePressed = false;
            return true;
        }
        else {
            return false;
        }
}
bool transitionS1S2(){
   
    currentStatus = "Home right.";
    if(digitalRead(rightStop) == LOW) {
        //Serial.println("We are at the rightposition, moving on. Homing left");
        homedR = 1;
        homedL = 0;
        currentStatus = "Start home left.";
        return true;
        
    }
 
}
bool transitionS2S3(){
    
 currentStatus = "Home left.";
    if(digitalRead(leftStop) == LOW) {
        homedR = 1;
        homedL = 1;
        // machine.transitionTo(S3);
        currentStatus = "Done homing.";
        return true;
    }
    
 
    
  
}
bool transitionS3S4(){
    currentStatus = "Press cont. S3";
    if(homedR == 1 && homedL == 1){
        if(continuePressed == true) {
            continuePressed = false;
            delay(5);
            return true;
        }
        else {
            return false;
        }
        
    }
    
  
}
bool transitionS4S5(){
    currentStatus = "Manual";
    if(pauzed == 1) {
        currentStatus = "Pauzed. Cont.";
    }
    if(continuePressed == true) {
            continuePressed = false;
             delay(5);
            
            return true;
        }
        else {
            return false;
        }
    
  
}

bool transitionS5S4() {
    currentStatus = "Coiling.";
    if(continuePressed == true) {
            continuePressed = false;
            currentStatus = "Set to manual";
             delay(5);
            return true;
        }
        else if(pauzed == 1) {
            continuePressed = false;
             delay(5);
            currentStatus = "Set to manual";
            return true;
        }
        else {
            return false;
        }
   
}

void state1() {  // Home the slider to the right.
    if(homedR == 0) {
        Serial.println("Homing right");
        
        while(digitalRead(rightStop) != LOW) {
            digitalWrite(dirSlide, LOW); // low CW (right) / high CCW (left)
            runSlideMotorFast(1);
        }
        
        
    }
    
    
   
    

}

void state2() {  // Home the slider to the left.
     if(homedL == 0) {
        Serial.println("Homing left");
        
        while(digitalRead(leftStop) != LOW) {
            digitalWrite(dirSlide, HIGH); // low CW (right) / high CCW (left)
            runSlideMotorFast(1);
        }
        
        
    }
}

void state3() {  // Move away from an endstop
     if(machine.executeOnce){
        continuePressed = false;
        delay(5);
    }
   
    if(millis() - leftStopActiveTime > 2000) {
        if(machine.executeOnce){
            if(digitalRead(leftStop) == LOW ) {
                digitalWrite(dirSlide, LOW); // low CW (right) / high CCW (left)
                runSlideMotor(stepsPerRevolutionSlideMotor, interval2);
            }
            else if(digitalRead(rightStop) == LOW ) {
                digitalWrite(dirSlide, HIGH); // low CW (right) / high CCW (left)
                runSlideMotor(leftStopActiveTime, runSlideMotor);
            }
        }
    }
    
    //transitionS3S4();
    
    
    
    

}
void state4() {  // Manual Mode
    //continuePressed = false;
    
    if(machine.executeOnce){
        continuePressed = false;
    }
  
    // In this state the manual move buttons on the display are active
    
    
    

}
void state5() {  // Wiggle with pride
    currentStatus = "Coiling";
    updateNextionScreen(currentStatus);
    int stopped = false;
    
    homedR = 0;
    homedL = 0;
    
    
    
    // Set motor directions
    digitalWrite(dirSlide, LOW); // low CW (right) / high CCW (left)
    digitalWrite(dirSpindle, coilDirection);
    continuePressed = false;   
    stopped = false;
    pauzed = false;
    long i;
    if(pauzed == 1) {
        totalsteps = totalsteps - prevNumberofTurnsDone;
        pauzed = 0;
    }
    
    
    for(i=0;i<totalsteps;i++) {
        //if(continuePressed == true) {
        //    break;
        //}

        long j;
        
        for(j=0;j<60;j++) {
            // If any of the switches go LOW stop motors and go to S1 / &state1
            if( digitalRead(rightStop) == LOW || digitalRead(leftStop) == LOW  || pauzed == 1) {
                Serial.println("Stop triggered");
                continuePressed = false; 
                stopSlideMotor();
                stopSpindleMotor();
                delay(5000);
                stopped = true;
                machine.transitionTo(S4);
                break;
            }
            else {
                
                
                runSpindleMotor(stepsPerResolutionSpindle, interval1);
                if(stepsPerResolutionSlide < 2) {
                    stepsPerResolutionSlide = 2;
                }
                runSlideMotor(stepsPerResolutionSlide, interval1);
               
                
                numberofTurnsDone = i + prevNumberofTurnsDone;
                    
                
                Serial3.print("page0.turnsdone.txt=\"");  
                Serial3.print(numberofTurnsDone);
                Serial3.print(" \"");
                Serial3.write(0xff);  
                Serial3.write(0xff);
                Serial3.write(0xff);
                
                nexLoop(nex_listen_list);  // Check for any touch event
            }
            if(i == totalsteps) {
                //Serial.println("Done coiling");
                currentStatus = "Done coiling.";
                break;
            }
            if(stopped == true || pauzed == true) {
                prevNumberofTurnsDone = numberofTurnsDone;
                break;
            }
            nexLoop(nex_listen_list);  // Check for any touch event
        }
        if(stopped == true) {
                break;
        }
        
        
            
    }
    machine.transitionTo(S4);

}


//////////////////////////////
// Non stateMachine methods //
//////////////////////////////

void runSlideMotor(long steps, long interval) {
    digitalWrite(enaSlide, LOW);
    
    long i;
    int pulses = steps * 2;
    for(i=0;i<pulses;i++) {
         pulse = !pulse; // invert pulse variable
       
         digitalWrite(pulSlide, pulse); // Assign step/pul to motordriver (pul, stop, pul, stop..repeat)
         delayMicroseconds(interval); 
    }
}
void runSlideMotorFast(long steps) {
    digitalWrite(enaSlide, LOW);
    long i;
    
     int pulses = steps * 2;
    for(i=0;i<pulses;i++) {
         pulse = !pulse; // invert pulse variable
       
         digitalWrite(pulSlide, pulse); // Assign step/pul to motordriver (pul, stop, pul, stop..repeat)
         delayMicroseconds(interval3); 
    }
}
void stopSlideMotor() { // STop the SLidemotor
    digitalWrite(enaSlide, HIGH); 
}

void runSpindleMotor(long steps, long interval) {
    digitalWrite(enaSpindle, LOW);
    int pulses = steps * 2;
    long i;
    for(i=0;i<pulses;i++) {
         pulse = !pulse; // invert pulse variable
       
         digitalWrite(pulSpindle, pulse); // Assign step/pul to motordriver (pul, stop, pul, stop..repeat)
         delayMicroseconds(interval); 
    }
}
void runSpindleMotorFast(long steps) {
     int pulses = steps * 2;
    digitalWrite(enaSpindle, LOW);
    long i;
    for(i=0;i<pulses;i++) {
         pulse = !pulse; // invert pulse variable
       
         digitalWrite(pulSpindle, pulse); // Assign step/pul to motordriver (pul, stop, pul, stop..repeat)
         delayMicroseconds(interval3); 
    }
}
void stopSpindleMotor() { // STop the SLidemotor
    digitalWrite(enaSpindle, HIGH); 
}


void triggerEndStopRight() {
    digitalWrite(dirSlide, HIGH); // Change direction to right
    rightStopActive = true;
}
void handleEndStopRight() {
    if(millis() - rightStopActiveTime >= 500) {
        if(digitalRead(dirSlide) == LOW) {
            digitalWrite(dirSlide, HIGH); // low CW (right) / high CCW (left)
            stopSlideMotor();
        }
        
        if(digitalRead(rightStop) == LOW) {
            digitalWrite(dirSlide, HIGH); // low CW (right) / high CCW (left)
            //runSlideMotor(stepsPerRevolutionSlideMotor);
            stopSlideMotor();
            
        }
        if(machine.isInState(S1) == true || digitalRead(rightStop) == LOW) {
            
            
            digitalWrite(dirSlide, HIGH); // low CW (right) / high CCW (left)
            //runSlideMotorFast(stepsPerRevolutionSlideMotor);
            stopSlideMotor();
            
        }
         else if(digitalRead(rightStop) == LOW) {
            stopSlideMotor();
        }
        else {
            stopSlideMotor();
        }
        rightStopActiveTime = millis();
        rightStopActive = false;
        
        
        
         Serial.println("Right stop triggered.");
    }
 
   
}
void triggerEndStopLeft() {
    digitalWrite(dirSlide, LOW); // Change direction to right
    leftStopActive = true;
    
    
}
void handleEndStopLeft() {
    
    if(millis() - leftStopActiveTime >= 500) {
        digitalWrite(dirSlide, LOW); // Change direction to right
        stopSlideMotor();
        
        if(machine.isInState(S1) == true || digitalRead(leftStop) == LOW) {
            
            
            homedR = 1;
            transitionS1S2();
            
        }
        else if(machine.isInState(S2) == true || machine.isInState(S3) == true || machine.isInState(S4) == true ) {
            
            
            digitalWrite(dirSlide, LOW); // low CW (right) / high CCW (left)
            //runSlideMotor(stepsPerRevolutionSlideMotor);
            stopSlideMotor();
            
        }
        else if(machine.isInState(S5) == true) {
            
            
            Serial.println("Stopped by endmarker right!");
            stopSlideMotor();
            stopSpindleMotor();
            machine.transitionTo(S0);
            
        }
        else if(digitalRead(leftStop) == LOW) {
            stopSlideMotor();
        }
        else {
            stopSlideMotor();
        }
        leftStopActiveTime = millis();
        leftStopActive = false;
        
        
        
        delay(1000);
        Serial.println("Left stop triggered.");
    }
}

void moveRightBtnPopCallback(void *ptr) {
    
    Serial.println("Move right triggered.");
    if(machine.isInState(S4) == true) {
        digitalWrite(dirSlide, LOW); // low CW (right) / high CCW (left)
        runSlideMotor(400, interval2);
    }
    
}
void moveLeftBtnPopCallback(void *ptr) {
    
    Serial.println("Move left triggered.");
    if(machine.isInState(S4) == true) {
        digitalWrite(dirSlide, HIGH); // low CW (right) / high CCW (left)
        runSlideMotor(400, interval2);
    }
    
}
void moveBackBtnPopCallback(void *ptr) {
    
    Serial.println("Move back triggered.");
    if(machine.isInState(S4) == true) {
        digitalWrite(dirSpindle, LOW); // low CW (BACK) / high CCW (FORWARD)
        runSpindleMotor(800, interval2);
    }
    
}
void moveForwardBtnPopCallback(void *ptr) {
    
    Serial.println("Move forward triggered.");
    if(machine.isInState(S4) == true) {
        digitalWrite(dirSpindle, HIGH); // low CW (back) / high CCW (forward)
        runSpindleMotor(800, interval2);
    }
    
}
void stopBtnPopCallback(void *ptr) {
    stopSlideMotor();
    stopSpindleMotor();
    pauzed =1;  
    Serial.println("Stop triggered.");
    
}


void continue_btnPopCallback(void *ptr) {
    continuePressed = true;
    Serial.println("Nextion continue triggered.");
}
