#include "mbed.h"
#include "rtos.h"
 
//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12
 
//Incremental encoder input pins
#define CHA   D7
#define CHB   D8  
 
//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20
 
//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/

/*
typedef enum 
{

    osPriorityLow           = -2,
    osPriorityBelowNormal   = -1,
    osPriorityNormal        =  0,
    osPriorityAboveNormal   =  1,
    osPriorityHigh          =  2,
    osPriorityError        =  0x84
} osPriority;
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};
 
//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed
 
//Phase lead to make motor spin
const int8_t lead = -2;  //2 for forwards, -2 for backwards
 
//Status LED
DigitalOut led1(LED1);
 
//Photointerrupter inputs
DigitalIn I1(I1pin);
DigitalIn I2(I2pin);
DigitalIn I3(I3pin);
 
//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);
 
//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
    }
    
    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }
 
//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(1.0);
    
    //Get the rotor state
    return readRotorState();
}
    
volatile int8_t intState;



void check_photo() 
{
    intState = readRotorState(); 
    
}

Ticker photo_checker_interrupt;
Timer timer;

int main() 
{
    int8_t orState = 0;    //Rotot offset at motor state 0
    
    //Initialise the serial port
    Serial pc(SERIAL_TX, SERIAL_RX);
    
    intState = 0;
    int8_t intStateOld = 0;
    pc.printf("Hello\n\r");
    
    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);

    uint32_t revolutions = 0;
    uint32_t reference_time = 0;
    uint32_t wait_time = 1;
    uint32_t ang_velocity = 30; // Revolutions per second
    
    photo_checker_interrupt.attach(&check_photo, 0.00005);
    
    timer.start();
    while (1) 
    {
        if (intState != intStateOld) 
        {
            if(intState == orState)
            {
               revolutions++;
               
               // Calculate time for previous revolution
               uint32_t current_time = timer.read_us();
               uint32_t time_passed = current_time - reference_time;
               
               if (time_passed < 1000000.0/ang_velocity) // 1000000 us
                {
                     //pc.printf("Too Fast\n\r");
                     wait_time+=50;   
                }
                else if (time_passed > 1000000.0/ang_velocity) // 1000000 us
                {
                     //pc.printf("Too Slow\n\r");
                     if (wait_time >= 5)
                     {    
                        wait_time-=5;
                     }
                }
               
               //pc.printf("%d\n\r", wait_time);
               reference_time = current_time;
            }
               
        intStateOld = intState;
        wait_us(wait_time);
        motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
            
        }
    }
}
 
