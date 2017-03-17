#include "mbed.h"
#include "rtos.h"
#include "PID.h"
#include "stdint.h"
#include <limits>
#include <algorithm>
#include <RawSerial.h>

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



//Initialise the serial port
Serial pc(SERIAL_TX, SERIAL_RX);
//rawSerial pc(USBTX,USBRX);

double dbl_max = std::numeric_limits<double>::max();




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
    
/*************************************
             Interrupts
*************************************/  
    
volatile int8_t intState;
Ticker photo_checker_interrupt;
Timer timer;

void check_photo() 
{
    intState = readRotorState(); 
}



/*************************************
             Threads
*************************************/ 

/* PID */

Thread th_pid(osPriorityNormal, 2048);
Timer pid_timer;

double kp = 0.1;
double ki = 0.0;
double kd = 10000000000;

double target_ang_velocity = 100.0; // Revolutions per second
double target_time_period = 1000000.0/target_ang_velocity; // 1000000 us
double target_revolutions = 100;

double revolutions = 0;

double wait_time = 1.0; // us
double time_passed = 0;

double prev_error = 0;
uint64_t prev_time = 0;

int8_t changed_intState;
int8_t orState;

void pid()
{
    while(true)
    {
        //pc.printf("PID \n\r");
        
        // Get time since last PID
        uint64_t time = pid_timer.read_us();
        uint64_t time_since_last_pid = time - prev_time;
    
        // Calculate errors
        double error = target_revolutions - revolutions   ; // Proportional
        double error_sum = error * (double)time_since_last_pid; // Integral
        double error_deriv = (error - prev_error) / ((double)time_since_last_pid*1000000); // Derivative
        
        // Weight errors to calculate output
        double output = kp*error + ki*error_sum + kd*error_deriv;
        
        // Lower limit output to small non-zero value to avoid divby0 error
        double output_angular_velocity = (output > 0) ? output : 0.00000000000001; 
        wait_time = 1000000.0/((output_angular_velocity)*6.0);
        
        
        // Store values for next iteration
        prev_error = error;
        prev_time = time;
        
        //pc.printf("%f, %f, %f, %f, %f, %f\n\r",target_revolutions, revolutions, error, wait_time, output_angular_velocity, error_deriv);
        
        Thread::wait(100); // ms    
    }
}



/* Motor Control */

Thread th_motor_control(osPriorityNormal, 1024);

void move_field()
{
    while(true)
    {
        Thread::wait(wait_time/1000.0);  
        motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive 
    }
}



/*************************************
                Main
*************************************/

int main() 
{
    orState = motorHome();
    int8_t intStateOld = 0;
    intState = orState;
    changed_intState = orState;
    uint32_t reference_time = 0;
    //new_state = true;
    
    pc.printf("Hello\n\r");
    pc.printf("Rotor origin: %x\n\r",orState);

    

    /* Start Timers */
    timer.start();
    pid_timer.start();
    
    /* Start Interrupts */
    photo_checker_interrupt.attach(&check_photo, 0.0001);
    
    /* Start Threads */
    th_pid.start(&pid);
    th_motor_control.start(&move_field);
    
    
    while (true) 
    {        
        pc.printf("%f\n\r", revolutions);
        
        int8_t local_intState = intState;
        if (local_intState != intStateOld) 
        {
            //new_state = true;
            //changed_intState = local_intState;
            if(local_intState == orState)
            {
               revolutions++;
            
               // Calculate time for previous revolution
               double current_time = (double)timer.read_us();
               time_passed = current_time - reference_time;

               reference_time = current_time;
            }
            
            
            intStateOld = local_intState;            
        }
        
        // On keypress read in chars and put into string to be processed into commands
        if(pc.readable())
        {     
            unsigned index = 0; 
            char cmd[20];
            char ch;
            
            while(ch != '\r')
            {           
                ch = pc.getc();
                
                if(ch == 'R')
                {  
                    
                }
                cmd[index++] = ch - '0';
                pc.printf("%c",ch);    
            }
         //target_revolutions = 100*input[0] + 10*input[1] + input[2] ; 
         //ch = '0';  
         pc.printf("\n\r%s\n\r", cmd);
        }    
    }
}
 
