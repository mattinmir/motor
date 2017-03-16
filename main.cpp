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


double ang_velocity = 100.0; // Revolutions per second
double target_time_period = 1000000.0/ang_velocity; // 1000000 us
double wait_time = 1.0; // us
double time_passed = 0;

//double error_sum = 0;
double prev_error = 0;

double kp = 1.0;
double ki = 0.0;
double kd = 1000000.0;

Timer pid_timer;
uint64_t prev_time = 0;
//uint32_t measurement_interval; // us
double RATE = 0.1;

PID controller(kp, ki, kd, RATE);


Thread th_pid(osPriorityNormal);

Thread thread_user_input(osPriorityLow);

//Ticker interrupt_pid;
/*
void pid()
{
    while(true)
    {
        
        pc.printf("PID \n\r");
        
        uint64_t time = pid_timer.read_us();
        uint64_t time_since_last_pid = time - prev_time;
        
        //uint64_t time_since_last_pid = 1000;
        double error = target_time_period - time_passed; // Proportional
        //pc.printf("%f, %f, %f\n\r",target_time_period, time_passed, error);
        double error_sum = error * time_since_last_pid; // Integral
        double error_deriv = (error - prev_error) / time_since_last_pid; // Derivative
        
        double output = kp*error + ki*error_sum + kd*error_deriv;
        wait_time = (output > 0) ? output : 0;
        
        prev_error = error;
        prev_time = time;
        
        //controller.setProcessValue(time_passed);
        //wait_time = controller.compute();
        //pc.printf("%f %f\n\r", wait_time, time_passed);
        Thread::wait(500); // ms    
    }
}
*/


double revolutions;
double target_revolutions = 200;
double dbl_max = std::numeric_limits<double>::max();


char input[5];
char ch;
int index = 0 ;
void pid()
{
    while(true)
    {
        
        //pc.printf("PID \n\r");
        
        uint64_t time = pid_timer.read_us();
        uint64_t time_since_last_pid = time - prev_time;
        
        //uint64_t time_since_last_pid = 1000;
        double error = target_revolutions - revolutions   ; // Proportional
        
        error = (error > 0) ? error : 0;
        double error_sum = error * (double)time_since_last_pid; // Integral
        double error_deriv = (error - prev_error) / (double)time_since_last_pid; // Derivative
        
        
        
        //double output = std::min(1.0/(kp*error + ki*error_sum + kd*error_deriv), dbl_max);
        double output = kp*error + ki*error_sum + kd*error_deriv;
        double output_angular_velocity = (output > 0) ? output : 0.00000000000001; //if angular velocity is 0 floor it to 0.0001
        wait_time = 1000000.0/((output_angular_velocity)*6.0);
        pc.printf("%f, %f, %f, %f, %f, %f\n\r",target_revolutions, revolutions, error, wait_time, output_angular_velocity, error_deriv);
        
        prev_error = error;
        prev_time = time;
        
        //controller.setProcessValue(time_passed);
        //wait_time = controller.compute();
        //pc.printf("%f %f\n\r", wait_time, time_passed);
        Thread::wait(100); // ms    
    }
}

/*
void user_input()
{
    while(true)
    {
        if(pc.readable()){
        
        //bool input_flag = 1 ;
        
        //while(input_flag == 1) {
            //ch = pc.getc();
           // input[index++] = ch - '0'; 
            
            //if(ch ='\n')
            
        //}
        
        
        while(ch != ' '){           ///////// would look of \n but there was some problem with putty 
            ch = pc.getc();
            input[index++] = ch - '0';   
        }
     target_revolutions = 100*input[0]+10*input[1]+input[2] ;   
    }
    
    }
       
    
}
*/

/*void readbuf()
 {
    if (readSize >= sizeof(cWord)) { readSize = sizeof(cWord)-1; }
    int iRtn =  pc.GetString(readSize,cWord); //Serial received chars byref of cWord
    pc.printf("inputreadbuff %s \n\r" , cWord) ;
 }*/





//////////Ticker input_ticker ;

/*************************************
                Main
*************************************/

int main() 
{
    
    /*
    controller.setSetPoint(target_time_period);
    controller.setInputLimits(1000, 1000000);
    controller.setOutputLimits(0, 1<<30);
    */
    
    /*Reading Input Variables*/ 
    

    char cWord[16];
    int readSize = 15;

    
    
    int8_t orState = 0;    //Rotot offset at motor state 0
    
    
    intState = 0;
    int8_t intStateOld = 0;
    pc.printf("Hello\n\r");
    
    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);

    revolutions = 0;
    uint32_t reference_time = 0;


    timer.start();
    pid_timer.start();
    
    photo_checker_interrupt.attach(&check_photo, 0.0001);
    
    th_pid.start(&pid);
    //interrupt_pid.attach(&pid, 0.001);
    //thread_user_input.start(&user_input);
    
    ////////////input_ticker.attach(&read_in , 10) ;
    
    
    while (1) 
    {
        //pc.printf("Main \n\r");
        int8_t local_intState = intState;
        if (local_intState != intStateOld) 
        {
            if(local_intState == orState)
            {
               revolutions++;
               //pc.printf("%d\n\r", revolutions);
               // Calculate time for previous revolution
               double current_time = (double)timer.read_us();
               time_passed = current_time - reference_time;

            
               /*
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
                
                */

               //pc.printf("%d\n\r", wait_time);
               reference_time = current_time;
               //pc.printf("%f\n\r", time_passed);
               //pid();
               
            }
            
            
               
        intStateOld = local_intState;
        wait_us(wait_time);
        motorOut((local_intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
            
        }
    
    if(pc.readable()){      // if a key is pressed then read in each char and put into string to be processed into commands
              
        while(ch != '\n\r'){           ///////// would look of \n but there was some problem with putty  
            ch = pc.getc();
            input[index++] = ch - '0';  
             
        }
     target_revolutions = 100*input[0]+10*input[1]+input[2] ; 
     ch = '0';  
    }
    
  /*  pc.baud(9600);                  //set baud rate
    pc.format(8, MySerial::None, 1);//set bits for a byte, parity bit, stop bit
    pc.SetRxWait(0.01, 0.001);       //set wait getting chars after interrupted, each char
    pc.attach( readbuf, MySerial::RxIrq );    //Set Interrupt by Serial receive
    
    //pc.printf(" target from input input: %s \n\r" , target_revolutions) ;
    pc.printf("input %s \n\r" , cWord) ;*/
    
    
    
    }
    
}
 
