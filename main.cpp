#include "mbed.h"
#include "rtos.h"
#include "stdint.h"
#include <limits>
#include <algorithm>
#include <RawSerial.h>
#include <cstdlib>
#include <string>
#include <map>
#include <vector>


/*

#define C 0.000477783f
#define Cs 0.000450966f
#define Db 0.000450966f
#define D 0.000425655f
#define Ds 0.000401765f
#define Eb 0.000401765f
#define E 0.000379216f
#define F 0.000357932f
#define Fs 0.000337842f
#define Gb 0.000337842f
#define G 0.000318882f
#define Gs 0.000300984f
#define Ab 0.000300984f
#define A 0.000284091f
#define As 0.000268146f
#define Bb 0.000268146f
#define B 0.000253096f

*/
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

double dbl_max = std::numeric_limits<double>::max();


//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};
 
//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed
 
//Phase lead to make motor spin
 int8_t lead = 2;  //2 for forwards, -2 for backwards  //
 
//Status LED
DigitalOut led1(LED1);
 
//Photointerrupter inputs
DigitalIn I1(I1pin);
DigitalIn I2(I2pin);
DigitalIn I3(I3pin);
 
//Motor Drive outputs
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

double delta = 1;
double OGdelta =0;
 
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
    if (driveOut & 0x01) L1L = delta;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = delta;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = delta;
    if (driveOut & 0x20) L3H = 0;
    }
    
    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }
    

//Initialise the serial port
//Serial pc(SERIAL_TX, SERIAL_RX);
RawSerial pc(USBTX,USBRX);    
    
 
//Basic synchronisation routine  -- improved to set to 0 before next command  
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    
    int two_prev_state = -2;                        // states needed to determine when motor has stopped spinning
    int prev_state = -1;
    int current_state = readRotorState();
    int going = 1 ;   
    
    while(going==1){
        if( !(current_state == prev_state && current_state == two_prev_state))  // while the states are still changing, keep shifting to the next state to compare
        {
            two_prev_state = prev_state;
            prev_state = current_state ;
            current_state = readRotorState();
            wait(0.3);                  // should be tuned
        }
        else
        {
            going = 0;
        }
    }
    //Get the rotor state
    //pc.printf("out of motorhome");
    return readRotorState();
}

    
/*************************************
             Interrupts
*************************************/ 
    
/* Read Photointerrupters */
void check_photo();

Ticker photo_checker_interrupt;
volatile int8_t intState;
    

/* Count Revolutions */
void increment_revolutions();

InterruptIn photointerrupter(D12);
Timer rev_count_timer;
double time_passed = 0;
double revolutions = 0;
uint32_t reference_time = 0;
double ang_velocity = 0;


/*************************************
             Threads
*************************************/ 


// General Global Variables
double wait_time; 
double prev_error = 0;
uint64_t prev_time = 0;
 
/* PID Revolutions */
void pid_rev();
 
Thread* th_pid_rev;
Timer pid_timer_rev;
double target_revolutions;
double max_ang_velocity = 100000;
double kp_rev;
double ki_rev;
double kd_rev;


/*   PID Velocity   */
void pid_vel();

Thread* th_pid_vel;
Timer pid_timer_vel;
double target_ang_velocity; 
double kp_vel;
double ki_vel;
double kd_vel;


/* Motor Control */
void move_field();
Thread* th_motor_control;
int8_t orState;


/* Melody Thread */
void play_melody();
Thread* th_play_melody;
std::map<std::string,double> notes;
double melody[16][2];
unsigned no_of_notes = 0;




/*************************************
                Utility
*************************************/

/* Terminate, delete, & respawn threads */
void reset_threads();


//#define A 1
//#define B 2 
    
    
//double notes[4] = { 1 , 2 , 3 , 4 } ;



/*************************************
                Main
*************************************/

int main() 
{

     pc.printf("Before\n\r");
    notes["C"] = 0.000477783f;
    notes["C#"] = 0.000450966f;
    notes["D^"] = 0.000450966f;
    notes["D"] = 0.000425655f;
    notes["D#"] = 0.000401765f;
    notes["E^"] = 0.000401765f;
    notes["E"] = 0.000379216f;
    notes["F"] = 0.000357932f;
    notes["F^"] = 0.000337842f;
    notes["G^"] = 0.000337842f;
    notes["G"] = 0.000318882f;
    notes["G#"] = 0.000300984f;
    notes["A^"] = 0.000300984f;
    notes["A"] = 0.000284091f;
    notes["A#"] = 0.000268146f;
    notes["B^"] = 0.000268146f;
    notes["B"] = 0.000253096f;

    
  
    
    orState = motorHome();
    intState = orState;

    
    

    pc.printf("Hello\n\rEnter command to begin.\n\r");
    
    /* PID Tuning */
    
    // pid_vel DO NOT CHANGE
    kp_vel = 1.0;
    ki_vel = 0.000001;
    kd_vel = 0.0;
    
    // pid_rev  
    kp_rev = 0.2;
    ki_rev = 0.0;
    kd_rev = 1000000000.0;
    
    // Initial Wait Time
    wait_time = 100; // us

    /* Start Timers */
    rev_count_timer.start();
    pid_timer_rev.start();
    pid_timer_vel.start();
     
    /* Start Interrupts */
    photo_checker_interrupt.attach(&check_photo, 0.0001);
    photointerrupter.rise(&increment_revolutions);
    
    /* Create Threads */
    th_pid_rev = new Thread(osPriorityNormal, 1024);
    th_pid_vel = new Thread(osPriorityNormal, 1024);
    th_motor_control = new Thread(osPriorityNormal, 1024);
    th_play_melody = new Thread(osPriorityNormal, 1024);
    
    
    
    while (true) 
    {        
    

        
    
        //pc.printf("%f, %f\n\r", ang_velocity, revolutions);
        //pc.printf("%f\n\r", revolutions);
        //pc.printf("%f, %f, %f\n\r", target_ang_velocity, ang_velocity, wait_time);
      

        
        // On keypress read in chars and put into string to be processed into commands
        if(pc.readable())
        {     

            unsigned index = 0;
            unsigned size = 0; 
            unsigned max_size = 32;
            char cmd[max_size];
            char ch = ' ';
            
            // Read in entire command into `cmd`
            while(ch != '\r')
            {           
                ch = pc.getc();
                
                //Detect backspace
                if(ch == 127 && index > 0)
                {
                    index--;
                    size--;
                    pc.printf("\b");
                        
                }
                else
                {
                    cmd[index++] = ch;
                    size++;
                    pc.printf("%c", ch);
                }
            }
            reset_threads();
            motorHome();
            
            // Set target revolutions
            if (cmd[0] == 'R')
            {
                char* next_cmd;
                
                // Read revolution target and convert to double
                target_revolutions = strtod(cmd+1, &next_cmd);
                if(target_revolutions < 0 ){
                    lead = -2 ;
                    target_revolutions = abs(target_revolutions) ;
                }
                else{
                    lead = 2;
                }
                
                // Set max velocity
                if(*next_cmd == 'V')
                {
                    // abs() to ignore sign
                    max_ang_velocity = abs(strtod(next_cmd+1, NULL));
                    
                    // Terminate and recreate Thread objects
                    //reset_threads();
                    
                    // Reset count & wait time
                    revolutions = 0;
                    wait_time = 100;
                    
                    pc.printf("Target revolutions is %f revs\n\rMax velocity is %f\n\r",target_revolutions, max_ang_velocity);
                    
                    // Respawn threads
                    th_motor_control->start(&move_field);
                    th_pid_rev->start(&pid_rev);
                    
                }
                
                // If no velocity given
                else
                {
                    // Terminate and recreate Thread objects
                    //reset_threads();
                    
                    // Reset count & wait time
                    revolutions = 0;
                    wait_time = 100;
                    
                    // No velocity limit given so set very high
                    max_ang_velocity = 100000;
                    
                    pc.printf("Target revolutions is %f revs\n\r",target_revolutions);
                    
                    // Respawn threads
                    th_motor_control->start(&move_field);
                    th_pid_rev->start(&pid_rev);

                    
                }
            }
            
            // Set target velocity
            else if (cmd[0] == 'V')
            {
                //pc.printf("V: %f\n\r",strtod(cmd+1, NULL));
                target_ang_velocity = strtod(cmd+1, NULL);
                if(target_ang_velocity < 0){
                    lead = -2 ;
                    target_ang_velocity = abs(target_ang_velocity);
                }
                else
                {
                    lead =2;
                }
                // Terminate and recreate Thread objects
                //reset_threads();

                // Reset wait time
                wait_time = 100;
                
                pc.printf("Target velocity is %f rev/s\n\r",target_ang_velocity);
                
                // Respawn threads
                th_motor_control->start(&move_field);
                th_pid_vel->start(&pid_vel);
                
            }
            
            
            // Melody
            else if (cmd[0] == 'T')
            {
                pc.printf("1\n\r");
                for(unsigned i = 0; i <16; i++)
                {
                    for(unsigned j = 0; j < 2; j++)
                    {
                        melody[i][j] = 0;
                    }
                }
                pc.printf("2\n\r");
                unsigned i = 1;
                unsigned note = 0;
                char pitch[2];
                
                // Repeat for every note given in command
                while (i < size)
                {
                    pitch[0] = cmd[i++];
                    pitch[1] = cmd[i++];
                    pc.printf("3\n\r");
                    // If not a sharp or flat
                    // eg. pitch = ['A', '4']
                    if(!(pitch[1] == '#' || pitch[1] == '^'))
                    {
                        // Convert char 'A' to std::string('A') and use to index `notes` map
                        double frequency = notes[std::string(1, pitch[0])];
                        pc.printf("4\n\r");
                        // Convert char '4' to double 4.0
                        double duration = strtod(pitch+1, NULL);
                       pc.printf("5\n\r");
                        melody[note][0] = frequency;
                        melody[note][1] = duration;
                        note++;
                        pc.printf("6\n\r");
                    }
                    
                    // If sharp or flat
                    // eg. pitch = ['F', '#']
                    else
                    {
                        // Convert char* ['F', '#'] to std::string("F#") and use to index `notes` map
                        double frequency = notes[std::string(pitch, 2)];
                        // Read in next char that is the duration and convert to double
                        char dur[1];
                        dur[0] = cmd[i++];
                        double duration = strtod(dur, NULL);
                        
                        melody[note][0] = frequency;
                        melody[note][1] = duration;
                        note++;
                    }
                    
                } 
                // Keep track of how many notes were requested
                no_of_notes = note-1;
                
                wait_time = 100;
                target_ang_velocity = 40;
                
                // Respawn threads
                pc.printf("7\n\r");
                th_motor_control->start(&move_field);
                pc.printf("8\n\r");
                th_pid_vel->start(&pid_vel);
                pc.printf("9\n\r");
                th_play_melody->start(&play_melody);
                
             
            }
            

        }
     //pc.printf("....intState:   %d  .... \n\r" , intState );
     Thread::wait(200);
    }
}
 
 
 void check_photo() 
{
    intState = readRotorState(); 
}

void increment_revolutions()
{
    revolutions++;
    
    // Calculate time for previous revolution
    double current_time = (double)rev_count_timer.read_us();
    time_passed = current_time - reference_time;
    
    ang_velocity = 1000000.0/time_passed;
    reference_time = current_time;
}


void pid_rev()
{

    while(true)
    {
       
        //pc.printf("PID \n\r");
        delta = 1;
        // Get time since last PID
        uint64_t time = pid_timer_rev.read_us();
        uint64_t time_since_last_pid = time - prev_time;
    
        // Calculate errors
        double error = target_revolutions - revolutions   ; // Proportional
        double error_sum = error * (double)time_since_last_pid; // Integral
        double error_deriv = (error - prev_error) / ((double)time_since_last_pid*1000000.0); // Derivative
        
        // Weight errors to calculate output
        double output = kp_rev*error + ki_rev*error_sum + kd_rev*error_deriv;
        
        
        //if(OGdelta ==0) OGdelta = output;
        
        //delta = (output > 0) ? output : 0;
        
        //delta = output/OGdelta;
    
 
    
        
        // Lower limit output to small non-zero value to avoid divby0 error
        double output_angular_velocity = (output > 0) ? output : 0.00000001; 

        pc.printf("%f,%f\n\r", output_angular_velocity, revolutions);

        // Cap at max velocity
        output_angular_velocity = (output_angular_velocity < max_ang_velocity) ? output_angular_velocity : max_ang_velocity;

        // Convert velocity to wait time
        wait_time = 1000000.0/((output_angular_velocity)*6.0);

        
        // Store values for next iteration
        prev_error = error;
        prev_time = time;

        //pc.printf("%f, %f, %f, %f, %f, %f\n\r",target_revolutions, revolutions, error, error_deriv, max_ang_velocity, ang_velocity);

        Thread::wait(100); // ms    
    }
}

void pid_vel()
{
    while(true)
    {       
        wait_time = 0;
        // Get time since last PID
        uint64_t time = pid_timer_vel.read_us();
        uint64_t time_since_last_pid = time - prev_time;
    
        // Calculate errors
        double error = target_ang_velocity - ang_velocity   ; // Proportional
        double error_sum = error * (double)time_since_last_pid; // Integral
        double error_deriv = (error - prev_error) / ((double)time_since_last_pid*1000000.0); // Derivative
        
        // Weight errors to calculate output
        //double output = kp_vel*error + ki_vel*error_sum + kd_vel*error_deriv;
        delta = kp_vel*error + ki_vel*error_sum + kd_vel*error_deriv;
        
        
        //pc.printf("%f\n\r", delta);
        
        // Lower limit output to small non-zero value to avoid divby0 error
       // double output_angular_velocity = (output > 0) ? output : 0.00000001; 
        
        // Convert velocity to wait time
        //wait_time = 1000000.0/((output_angular_velocity)*6.0);
        
        
        // Store values for next iteration
        prev_error = error;
        prev_time = time;
        
        //Debugging
        //pc.printf("%f, %f, %f, %f, %f, %f\n\r",target_ang_velocity, ang_velocity, error, error_sum, error_deriv, wait_time);
        //pc.printf("ang vel: %f  \n\r" , ang_velocity );
        Thread::wait(100); // ms    
    }
}

void move_field()
{
    while(true)
    {
        //pc.printf("%f\n\r", wait_time);
        motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive 
        //pc.printf("state:  %d \r\n" , states) ;
        if(wait_time>0)
        {
        Thread::wait(wait_time/1000.0);  
        }
        
    }
}

void play_melody()
{

    while(true)
    {
        for(unsigned i =0; i < no_of_notes; i++)
        {
         double frequency = melody[i][0];
         L1L.period(frequency);
         L2L.period(frequency);
         L3L.period(frequency);
         
         Thread::wait(melody[i][1]);
        }
    }
}       

void reset_threads()
{
    th_pid_vel->terminate();
    th_pid_rev->terminate();
    th_motor_control->terminate();
    th_play_melody->terminate();
    
    delete th_pid_vel;
    delete th_pid_rev;
    delete th_motor_control;
    delete th_play_melody;
    

    th_pid_rev = new Thread(osPriorityNormal, 1024);
    th_pid_vel = new Thread(osPriorityNormal, 1024);
    th_motor_control = new Thread(osPriorityNormal, 1024);
    th_play_melody = new Thread(osPriorityNormal, 1024);

}
