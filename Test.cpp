#include "Motor.h"
#include "QEI.h"
#include "PID.h"

const float kp = 0.412;
const float ki = 0.1;
const float kd = 0.0;
const float Ts = 0.01;

int Pulses      = 0;
int PrevPulses  = 0;
float Velocity  = 0.0;
int distance    = 6000;

int ppr = 22;
float setpoint;
int flag;
float userInput;

Motor Motor(D9, D2, D4);
QEI Qei(D5, D6, NC, ppr, QEI::X2_ENCODING);
PID Pid(kp, ki, kd, Ts);

void call_speed () {

    Pulses = Qei.getPulses();
    //printf("Puulse: %d\r\n",Pulses);
    Velocity = (Pulses - PrevPulses) / Ts;
    PrevPulses = Pulses;

    Pid.setProcessValue(fabs(Velocity));
 
}

void direction () {
    
    if (( setpoint >= 0) ) { 
         Motor.speed(Pid.compute());
    }         
         
    else {  
         Motor.speed(-Pid.compute());
    }

}

void userIn () {
    
    printf("Enter Speed/RPM (-250.0 to 250.0)\r\n");
    flag = scanf("%f", &userInput);
    setpoint = userInput;
    printf(">>> %1.2f RPM\r\n", setpoint);

}

int main () {
    userIn ();
    
    // Set parameters
    int distance    = 6000;
    Motor.period(0.00004);
    Pid.setInputLimits(0, 130);
    Pid.setOutputLimits(0.0, 0.9);
    Pid.setMode(AUTO_MODE);

    thread_sleep_for (1);
    
    setpoint = setpoint / 60;
    Pid.setSetPoint(fabs(setpoint));

    while ( (Pulses < distance) &&  (Pulses > -distance) ) {
        
        call_speed ();
        direction ();
       
    } 
    
    Motor.brake();
    
} 


