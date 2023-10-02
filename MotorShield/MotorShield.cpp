/* Library to interface with 2.74 Motor Shield
** Uses low level HAL libraries to enable high speed PWM 
** Use as follows:
** - Create shield object and specify PWM period for the motors
** - Set the duty cycle and direction for each motor 
*/

#include "mbed.h"
#include "MotorShield.h"
#include "HardwareSetup.h"

MotorShield::MotorShield(int periodTicks) {
    periodTickVal = periodTicks; 
    init();
}
 
void MotorShield::init() {
    /** Initial config for the STM32H743 **/
    
    initHardware(periodTickVal);   // Setup PWM
    wait_us(100);
        
}
 
void MotorShield::motorAWrite(float duty_cycle, int direction) {
    int tick = (int)(periodTickVal * duty_cycle); 

    if (direction){
            
            TIM15->CCR2 = tick;
            TIM15->CCR1 = 0;

    }
    else {
            TIM15->CCR2 = 0;
            TIM15->CCR1 = tick;
    }
    
}

void MotorShield::motorBWrite(float duty_cycle, int direction) {
    int tick = (int)(periodTickVal * duty_cycle); 

    if (direction){
            TIM12->CCR2 = tick;
            TIM12->CCR1 = 0;
    }
    else {
            TIM12->CCR2 = 0;
            TIM12->CCR1 = tick;
    }

}

void MotorShield::motorCWrite(float duty_cycle, int direction) {
    int tick = (int)(periodTickVal * duty_cycle); 

    if (direction){
            TIM13->CCR1 = 0;
            TIM14->CCR1 = tick;
    }
    else {
            TIM13->CCR1 = tick;
            TIM14->CCR1 = 0;
    }
}
void MotorShield::motorDWrite(float duty_cycle, int direction) {
    int tick = (int)(periodTickVal * duty_cycle); 

    if (direction){
            TIM16->CCR1 = tick;
            TIM17->CCR1 = 0;
    }
    else {
            TIM16->CCR1 = 0;
            TIM17->CCR1 = tick;
    }
}

uint32_t MotorShield::readCurrentA() {
    return readADC1(0);
}
uint32_t MotorShield::readCurrentB() {
    return readADC1(1);
}
uint32_t MotorShield::readCurrentC() {
    return readADC2(1);
}
uint32_t MotorShield::readCurrentD() {
    return readADC2(0);
}

void MotorShield::changePeriod(int periodTicks){
    periodTickVal = periodTicks; 
    init();
}

