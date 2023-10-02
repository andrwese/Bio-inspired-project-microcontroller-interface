/* Library to interface with 2.74 Motor Shield
** Uses low level HAL libraries to enable high speed PWM 
** Use as follows:
** - Create shield object and specify PWM period for the motors
** - Set the duty cycle and direction for each motor 
*/

class MotorShield {
    
    public:
    
    MotorShield(int periodTicks);
    void motorAWrite(float duty_cycle, int direction);
    void motorBWrite(float duty_cycle, int direction);
    void motorCWrite(float duty_cycle, int direction);
    void motorDWrite(float duty_cycle, int direction);
    uint32_t readCurrentA();
    uint32_t readCurrentB();
    uint32_t readCurrentC();
    uint32_t readCurrentD();
    void changePeriod(int periodTicks); 
    
    private:
    
    int periodTickVal;
    void init();
    
};

