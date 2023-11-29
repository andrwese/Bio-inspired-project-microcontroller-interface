#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "MotorShield.h" 
#include "HardwareSetup.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>


#define dt  0.03        // length of timestep
#define N   16          // number of timesteps

#define Nz 3           // number of controllable generalized coordinates (q2 - q4)
#define NUM_INPUTS 14 + 6*(N+1)     
#define NUM_OUTPUTS 18 
#define PULSE_TO_RAD (2.0f*3.14159f / 1200.0f)

// Initializations
Serial pc(USBTX, USBRX);    // USB Serial Terminal
ExperimentServer server;    // Object that lets us communicate with MATLAB
Timer t;                    // Timer to measure elapsed time of experiment

QEI encoderA(PE_9,PE_11, NC, 1200, QEI::X4_ENCODING);  // MOTOR A ENCODER (no index, 1200 counts/rev, Quadrature encoding) -> Hip joint
QEI encoderB(PA_5, PB_3, NC, 1200, QEI::X4_ENCODING);  // MOTOR B ENCODER (no index, 1200 counts/rev, Quadrature encoding) -> Knee joint
QEI encoderC(PC_6, PC_7, NC, 1200, QEI::X4_ENCODING);  // MOTOR C ENCODER (no index, 1200 counts/rev, Quadrature encoding) -> Shoulder joint
QEI encoderD(PD_12, PD_13, NC, 1200, QEI::X4_ENCODING);// MOTOR D ENCODER (no index, 1200 counts/rev, Quadrature encoding) -> Gantry?

MotorShield motorShield(24000); //initialize the motor shield with a period of 24000 ticks or ~10kHZ
Ticker currentLoop;

// Variables for q1 -> body
float angle1_init;
float angle1;
float velocity1;

// Variables for q2 -> hip
float current2;
float current_des2 = 0;
float prev_current_des2 = 0;
float current_int2 = 0;
float angle2;
float velocity2;
float duty_cycle2;
float angle2_init;

// Variables for q3 -> knee
float current3;
float current_des3 = 0;
float prev_current_des3 = 0;
float current_int3 = 0;
float angle3;
float velocity3;
float duty_cycle3;
float angle3_init;

// Variables for q4 -> shoulder
float current4;
float current_des4 = 0;
float prev_current_des4 = 0;
float current_int4 = 0;
float angle4;
float velocity4;
float duty_cycle4;
float angle4_init;

// Fixed kinematic parameters
const float l_OA=.152;              // length from origin to end of pendulum
const float l_AB=.083;              // end of pendulum to hip joint
const float l_BC=.082;              // length of thigh link
const float l_CD=.088;              // length of leg link
const float l_AE=.096;              // length of arm link

// Timing parameters
float current_control_period_us = 200.0f;     // 5kHz current control loop
float impedance_control_period_us = dt*1e6;  // 100Hz impedance control loop        ---- verify that this is possible ----
float start_period, traj_period, end_period;

// Control parameters
float current_Kp = 4.0f;         
float current_Ki = 0.4f;           
float current_int_max = 3.0f;       
float duty_max;      
float K_2; // joint control -> P-gain for q2
float K_3;
float K_4;
float D_2; // joint control -> D-gain for q2
float D_3;
float D_4;

// Model parameters
float supply_voltage = 12;     // motor supply voltage
float R = 2.0f;                // motor resistance
float k_t = 0.18f;             // motor torque constant
float nu = 0.0005;             // motor viscous friction

// Current control interrupt function
void CurrentLoop() {
    // This loop sets the motor voltage commands using PI current controllers with feedforward terms.
    
    //use the motor shield as follows:
    //motorShield.motorAWrite(DUTY CYCLE, DIRECTION), DIRECTION = 0 is forward, DIRECTION =1 is backwards.
        
    // HIP JOINT MOTOR
    current2 = -(((float(motorShield.readCurrentA())/65536.0f)*30.0f)-15.0f);           // measure current
    velocity2 = encoderA.getVelocity() * PULSE_TO_RAD;                                  // measure velocity        
    float err_c2 = current_des2 - current2;                                             // current errror
    current_int2 += err_c2;                                                             // integrate error
    current_int2 = fmaxf( fminf(current_int2, current_int_max), -current_int_max);      // anti-windup
    float ff2 = R*current_des2 + k_t*velocity2;                                         // feedforward terms
    duty_cycle2 = (ff2 + current_Kp*err_c2 + current_Ki*current_int2)/supply_voltage;   // PI current controller
    
    float absDuty2 = abs(duty_cycle2);
    if (absDuty2 > duty_max) {
        duty_cycle2 *= duty_max / absDuty2;
        absDuty2 = duty_max;
    }    
    if (duty_cycle2 < 0) { // backwards
        motorShield.motorAWrite(absDuty2, 1);
    } else { // forwards
        motorShield.motorAWrite(absDuty2, 0);
    }             
    prev_current_des2 = current_des2; 
    
    // KNEE JOINT MOTOR
    current3 = -(((float(motorShield.readCurrentB())/65536.0f)*30.0f)-15.0f);           // measure current
    velocity3 = encoderB.getVelocity() * PULSE_TO_RAD;                                  // measure velocity  
    float err_c3 = current_des3 - current3;                                             // current error
    current_int3 += err_c3;                                                             // integrate error
    current_int3 = fmaxf( fminf(current_int3, current_int_max), -current_int_max);      // anti-windup   
    float ff3 = R*current_des3 + k_t*velocity3;                                         // feedforward terms
    duty_cycle3 = (ff3 + current_Kp*err_c3 + current_Ki*current_int3)/supply_voltage;   // PI current controller
    
    float absDuty3 = abs(duty_cycle3);
    if (absDuty3 > duty_max) {
        duty_cycle3 *= duty_max / absDuty3;
        absDuty3 = duty_max;
    }    
    if (duty_cycle3 < 0) { // backwards
        motorShield.motorBWrite(absDuty3, 1);
    } else { // forwards
        motorShield.motorBWrite(absDuty3, 0);
    }             
    prev_current_des3 = current_des3; 

    // SHOULDER JOINT MOTOR
    current4 = -(((float(motorShield.readCurrentC())/65536.0f)*30.0f)-15.0f);           // measure current
    velocity4 = encoderB.getVelocity() * PULSE_TO_RAD;                                  // measure velocity  
    float err_c4 = current_des4 - current4;                                             // current error
    current_int4 += err_c4;                                                             // integrate error
    current_int4 = fmaxf( fminf(current_int4, current_int_max), -current_int_max);      // anti-windup   
    float ff4 = R*current_des4 + k_t*velocity4;                                         // feedforward terms
    duty_cycle4 = (ff4 + current_Kp*err_c4 + current_Ki*current_int4)/supply_voltage;   // PI current controller
    
    float absDuty4 = abs(duty_cycle4);
    if (absDuty4 > duty_max) {
        duty_cycle4 *= duty_max / absDuty4;
        absDuty4 = duty_max;
    }    
    if (duty_cycle4 < 0) { // backwards
        motorShield.motorCWrite(absDuty4, 1);
    } else { // forwards
        motorShield.motorCWrite(absDuty4, 0);
    }             
    prev_current_des4 = current_des4; 
}

void readTrajectoriesFromFile(std::string filename, float (&array1)[N+1], float (&array2)[N+1], float (&array3)[N+1]){
    // populates the three float arrays by reading out the tree first rows of the file
    std::ifstream file(filename);
    if (!file.is_open()) {
        pc.printf("Error opening file");
        return;
    }
    std::string line;
    const int ROWS = 4;
    const int COLS = N+1;

    int row = 1;

    while (std::getline(file, line) && row < ROWS) {
        std::stringstream ss(line);
        std::string cell;
        int col = 0;

        while (std::getline(ss, cell, ',') && col < COLS) {
            // Convert the cell value to a float and populate the arrays
            if (row == 1) {
                array1[col] = std::stof(cell);
            } else if (row == 2) {
                array2[col] = std::stof(cell);
            } else if (row == 3) {
                array3[col] = std::stof(cell);
            }

            col++;
        }

        row++;
    }

}

int main(void) {
    pc.baud(115200);
    // Object for desired trajectory of all angles
    float q2_des[N+1];
    float q3_des[N+1];
    float q4_des[N+1];
    float dq2_des[N+1];
    float dq3_des[N+1];
    float dq4_des[N+1];
    
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();
    
    // Continually get input from MATLAB and run experiments
    float input_params[NUM_INPUTS];
    pc.printf("%f",input_params[0]);
    
    while(1) {
        // If there are new inputs, this code will run
        if (server.getParams(input_params,NUM_INPUTS)) {   
            pc.printf("Receiving inputs\n");       
            // Get inputs from MATLAB   
            start_period                = input_params[0];      // First buffer time, before trajectory
            traj_period                 = input_params[1];      // Trajectory time/length
            end_period                  = input_params[2];      // Second buffer time, after trajectory

            angle1_init                 = input_params[3];      // Initial angel for q1 (body) (rad)
            angle2_init                 = input_params[4];      // Initial angle for q2 (hip) (rad)
            angle3_init                 = input_params[5];      // Initial angle for q3 (knee) (rad)
            angle4_init                 = input_params[6];      // Initial angle for q4 (shoulder) (rad)

            K_2                        = input_params[7];       // Foot stiffness N/m
            K_3                        = input_params[8];       // Foot stiffness N/m
            K_4                        = input_params[9];       // Foot stiffness N/m
            D_2                        = input_params[10];      // Foot damping N/(m/s)
            D_3                        = input_params[11];      // Foot damping N/(m/s)
            D_4                        = input_params[12];      // Foot damping N/(m/s)
            duty_max                   = input_params[13];      // Maximum duty factor
          
            // Get desired angles and angular velocities
            for(int i = 0; i<N+1;i++) {
              q2_des[i] = input_params[14+Nz*i];
              q3_des[i] = input_params[15+Nz*i]; 
              q4_des[i] = input_params[16+Nz*i]; 
              dq2_des[i] = input_params[17+Nz*(N+1)+Nz*i]; 
              dq3_des[i] = input_params[18+Nz*(N+1)+Nz*i]; 
              dq4_des[i] = input_params[19+Nz*(N+1)+Nz*i];

              pc.printf("q2_des idx %i: %f\n",i,q2_des[i]);   
            }
            // readTrajectoriesFromFile("./optimal_angles.csv",q2_des, q3_des, q4_des);
            // readTrajectoriesFromFile("./optimal_angular_velocities.csv",dq2_des, dq3_des, dq4_des);

            for (int i=0;i<N+1;i++){
                pc.printf("q4 des at %i: %f \n",i,q4_des[i]);

            }

            
            // Attach current loop interrupt
            currentLoop.attach_us(CurrentLoop,current_control_period_us);
                        
            // Setup experiment
            t.reset();
            t.start();
            encoderA.reset();
            encoderB.reset();
            encoderC.reset();
            encoderD.reset();

            motorShield.motorAWrite(0, 0); //turn motor A off
            motorShield.motorBWrite(0, 0); //turn motor B off
            motorShield.motorCWrite(0, 0); //turn motor C off
            pc.printf("Initialization complete\n");
                         
            float init_time = t.read();
            // Run experiment
            while( t.read() < start_period + traj_period + end_period) { 
                float t_start_loop = t.read();
                pc.printf("Control loop\n");
                // Read encoders to get motor states
                angle2 = encoderA.getPulses() *PULSE_TO_RAD + angle2_init;       
                velocity2 = encoderA.getVelocity() * PULSE_TO_RAD;
                 
                angle3 = encoderB.getPulses() * PULSE_TO_RAD + angle3_init;       
                velocity3 = encoderB.getVelocity() * PULSE_TO_RAD; 

                angle4 = encoderC.getPulses() * PULSE_TO_RAD + angle4_init;       
                velocity4 = encoderC.getVelocity() * PULSE_TO_RAD; 

                angle1 = encoderD.getPulses() * PULSE_TO_RAD + angle1_init;       
                velocity1 = encoderD.getVelocity() * PULSE_TO_RAD;    


                // Generalized coordinates
                const float q1 = angle1;
                const float q2 = angle2;
                const float q3 = angle3;
                const float q4 = angle4;
                const float dq1= velocity1;
                const float dq2= velocity2;
                const float dq3= velocity3;
                const float dq4= velocity4;
 

                // Set gains based on buffer and traj times, then calculate desired x,y from Bezier trajectory at current time if necessary
                float teff  = 0;

                if (t < start_period) {
                    if (K_2 > 0 || K_3 > 0 || K_4 > 0) {
                        K_2 = 1; // for joint space control, set these to 1; 
                        K_3 = 1; 
                        K_4 = 1; 
                        D_2 = 0.1;  // for joint space control, set these to 0.1; 
                        D_3 = 0.1;
                        D_4 = 0.1;  
                    }
                    teff = 0;
                }
                else if (t < start_period + traj_period) {
                    K_2                        = input_params[7];       // Foot stiffness N/m
                    K_3                        = input_params[8];       // Foot stiffness N/m
                    K_4                        = input_params[9];       // Foot stiffness N/m
                    D_2                        = input_params[10];      // Foot damping N/(m/s)
                    D_3                        = input_params[11];      // Foot damping N/(m/s)
                    D_4                        = input_params[12];      // Foot damping N/(m/s)
                    teff = (t-start_period);

                }
                else {
                    teff = traj_period;

                }
                
        
                // Calculate error variables
                int t_idx = (teff)/dt; // current timestep (between 0 and N?)
                if(t_idx > N) t_idx = N ; // ensure t_idx <=N
                float e_q2 = q2_des[t_idx] - q2;
                float e_q3 = q3_des[t_idx] - q3;
                float e_q4 = q4_des[t_idx] - q4;
                float e_dq2 = dq2_des[t_idx] - dq2;
                float e_dq3 = dq3_des[t_idx] - dq3;
                float e_dq4 = dq4_des[t_idx] - dq4;
                pc.printf("Computed errors:\nt: %f \nIdx: %i\nq2: %f\nq2_des: %f\n", t.read()-init_time, t_idx, q2,q2_des[t_idx]);
        
        
                // Joint impedance
                float tau2 = K_2*e_q2 + D_2*e_dq2;           
                float tau3 = K_3*e_q3 + D_3*e_dq3;           
                float tau4 = K_4*e_q4 + D_4*e_dq4; 

                current_des2 = -k_t*tau2; // opposite sign bc hip motor is located on other side of body compared to leg/shoulder motors
                current_des3 = k_t*tau3;          
                current_des4 = k_t*tau4;                         

                pc.printf("Current 2 %f\n", current_des2);
                


                // Form output to send to MATLAB     
                float output_data[NUM_OUTPUTS];
                // current time
                output_data[0] = t.read()-init_time;

                // body data
                output_data[1] = angle1;
                //output_data[2] = q1_des[t_idx];
                output_data[2] = velocity1; 
                //output_data[4] = dq1_des[t_idx]; 

                // hip data
                output_data[3] = angle2;
                output_data[4] = q2_des[t_idx];
                output_data[5] = velocity2; 
                output_data[6] = dq2_des[t_idx];

                // foot data
                output_data[7] = angle3;
                output_data[8] = q3_des[t_idx];
                output_data[9] = velocity3; 
                output_data[10] = dq3_des[t_idx];

                // arm data
                output_data[11] = angle4;
                output_data[12] = q4_des[t_idx];
                output_data[13] = velocity4; 
                output_data[14] = dq4_des[t_idx];

                // control inputs
                output_data[15] = tau2; 
                output_data[16] = tau3; 
                output_data[17] = tau4;
                
                
                // Send data to MATLAB
                server.sendData(output_data,NUM_OUTPUTS);
                pc.printf("Output sent\n\n");

                float t_end_loop = t.read();

                float wait_time = impedance_control_period_us-(t_end_loop-t_start_loop)*1e6;

                wait_us(wait_time);   
            }
            
            // Cleanup after experiment
            server.setExperimentComplete();
            currentLoop.detach();
            motorShield.motorAWrite(0, 0); //turn motor A off
            motorShield.motorBWrite(0, 0); //turn motor B off
            motorShield.motorCWrite(0, 0); //turn motor C off
        } // end if
    } // end while
} // end main

