#include <stdio.h>  
#include <stdlib.h>  
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h> 
#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/stat.h> 
#include <curses.h>
#include <unistd.h>

#include "vive.h"

//gcc -o week1 week1_student.cpp -lwiringPi -lncurses -lm
//gcc -o week1 week1_student.cpp -lwiringPi -lm

#define frequency 25000000.0
#define CONFIG           0x1A
#define SMPLRT_DIV       0x19
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C

// safety constants
#define MAX_GYRO_RATE 350 //was 300
#define MAX_ROLL_ANGLE 40 //was 45
#define MAX_PITCH_ANGLE 40 //was 45

//motor constants 
#define PWM_MAX 2000
#define THRUST_NEUTRAL 1500
#define THRUST_MAX 200 
#define MAX_PITCH_DESIRED 10
#define MAX_ROLL_DESIRED 10
#define MAX_YAW_RATE_DESIRED 125
 
#define frequency 25000000.0
#define LED0 0x6             
#define LED0_ON_L 0x6       
#define LED0_ON_H 0x7       
#define LED0_OFF_L 0x8      
#define LED0_OFF_H 0x9      
#define LED_MULTIPLYER 4

enum Ascale {
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
};

enum Gscale {
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
};  

int setup_imu();
void calibrate_imu();
void read_imu();
void update_filter();

//global variables
int imu;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6]; //gyro xyz, accel xyz
long time_curr;
long time_prev;
struct timespec te;
float pitch_angle=0;
float roll_angle=0;

//temp global variables for printing
float roll_angle_accel = 0;
float pitch_angle_accel = 0;

// new globals for gyro
float pitch_angle_gyro = 0;
float roll_angle_gyro = 0;

//Data Setup
//update shared memory struct

struct data {
  int keypress;
  int pitch;
  int roll;
  int yaw;
  int thrust;
  int sequence_num;
};
data* shared_memory; 
int run_program=1;
long prev_sequence_num_time = 0;
int prev_sequence_num = 0;



//motor variables 
int pwm;

float pitch_eint = 0;
float pitch_prev = 0;
float pitch_eprev = 0;

float roll_eint = 0;
float roll_prev = 0;
float roll_eprev = 0;

float prev_error_time = 0;

bool pause_motors = false;
bool calibrate_motors = false;

//VIVE data
Position local_p;
Position desired_p;
int last_vive_version = 0;
int last_vive_time = 0;

float vive_yaw = 0;

float vive_x_estimated = 0;
float prev_vive_x = 0;
float vive_roll = 0;
float vive_roll_diff = 0;
float vive_roll_velocity = 0;

float vive_y_estimated = 0;
float prev_vive_y = 0;
float vive_pitch = 0;
float vive_pitch_diff = 0;
float vive_pitch_velocity = 0;

float vive_z_estimated = 0;
float prev_vive_z = 0;
float vive_thrust = 0;
float vive_height_diff = 0;
float vive_height_velocity = 0;

//pid input variables
float P = 0;
float D = 0;
float I = 0;



/* ------ IMU DATA -------- */

void calibrate_imu()
{
    float x_gyro_temp, y_gyro_temp, z_gyro_temp, roll_temp, pitch_temp = 0;

    float cal_amount = 1000;
    for (int i = 0; i < cal_amount; i++) {
        read_imu();
        x_gyro_temp += imu_data[3] / cal_amount;
        y_gyro_temp += imu_data[4] / cal_amount;
        z_gyro_temp += imu_data[5] / cal_amount;
        
        roll_temp += roll_angle / cal_amount;
        pitch_temp += pitch_angle / cal_amount;
    }
    
    x_gyro_calibration = x_gyro_temp;
    y_gyro_calibration = y_gyro_temp;
    z_gyro_calibration = z_gyro_temp;
    roll_calibration = roll_temp;
    pitch_calibration = pitch_temp;

    //Set Vive Position Origin
    desired_p = local_p;

    //Pretty Print Calibration Values
    printf("Calibration Complete: (x_gyro, y_gyro, z_gyro, pitch, roll),  (%10.5f, %10.5f, %10.5f, %10.5f, %10.5f)\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,pitch_calibration,roll_calibration);
}

void read_imu()
{
    
    //Accelerometer
    int address= 59; //Accelerator X Value Raw Data
    float ax=0;
    float az=0;
    float ay=0;
    float g=9.8;
    int vh,vl;
    
    //read in data
    vh=wiringPiI2CReadReg8(imu,address);
    vl=wiringPiI2CReadReg8(imu,address+1);
    //convert 2 complement
    int vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
    if(vw>0x8000)
    {
        vw=vw ^ 0xffff;
        vw=-vw-1;
    }
    imu_data[3]= (float)vw / 32768 * (2*g) ;// Convert from Raw to "G's"
    
    
    address=61; //Accelerator Y Value Raw Data
    vh=wiringPiI2CReadReg8(imu,address);
    vl=wiringPiI2CReadReg8(imu,address+1);
    vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
    if(vw>0x8000)
    {
        vw=vw ^ 0xffff;
        vw=-vw-1;
    }
    imu_data[4]= (float)vw / 32768 * (2*g) ;// Convert from Raw to "G's"
    
    
    address=63; //Accelerator Z Value Raw Data
    vh=wiringPiI2CReadReg8(imu,address);
    vl=wiringPiI2CReadReg8(imu,address+1);
    vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
    if(vw>0x8000)
    {
        vw=vw ^ 0xffff;
        vw=-vw-1;
    }
    imu_data[5]= (float)vw / 32768 * (2*g); // Convert from Raw to "G's"
    

    //Gyro
    
    address= 67; //Gyro X Value Address
    vh=wiringPiI2CReadReg8(imu,address);
    vl=wiringPiI2CReadReg8(imu,address+1);
    vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
    if(vw>0x8000)
    {
        vw=vw ^ 0xffff;
        vw=-vw-1;
    }
    imu_data[0]= ((float)vw / 32768 * 500) - x_gyro_calibration; // Convert from Raw to deg/sec
    //printf("x gyro deg/s: %10.5f \n", imu_data[0]);
    
    address= 69; //Gyro Y Value Address
    vh=wiringPiI2CReadReg8(imu,address);
    vl=wiringPiI2CReadReg8(imu,address+1);
    vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
    if(vw>0x8000)
    {
        vw=vw ^ 0xffff;
        vw=-vw-1;
    }
    imu_data[1]= ((float)vw / 32768 * 500) - y_gyro_calibration; // Convert from Raw to deg/sec
    
    address= 71; //Gyro Z Value Address
    vh=wiringPiI2CReadReg8(imu,address);
    vl=wiringPiI2CReadReg8(imu,address+1);
    vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
    if(vw>0x8000)
    {
        vw=vw ^ 0xffff;
        vw=-vw-1;
    }
    imu_data[2]= ((float)vw / 32768 * 500) - z_gyro_calibration; // Convert from Raw to deg/sec    
    
}

void update_filter()
{
    
    //get current time in nanoseconds
    timespec_get(&te,TIME_UTC);
    time_curr=te.tv_nsec;
    //compute time since last execution
    float imu_diff=time_curr-time_prev;
    
    //check for rollover
    if(imu_diff<=0)
    {
        imu_diff+=1000000000; 
    }
    //convert to seconds
    imu_diff=imu_diff/1000000000; // delta t
    time_prev=time_curr;
      
    //Roll and pitch from accel data - roll (y-axis), pitch (x-axis)
    pitch_angle_accel = (-atan2( imu_data[4], -imu_data[5]) / 0.017453) - pitch_calibration; // angle of rotation x
    roll_angle_accel =  (atan2( imu_data[3], -imu_data[5]) / 0.017453) - roll_calibration; // angle of rot y
    
    //Change in Roll, Pitch, Yaw from integrated gyro
    float roll_gyro_delta = imu_data[1] * imu_diff;
    float pitch_gyro_delta = imu_data[0] * imu_diff;
    
    //Complimentary Filter for roll, pitch here:
    //Rollt+1=roll_accel*A+(1-A)*(roll_gyro_delta+ Rollt), //Where A << 1
    float A = 0.003; //5
    roll_angle = roll_angle_accel * A + (1-A) * (roll_gyro_delta + roll_angle);
    pitch_angle = pitch_angle_accel * A + (1-A) * (pitch_gyro_delta + pitch_angle);
    
}


int setup_imu()
{
    wiringPiSetup ();
    
    
    //setup imu on I2C
    imu=wiringPiI2CSetup (0x68) ; //accel/gyro address
    
    if(imu==-1)
    {
        printf("-----cant connect to I2C device %d --------\n",imu);
        return -1;
    }
    else
    {
        
        printf("connected to i2c device %d\n",imu);
        printf("imu who am i is %d \n",wiringPiI2CReadReg8(imu,0x75));
        
        uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
        uint8_t Gscale = GFS_500DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
        
        
        //init imu
        wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x00);
        printf("                    \n\r");
        wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x01);
        wiringPiI2CWriteReg8(imu, CONFIG, 0x00);
        wiringPiI2CWriteReg8(imu, SMPLRT_DIV, 0x00); //0x04
        int c=wiringPiI2CReadReg8(imu,  GYRO_CONFIG);
        wiringPiI2CWriteReg8(imu,  GYRO_CONFIG, c & ~0xE0);
        wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0x18);
        wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c | Gscale << 3);
        c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG);
        wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
        wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
        wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c | Ascale << 3);
        c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);
        wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2, c & ~0x0F); //
        wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2,  c | 0x00);
    }
    return 0;
}



/* ------ MOTOR INTERACTION -------- */

void init_pwm()
{

    pwm=wiringPiI2CSetup (0x40);
    if(pwm==-1)
    {
      printf("-----cant connect to I2C device %d --------\n",pwm);
     
    }
    else
    {
  
      float freq =400.0*.95;
      float prescaleval = 25000000;
      prescaleval /= 4096;
      prescaleval /= freq;
      prescaleval -= 1;
      uint8_t prescale = floor(prescaleval+0.5);
      int settings = wiringPiI2CReadReg8(pwm, 0x00) & 0x7F;
      int sleep = settings | 0x10;
      int wake  = settings & 0xef;
      int restart = wake | 0x80;
      wiringPiI2CWriteReg8(pwm, 0x00, sleep);
      wiringPiI2CWriteReg8(pwm, 0xfe, prescale);
      wiringPiI2CWriteReg8(pwm, 0x00, wake);
      delay(10);
      wiringPiI2CWriteReg8(pwm, 0x00, restart|0x20);
    }
}



void init_motor(uint8_t channel)
{
    int on_value=0;

    int time_on_us=900;
    uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

    wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
    delay(100);

     time_on_us=1200;
     off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

    wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
    delay(100);

     time_on_us=1000;
     off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

    wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
    delay(100);

}


void set_PWM( uint8_t channel, float time_on_us)
{
  if(run_program==1)
  {
    if(time_on_us>PWM_MAX)
    {
      time_on_us=PWM_MAX;
    }
    else if(time_on_us<1000)
    {
      time_on_us=1000;
    }
    uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
    wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }
  else
  {  
    time_on_us=1000;   
    uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
    wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }
}

void stop_motors() {
    set_PWM(0,0);
    set_PWM(1,0);
    set_PWM(2,0);
    set_PWM(3,0);
}


/* ----------- PID CONTROL ------------- */

float normalize_joystick_data(int js_value) {
    return -1 * ((((float)js_value - 16) / 224)*2 - 1);
}


void vive_update()
{


    //TODO: modify gamepad input to just change desire_p?? 
    //mixed autonomy doesn't make sense if the drone wants to resist change !!!!!!
    //note: for now, the purpose of the controller IS just to introduce noise. change it later.

    float P_vive_pitch = P; //0.04 
    float D_vive_pitch = D; //0.06 


    float P_vive_roll = P;
    float D_vive_roll = D;

    float P_vive_thrust = 0;
    float D_vive_thrust = 0;
    
    //Yaw angle P controller
    float desired_yaw = 0; //or desired_p.yaw (to keep it in start position instead of zero)
    float P_vive_yaw = 400;
    vive_yaw = P_vive_yaw * (local_p.yaw - desired_yaw);

    //Inital position for exponential filter
    if (vive_y_estimated == 0) {
        vive_y_estimated = local_p.y;
        vive_x_estimated = local_p.x;
        vive_z_estimated = local_p.z;
    }
    vive_y_estimated = vive_y_estimated * 0.6 + local_p.y * 0.4;
    vive_x_estimated = vive_x_estimated * 0.6 + local_p.x * 0.4; 
    vive_z_estimated = vive_z_estimated * 0.6 + local_p.z * 0.4; 

    //Update D only if there is new position information

    if ( last_vive_version == 0  || last_vive_version != local_p.version ) {

        timespec_get(&te,TIME_UTC);
        time_curr=te.tv_nsec;

        //compute time since last sequence_num
        float dt = time_curr - last_vive_time;
        
        //check for rollover and convert to seconds
        if (dt <= 0) dt += 1000000000;
        dt = dt / 1000000000;

        last_vive_time = te.tv_nsec;
        last_vive_version = local_p.version;

        //pitch values (lock y axis)
        vive_pitch_diff = (desired_p.y - vive_y_estimated);
        vive_pitch_velocity = (prev_vive_y - vive_y_estimated ) / dt;
        prev_vive_y = vive_y_estimated; //now set previous value for next computation

        //roll values (lock x axis)
        vive_roll_diff = (vive_x_estimated - desired_p.x);
        vive_roll_velocity = (prev_vive_x - vive_x_estimated) / dt;
        prev_vive_x = vive_x_estimated; 

        //thrust values (lock z axis) 
        vive_height_diff = (desired_p.z - vive_z_estimated);
        vive_height_velocity = (vive_z_estimated - prev_vive_z) / dt;
        prev_vive_z = vive_z_estimated; 

        //compute PD controller values for PWM modification
        vive_pitch = P_vive_pitch * vive_pitch_diff + D_vive_pitch * (vive_pitch_velocity);
        vive_roll = P_vive_roll * vive_roll_diff + D_vive_roll * (vive_roll_velocity);  
        vive_thrust = P_vive_thrust * vive_height_diff + D_vive_thrust * (vive_height_velocity);  
    } else {  
        //Vive Safety Timeout
        //get current time in nanoseconds
        timespec_get(&te,TIME_UTC);
        time_curr=te.tv_nsec;

        //compute time since last sequence_num
        float diff = time_curr - last_vive_time;
        
        //check for rollover and convert to seconds
        if (diff <= 0) diff += 1000000000;
        diff = diff / 1000000000;
        if (diff > 0.5) {
            printf("Vive Timeout - ending program\n\r");
            run_program=0;
        }
    } 
    
    //printf("Desired Values= x: %10.5f, y: %10.5f, z: %10.5f, yaw: %10.5f\n\r", desired_p.x, desired_p.y, desired_p.z, desired_p.yaw);
    printf("Vive Values= x: %10.5f, y: %10.5f, z: %10.5f, yaw: %10.5f\n\r", local_p.x, local_p.y, local_p.z, local_p.yaw);

}
void pid_update() 
{
    float thrust = THRUST_NEUTRAL + (THRUST_MAX * -1 * normalize_joystick_data(shared_memory->thrust) * 1); // + (vive_thrust * 0.5) + 

    if ( prev_error_time == 0) {
        timespec_get(&te,TIME_UTC);
        prev_error_time=te.tv_nsec;
    }
    //Get dt 
    timespec_get(&te,TIME_UTC);
    time_curr=te.tv_nsec;
    //compute time since last check
    float dt = time_curr - prev_error_time;
    //check for rollover and convert to seconds 
    if (dt <= 0) dt += 1000000000;
    dt = dt / 1000000000;
    prev_error_time=time_curr;
 
    //compute pitch PID control
    float pitch_P = 10;  //15  //noise, no overshoot 18 4 0.06     //overshoot, no noise 12 2 0.03
    float pitch_D = 1; //2 
    float pitch_I = 0.04; //0.04

    //desired pitch with mixed autonomy input from vive and gamepad
    float desired_pitch = (vive_pitch * 0.5) + (normalize_joystick_data(shared_memory->pitch) * MAX_PITCH_DESIRED * 0.5);

    float pitch_error = desired_pitch - pitch_angle;
    float pitch_velocity = (pitch_prev - pitch_angle) / dt;
    pitch_prev = pitch_angle;
    pitch_eint += pitch_error * pitch_I;

    //compute roll PID control
    float roll_P = 11; //17
    float roll_D = 1; //2
    float roll_I = 0.05; //0.05
    float desired_roll = (vive_roll * 0.5) + (normalize_joystick_data(shared_memory->roll) * MAX_ROLL_DESIRED * 0.5);
    float roll_error = desired_roll - roll_angle;
    float roll_velocity = (roll_prev - roll_angle) / dt;
    roll_prev = roll_angle;
    roll_eint += roll_error * roll_I;

    //compute yaw P control
    float yaw_P = 2.5;
    float yaw_diff = -imu_data[2];

    //compute combined and check boundaries
    float pitch_pwm = pitch_error * pitch_P  + pitch_velocity * pitch_D + pitch_eint;
    float roll_pwm = roll_error * roll_P  + roll_velocity * roll_D + roll_eint;
    float yaw_pwm = vive_yaw + yaw_diff * yaw_P; 

    float fr = thrust + pitch_pwm + roll_pwm + yaw_pwm; 
    float br = thrust - pitch_pwm + roll_pwm - yaw_pwm;
    float bl = thrust - pitch_pwm - roll_pwm + yaw_pwm;
    float fl = thrust + pitch_pwm - roll_pwm - yaw_pwm;

    if (fr > PWM_MAX) fr = PWM_MAX;
    if (fr < 1000) fr = 1000;
    if (fl > PWM_MAX) fl = PWM_MAX;
    if (fl < 1000) fl = 1000;
    if (br > PWM_MAX) br = PWM_MAX;
    if (br < 1000) br = 1000;
    if (bl > PWM_MAX) bl = PWM_MAX;
    if (bl < 1000) bl = 1000;

    set_PWM(0, fr); 
    set_PWM(3, fl); 
    set_PWM(1, br); 
    set_PWM(2, bl);
    
    //PWM printouts
    //printf("%10.5f\n", yaw_pwm);
    //printf("fr: %10.5f, fl: %10.5f, br: %10.5f, bl: %10.5f\n", fr, fl, br, bl);

}



/* ------ SAFETY -------- */

void setup_shared_data()
{

  int segment_id;   
  struct shmid_ds shmbuffer; 
  int segment_size; 
  const int shared_segment_size = 0x6400; 
  int smhkey = 33222;
  
  /* Allocate a shared memory segment.  */ 
  segment_id = shmget (smhkey, shared_segment_size, IPC_CREAT | 0666); 
  /* Attach the shared memory segment.  */ 
  shared_memory = (data*) shmat (segment_id, 0, 0); 
  //printf ("shared memory attached at address %p\n", shared_memory); 
  /* Determine the segment's size. */ 
  shmctl (segment_id, IPC_STAT, &shmbuffer); 
  segment_size  =               shmbuffer.shm_segsz; 
  //printf ("segment size: %d\n", segment_size); 
  /* Write a string to the shared memory segment.  */ 
  //sprintf (shared_memory, "test!!!!."); 

}


//when cntrl+c pressed, kill motors
void trap(int signal)
{
    printf("ending program\n\r");
    run_program=0;
    stop_motors();
}

void safety_check() 
{
    if (abs(imu_data[0]) > MAX_GYRO_RATE) {
        printf("X gyro speed over max rate - ending program\n\r");
        run_program=0;
    }
    if (abs(imu_data[1]) > MAX_GYRO_RATE) {
        printf("Y gyro speed over max rate - ending program\n\r");
        run_program=0;
    } 
    if (abs(imu_data[2]) > MAX_GYRO_RATE) {
        printf("Z gyro speed over max rate - ending program\n\r");
        run_program=0;
    } 
    if (abs(pitch_angle) > 45) {
        printf("Pitch Angle over maximum - ending program\n\r");
        run_program=0;
    }
    if (abs(roll_angle) > 45) {
        printf("Roll Angle over maximum - ending program\n\r");
        run_program=0;
    }
    if (shared_memory->keypress == 32) {
        printf("Kill Pressed - ending program\n\r");
        run_program=0;
    }
    
    
    //Gamepad Timeout
    if ( prev_sequence_num_time == 0  || prev_sequence_num != shared_memory->sequence_num ) {
        timespec_get(&te,TIME_UTC);
        prev_sequence_num_time=te.tv_nsec;
        prev_sequence_num = shared_memory->sequence_num;
    } else {
        //get current time in nanoseconds
        timespec_get(&te,TIME_UTC);
        time_curr=te.tv_nsec;

        //compute time since last sequence_num
        float diff = time_curr - prev_sequence_num_time;
        
        //check for rollover and convert to seconds
        if (diff <= 0) diff += 1000000000;
        diff = diff / 1000000000;
        if (diff > 0.9) {
            printf("Client/Server Timeout - ending program\n\r");
            run_program=0;
        }
    }

    //Pause + Unpause
    if (shared_memory->keypress == 33) {
        pause_motors = true;
    }
    if (shared_memory->keypress == 34) {
        pause_motors = false;
    }

    //Calibrate
    if (shared_memory->keypress == 35) {
        calibrate_motors = true;
    }

    //Vive Distance Shutdown
    if (abs(local_p.x) > 1300 || abs(local_p.y) > 1300 ) {
        printf("Vive Distance Out of Range - ending program\n\r");
        run_program=0;
    }
    

}



/* ------ MAIN -------- */

int main (int argc, char *argv[])
{   
    P = atof(argv[1]);
    //Y_max = atoi(argv[2]);
    D = atof(argv[2]);
    //I = atof(argv[3]);

    //Safety Setup
    setup_shared_data(); //human controllers
    init_shared_memory(); //vive data
    local_p = *position;

    signal(SIGINT, &trap);

    //Motor Setup
    init_pwm();
    init_motor(0);
    init_motor(1);
    init_motor(2);
    init_motor(3);
    delay(1000);

    //IMU Setup
    setup_imu();
    printf("Calibrating....");
    calibrate_imu();

    while(run_program == 1)
    {
        local_p = *position; 
    
        read_imu();
        update_filter();
        safety_check();
        
        if (calibrate_motors) {
            stop_motors();
            calibrate_imu();
            calibrate_motors = false;
        }
        if (pause_motors) {
            stop_motors();
        } else {
            vive_update();
            pid_update();
        }
        
    }
    stop_motors();

    return 0;
}
