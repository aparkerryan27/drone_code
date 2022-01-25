#include <stdio.h>
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
#define MAX_GYRO_RATE 300
#define MAX_ROLL_ANGLE 45
#define MAX_PITCH_ANGLE 45

//motor constants 
#define PWM_MAX 1300
#define PWM_NEUTRAL 1100
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
float yaw=0;
float pitch_angle=0;
float roll_angle=0;

//temp global variables for printing
float roll_angle_accel = 0;
float pitch_angle_accel = 0;

// new globals for gyro
float pitch_angle_gyro = 0;
float roll_angle_gyro = 0;

//Keyboard Setup
struct Keyboard {
  char key_press;
  int heartbeat;
  int version;
};
Keyboard* shared_memory; 
int run_program=1;
long prev_heartbeat_time = 0;
int prev_heartbeat;



//motor variables 
int pwm;


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
    
    //Change in Roll and Pitch from integrated gyro
    float roll_gyro_delta = imu_data[1] * imu_diff;
    float pitch_gyro_delta = imu_data[0] * imu_diff;
    
    //Complimentary Filter for roll, pitch here:
    //Rollt+1=roll_accel*A+(1-A)*(roll_gyro_delta+ Rollt), //Where A << 1
    float A = 0.02;
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
      int sleep	= settings | 0x10;
      int wake 	= settings & 0xef;
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


/* ----------- PID CONTROL ------------- */

void pid_update(float pitch_reference = 0) 
{
    //args: pitch_reference is desired pitch
    float P = 7;

    float pitch_error = pitch_reference - pitch_angle;
    float pwm = pitch_error * P;

    if (pwm > (PWM_MAX - PWM_NEUTRAL)) pwm = (PWM_MAX - PWM_NEUTRAL);

    set_PWM(0, PWM_NEUTRAL + pwm);
    set_PWM(3, PWM_NEUTRAL + pwm);
    set_PWM(1, PWM_NEUTRAL - pwm);
    set_PWM(2, PWM_NEUTRAL - pwm);
    
    printf("%10.5f %10.5f %10.5f %10.5f\n", pitch_angle, pitch_angle_accel, (PWM_NEUTRAL + pwm), (PWM_NEUTRAL - pwm));

}



/* ------ SAFETY -------- */

void setup_keyboard()
{

  int segment_id;   
  struct shmid_ds shmbuffer; 
  int segment_size; 
  const int shared_segment_size = 0x6400; 
  int smhkey = 33222;
  
  /* Allocate a shared memory segment.  */ 
  segment_id = shmget (smhkey, shared_segment_size, IPC_CREAT | 0666); 
  /* Attach the shared memory segment.  */ 
  shared_memory = (Keyboard*) shmat (segment_id, 0, 0); 
  printf ("shared memory attached at address %p\n", shared_memory); 
  /* Determine the segment's size. */ 
  shmctl (segment_id, IPC_STAT, &shmbuffer); 
  segment_size  =               shmbuffer.shm_segsz; 
  printf ("segment size: %d\n", segment_size); 
  /* Write a string to the shared memory segment.  */ 
  //sprintf (shared_memory, "test!!!!."); 

}


//when cntrl+c pressed, kill motors
void trap(int signal)
{
    printf("ending program\n\r");
    run_program=0;
    set_PWM(0,0);
    set_PWM(1,0);
    set_PWM(2,0);
    set_PWM(3,0);
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
    if (shared_memory->key_press == ' ') {
        printf("Space Pressed - ending program\n\r");
        run_program=0;
    }
    
    /* 
    //Keyboard Timeout
    if ( prev_heartbeat_time == 0  || prev_heartbeat != shared_memory->heartbeat ) {
        timespec_get(&te,TIME_UTC);
        prev_heartbeat_time=te.tv_nsec;
        prev_heartbeat = shared_memory->heartbeat;
    } else {
        //get current time in nanoseconds
        timespec_get(&te,TIME_UTC);
        time_curr=te.tv_nsec;

        //compute time since last heartbeat
        float diff = time_curr - prev_heartbeat_time;
        
        //check for rollover and convert to seconds
        if (diff <= 0) diff += 1000000000;
        diff = diff / 1000000000;

        if (diff > 0.25) {
            printf("Keyboard Timeout - ending program\n\r");
            run_program=0;
        }
    } 
    */

}



/* ------ MAIN -------- */

int main (int argc, char *argv[])
{

    //Safety Setup
    setup_keyboard();
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
    calibrate_imu();
    printf("Calibrating....");
    
    pid_update();
    while(run_program == 1)
    {
        //sleep(1);
        read_imu();
        update_filter();
        safety_check();
        pid_update();
        //printf("IMU Values: x_gyro= %10.5f, y_gyro = %10.5f, z_gyro = %10.5f, pitch =  %10.5f, roll = %10.5f\n\r",imu_data[0],imu_data[1],imu_data[2], pitch_angle, roll_angle);
        
    }
    //turn off motors when program is ended
    set_PWM(0,0);
    set_PWM(1,0);
    set_PWM(2,0);
    set_PWM(3,0);

    return 0;
}