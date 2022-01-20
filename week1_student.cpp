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

//gcc -o week1 week_1.cpp -lwiringPi -lncurses -lm
// gcc -o week1 week1_student.cpp -lwiringPi -lm

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
#define MIN_ROLL_ANGLE -45
#define MAX_PITCH_ANGLE 45
#define MIN_PITCH_ANGLE -45

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

// new globals for gyro
float pitch_angle_gyro = 0;
float roll_angle_gyro = 0;


int main (int argc, char *argv[])
{
    
    setup_imu();
    calibrate_imu();
    
    printf("did that!");
    
    while(1)
    {
        //sleep(1);
        read_imu();
        update_filter();
        
        //Pretty Print Relevant Info for Milestone 1
        //printf("IMU Values: x_gyro= %10.5f, y_gyro = %10.5f, z_gyro = %10.5f, pitch =  %10.5f, roll = %10.5f\n\r",imu_data[0],imu_data[1],imu_data[2], pitch_angle, roll_angle);
        
    }
    
    
    
    
}

void calibrate_imu()
{
    float x_gyro_temp = 0;
    float y_gyro_temp = 0;
    float z_gyro_temp = 0;
    float roll_temp = 0;
    float pitch_temp = 0;
    
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
    //Pretty Print Milestone 1 Calibration Values
    //printf("Calibration Values (x_gyro, y_gyro, z_gyro, pitch, roll),  (%10.5f, %10.5f, %10.5f, %10.5f, %10.5f)\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,pitch_calibration,roll_calibration);
    
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
    imu_data[0]= ((float)vw / 32768 * 250) - x_gyro_calibration; // Convert from Raw to deg/sec
    
    address= 69; //Gyro Y Value Address
    vh=wiringPiI2CReadReg8(imu,address);
    vl=wiringPiI2CReadReg8(imu,address+1);
    vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
    if(vw>0x8000)
    {
        vw=vw ^ 0xffff;
        vw=-vw-1;
    }
    imu_data[1]= ((float)vw / 32768 * 250) - y_gyro_calibration; // Convert from Raw to deg/sec
    
    address= 71; //Gyro Z Value Address
    vh=wiringPiI2CReadReg8(imu,address);
    vl=wiringPiI2CReadReg8(imu,address+1);
    vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
    if(vw>0x8000)
    {
        vw=vw ^ 0xffff;
        vw=-vw-1;
    }
    //TODO: Z gyro is not making any changes with the calibration when calibration is spot on
    imu_data[2]= ((float)vw / 32768 * 250) - z_gyro_calibration; // Convert from Raw to deg/sec
    //printf("z: vh  %d vl %d vw %d deg/sec %f \n\r", vh, vl, vw, imu_data[2]);
    
    
    //  //Finding Pitch and Roll from Accel Data
    //  pitch_angle = (-atan2( imu_data[4], -imu_data[5]) / 0.017453) - pitch_calibration; // angle of rotation x
    //  roll_angle =  (atan2( imu_data[3], -imu_data[5]) / 0.017453) - roll_calibration; // angle of rot y
    
    
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
    
    // roll (y-axis), pitch (x-axis)
    float A = 0.02;
    float pitch_angle_accel, roll_angle_accel;
    
    // roll and pitch from accel data
    pitch_angle_accel = (-atan2( imu_data[4], -imu_data[5]) / 0.017453) - pitch_calibration; // angle of rotation x
    roll_angle_accel =  (atan2( imu_data[3], -imu_data[5]) / 0.017453) - roll_calibration; // angle of rot y
    
    // roll and pitch from integrated gyro
    float roll_gyro_delta, pitch_gyro_delta;
    roll_gyro_delta = imu_data[1] * imu_diff;
    pitch_gyro_delta = imu_data[0] * imu_diff;
    
    roll_angle_gyro = roll_gyro_delta + roll_angle_gyro;  // integrated gyro
    pitch_angle_gyro = pitch_gyro_delta + pitch_angle_gyro;
    
    //comp. filter for roll, pitch here:
    //Rollt+1=roll_accel*A+(1-A)*(roll_gyro_delta+ Rollt), //Where A << 1 (try .02)
    roll_angle = roll_angle_accel * A + (1-A) * (roll_gyro_delta + roll_angle);
    pitch_angle = pitch_angle_accel * A + (1-A) * (pitch_gyro_delta + pitch_angle);
    
    // Print results for milestone 2
    //printf("%10.5f %10.5f %10.5f\n\r", roll_angle, roll_angle_accel, roll_angle_gyro);
    printf("%10.5f %10.5f %10.5f\n\r", pitch_angle, pitch_angle_accel, pitch_angle_gyro);
    
    
    

    
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

