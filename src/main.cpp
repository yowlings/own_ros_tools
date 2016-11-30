/**
 * IMU filter example.
 *
 * Calculate the roll, pitch and yaw angles.
 */
#include "IMUfilter.h"
#include "ADXL345.h"
#include "ITG3200.h"

//Gravity at Earth's surface in m/s/s
#define g0 9.812865328
//Number of samples to average.
#define SAMPLES 4
//Number of samples to be averaged for a null bias calculation
//during calibration.
#define CALIBRATION_SAMPLES 128
//Convert from radians to degrees.
#define toDegrees(x) (x * 57.2957795)
//Convert from degrees to radians.
#define toRadians(x) (x * 0.01745329252)
//ITG-3200 sensitivity is 14.375 LSB/(degrees/sec).
#define GYROSCOPE_GAIN (1 / 14.375)
//Full scale resolution on the ADXL345 is 4mg/LSB.
#define ACCELEROMETER_GAIN (0.004 * g0)
//Sampling gyroscope at 200Hz.
#define GYRO_RATE   0.005
//Sampling accelerometer at 200Hz.
#define ACC_RATE    0.005
//Updating filter at 40Hz.
#define FILTER_RATE 0.1

Serial pc(USBTX, USBRX);
//At rest the gyroscope is centred around 0 and goes between about
//-5 and 5 counts. As 1 degrees/sec is ~15 LSB, error is roughly
//5/15 = 0.3 degrees/sec.
IMUfilter imuFilter(FILTER_RATE, 0.3);
ADXL345 accelerometer(p5, p6, p7, p8);
ITG3200 gyroscope(p9, p10);
Ticker accelerometerTicker;
Ticker gyroscopeTicker;
Ticker filterTicker;

//Offsets for the gyroscope.
//The readings we take when the gyroscope is stationary won't be 0, so we'll
//average a set of readings we do get when the gyroscope is stationary and
//take those away from subsequent readings to ensure the gyroscope is offset
//or "biased" to 0.
double w_xBias;
double w_yBias;
double w_zBias;

//Offsets for the accelerometer.
//Same as with the gyroscope.
double a_xBias;
double a_yBias;
double a_zBias;

//Accumulators used for oversampling and then averaging.
volatile double a_xAccumulator = 0;
volatile double a_yAccumulator = 0;
volatile double a_zAccumulator = 0;
volatile double w_xAccumulator = 0;
volatile double w_yAccumulator = 0;
volatile double w_zAccumulator = 0;

//Accelerometer and gyroscope readings for x, y, z axes.
volatile double a_x;
volatile double a_y;
volatile double a_z;
volatile double w_x;
volatile double w_y;
volatile double w_z;

//Buffer for accelerometer readings.
int readings[3];
//Number of accelerometer samples we're on.
int accelerometerSamples = 0;
//Number of gyroscope samples we're on.
int gyroscopeSamples = 0;

/**
 * Prototypes
 */
//Set up the ADXL345 appropriately.
void initializeAcceleromter(void);
//Calculate the null bias.
void calibrateAccelerometer(void);
//Take a set of samples and average them.
void sampleAccelerometer(void);
//Set up the ITG3200 appropriately.
void initializeGyroscope(void);
//Calculate the null bias.
void calibrateGyroscope(void);
//Take a set of samples and average them.
void sampleGyroscope(void);
//Update the filter and calculate the Euler angles.
void filter(void);

void initializeAccelerometer(void) {

    //Go into standby mode to configure the device.
    accelerometer.setPowerControl(0x00);
    //Full resolution, +/-16g, 4mg/LSB.
    accelerometer.setDataFormatControl(0x0B);
    //200Hz data rate.
    accelerometer.setDataRate(ADXL345_200HZ);
    //Measurement mode.
    accelerometer.setPowerControl(0x08);
    //See http://www.analog.com/static/imported-files/application_notes/AN-1077.pdf
    wait_ms(22);

}

void sampleAccelerometer(void) {

    //Have we taken enough samples?
    if (accelerometerSamples == SAMPLES) {

        //Average the samples, remove the bias, and calculate the acceleration
        //in m/s/s.
        a_x = ((a_xAccumulator / SAMPLES) - a_xBias) * ACCELEROMETER_GAIN;
        a_y = ((a_yAccumulator / SAMPLES) - a_yBias) * ACCELEROMETER_GAIN;
        a_z = ((a_zAccumulator / SAMPLES) - a_zBias) * ACCELEROMETER_GAIN;

        a_xAccumulator = 0;
        a_yAccumulator = 0;
        a_zAccumulator = 0;
        accelerometerSamples = 0;

    } else {
        //Take another sample.
        accelerometer.getOutput(readings);

        a_xAccumulator += (int16_t) readings[0];
        a_yAccumulator += (int16_t) readings[1];
        a_zAccumulator += (int16_t) readings[2];

        accelerometerSamples++;

    }

}

void calibrateAccelerometer(void) {

    a_xAccumulator = 0;
    a_yAccumulator = 0;
    a_zAccumulator = 0;
    
    //Take a number of readings and average them
    //to calculate the zero g offset.
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {

        accelerometer.getOutput(readings);

        a_xAccumulator += (int16_t) readings[0];
        a_yAccumulator += (int16_t) readings[1];
        a_zAccumulator += (int16_t) readings[2];

        wait(ACC_RATE);

    }

    a_xAccumulator /= CALIBRATION_SAMPLES;
    a_yAccumulator /= CALIBRATION_SAMPLES;
    a_zAccumulator /= CALIBRATION_SAMPLES;

    //At 4mg/LSB, 250 LSBs is 1g.
    a_xBias = a_xAccumulator;
    a_yBias = a_yAccumulator;
    a_zBias = (a_zAccumulator - 250);

    a_xAccumulator = 0;
    a_yAccumulator = 0;
    a_zAccumulator = 0;

}

void initializeGyroscope(void) {

    //Low pass filter bandwidth of 42Hz.
    gyroscope.setLpBandwidth(LPFBW_42HZ);
    //Internal sample rate of 200Hz. (1kHz / 5).
    gyroscope.setSampleRateDivider(4);

}

void calibrateGyroscope(void) {

    w_xAccumulator = 0;
    w_yAccumulator = 0;
    w_zAccumulator = 0;

    //Take a number of readings and average them
    //to calculate the gyroscope bias offset.
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {

        w_xAccumulator += gyroscope.getGyroX();
        w_yAccumulator += gyroscope.getGyroY();
        w_zAccumulator += gyroscope.getGyroZ();
        wait(GYRO_RATE);

    }

    //Average the samples.
    w_xAccumulator /= CALIBRATION_SAMPLES;
    w_yAccumulator /= CALIBRATION_SAMPLES;
    w_zAccumulator /= CALIBRATION_SAMPLES;

    w_xBias = w_xAccumulator;
    w_yBias = w_yAccumulator;
    w_zBias = w_zAccumulator;

    w_xAccumulator = 0;
    w_yAccumulator = 0;
    w_zAccumulator = 0;

}

void sampleGyroscope(void) {

    //Have we taken enough samples?
    if (gyroscopeSamples == SAMPLES) {

        //Average the samples, remove the bias, and calculate the angular
        //velocity in rad/s.
        w_x = toRadians(((w_xAccumulator / SAMPLES) - w_xBias) * GYROSCOPE_GAIN);
        w_y = toRadians(((w_yAccumulator / SAMPLES) - w_yBias) * GYROSCOPE_GAIN);
        w_z = toRadians(((w_zAccumulator / SAMPLES) - w_zBias) * GYROSCOPE_GAIN);

        w_xAccumulator = 0;
        w_yAccumulator = 0;
        w_zAccumulator = 0;
        gyroscopeSamples = 0;

    } else {
        //Take another sample.
        w_xAccumulator += gyroscope.getGyroX();
        w_yAccumulator += gyroscope.getGyroY();
        w_zAccumulator += gyroscope.getGyroZ();

        gyroscopeSamples++;

    }

}

void filter(void) {

    //Update the filter variables.
    imuFilter.updateFilter(w_y, w_x, w_z, a_y, a_x, a_z);
    //Calculate the new Euler angles.
    imuFilter.computeEuler();

}

int main() {

    pc.printf("Starting IMU filter test...\n");

    //Initialize inertial sensors.
    initializeAccelerometer();
    calibrateAccelerometer();
    initializeGyroscope();
    calibrateGyroscope();


    //Set up timers.
    //Accelerometer data rate is 200Hz, so we'll sample at this speed.
    accelerometerTicker.attach(&sampleAccelerometer, 0.005);
    //Gyroscope data rate is 200Hz, so we'll sample at this speed.
    gyroscopeTicker.attach(&sampleGyroscope, 0.005);
    //Update the filter variables at the correct rate.
    filterTicker.attach(&filter, FILTER_RATE);
   
    while (1) {

        wait(FILTER_RATE);

        
        pc.printf("%f,%f,%f\n",toDegrees(imuFilter.getRoll()),
                  toDegrees(imuFilter.getPitch()),
                  toDegrees(imuFilter.getYaw()));
         

    }

}
