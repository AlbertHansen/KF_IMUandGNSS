#include <iostream>
#include <vector>
#include <LinearAlgebra.h>
#include <IMUclass.h>
#include <Arduino_LSM6DS3.h>
#include "thingProperties.h"

constexpr struct Settings
{
  size_t BAUDRATE { 115200 };
} KF_SETTINGS;

//-------------------------------------------------
// INITIAL state estimate, x_hat = [ position_x velocity_x acceleration_x position_y velocity_y acceleration_y]'
std::vector<std::vector<float>> x_hat {
    {0.f}, 
    {0.f}, 
    {0.f}, 
    {0.f}, 
    {0.f}, 
    {0.f}};


// State transition matrix, F
float dt = 0.018f;
std::vector<std::vector<float>> F {
    {1.f,  dt, 0.5f*dt*dt, 0.f, 0.f,       0.f},
    {0.f, 1.f,        dt, 0.f, 0.f,        0.f},
    {0.f, 0.f,       1.f, 0.f, 0.f,        0.f},
    {0.f, 0.f,       0.f, 1.f,  dt, 0.5f*dt*dt},
    {0.f, 0.f,       0.f, 0.f, 1.f,         dt},
    {0.f, 0.f,       0.f, 0.f, 0.f,        1.f}};

// INITIAL uncertainty of estimate (covariance matrix) of the current state, P
std::vector<std::vector<float>> P { 
    {1.f, 1.f, 1.f, 0.f, 0.f, 0.f}, 
    {1.f, 1.f, 1.f, 0.f, 0.f, 0.f}, 
    {1.f, 1.f, 1.f, 0.f, 0.f, 0.f}, 
    {0.f, 0.f, 0.f, 1.f, 1.f, 1.f,}, 
    {0.f, 0.f, 0.f, 1.f, 1.f, 1.f,}, 
    {0.f, 0.f, 0.f, 1.f, 1.f, 1.f}};

// process noise is a covariance matrix denoted by,  Q (OBS initiated as zero)

std::vector<std::vector<float>> Q {
    {0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}};

// Measurement matrix, H (used as state selection)
 std::vector<std::vector<float>> H { 
    {0.f, 0.f, 1.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f, 0.f, 1.f}};

// Measurement noise is a covariance matrix denoted by,  R
std::vector<std::vector<float>> R {
    {0.1f,  0.f },
    {0.f ,  0.1f}};

// Identity matrix, I
std::vector<std::vector<float>> I_6x6 { 
    {1.f, 0.f, 0.f, 0.f, 0.f, 0.f}, 
    {0.f, 1.f, 0.f, 0.f, 0.f, 0.f}, 
    {0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, 
    {0.f, 0.f, 0.f, 1.f, 0.f, 0.f}, 
    {0.f, 0.f, 0.f, 0.f, 1.f, 0.f}, 
    {0.f, 0.f, 0.f, 0.f, 0.f, 1.f}};

//-------------------------------------------------

void setup()
{
  Serial.begin(KF_SETTINGS.BAUDRATE);
  while(!Serial);
  if (!IMU.begin()) 
    {
      Serial.println("Failed to initialize IMU! Halting.");
      while (1);
    }
  initProperties(); // IOT thing
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.print("Gyro sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
    Serial.println("_____ P _____");
    printMatrix(P);
    Serial.println("_____ Q _____");
    printMatrix(Q);
    Serial.println("_____ F _____");
    printMatrix(F);
    Serial.println("_____ H _____");
    printMatrix(H);
    Serial.println("_____ R _____");
    printMatrix(R);
}

IMUreader IMUobject;

// KALMAN FILTER LOOP
void loop()
{
  // Initial Estimate is done when initializing x_hat and P
  // Predict (Time update)
    // 1. extrapolate the current state estimate to obtain a priori estimate for the next time step
      // x_hat = F * x_hat + G * u; changed to x_hat = F * x_hat
    x_hat = MatrixProduct(F, x_hat);

    // 2. extrapolate the error covariance to obtain a priori estimate covariance
      // P = F * P * F' + Q;
    P = sum(MatrixProduct(F, MatrixProduct(P, transpose(F))), Q);
    // Serial.println("_____________");
    // printMatrix(P);

  // Update (Measurement update)
    // 0. get measurement
    IMUobject.MakeMeasurements();
    std::vector<std::vector<float>> Acc = IMUobject.GetAccMean();
    std::vector<std::vector<float>> x = {
        {0},
        {0},
        {Acc.at(0).at(0)}, 
        {0},
        {0},
        {Acc.at(1).at(0)}};
    std::vector<std::vector<float>> z = MatrixProduct(H, x);
    
    // -------------- print measurement -----------------------//
    Serial.print(z.at(0).at(0), 12);
    Serial.print(", ");
    Serial.print(z.at(1).at(0), 12);
    Serial.print(", ");
    Serial.println(IMUobject.GetYaw(), 12);
    // --------------------------------------------------------//
    
    // 1. compute the Kalman gain
      // K = P * H' * inv(H * P * H' + R);
    std::vector<std::vector<float>> K = MatrixProduct(P, MatrixProduct(transpose(H), inverse(sum(MatrixProduct(H, MatrixProduct(P, transpose(H))), R))));
    // Serial.println(K.at(0).at(0), 12);

    // 2. update estimate with measurement z
      // x_hat = x_hat + K * (z - H * x_hat);
    x_hat = sum(x_hat, MatrixProduct(K, diff(z, MatrixProduct(H, x_hat))));

    // 3. update the error covariance
      // P = (I - K * H) * P * (I - K * H)' + K * R * K';
    P = sum(MatrixProduct(diff(I_6x6, MatrixProduct(K, H)), MatrixProduct(P, transpose(diff(I_6x6, MatrixProduct(K, H))))), MatrixProduct(K, MatrixProduct(R, transpose(K))));


  vel_x = x_hat.at(1).at(0);
  vel_y = x_hat.at(4).at(0);
  acc_x = x_hat.at(2).at(0);
  acc_y = x_hat.at(5).at(0);
  yaw = IMUobject.GetYaw();
  ArduinoCloud.update();  
  //-------------------- Print results --------------------------//
  /*
  for(size_t i = 0; i < x_hat.size(); i++)
  {
    Serial.print(x_hat.at(i).at(0), 12);
    Serial.print(", ");
  }
  
  Serial.print(K.at(0).at(0), 12);
  Serial.print(", ");
  Serial.print(K.at(1).at(0), 12);
  Serial.print(", ");
  Serial.print(IMUobject.GetYaw(), 12);
  Serial.println();
  */
  //-------------------------------------------------------------//
}
