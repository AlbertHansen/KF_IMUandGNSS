#include <iostream>
#include <vector>
#include <LinearAlgebra.h>
#include <IMUclass.h>
#include <Arduino_LSM6DS3.h>

constexpr struct Settings
{
  size_t BAUDRATE { 115200 };
} KF_SETTINGS;

std::vector<std::vector<float>> SampleAccGyro()
{
  std::vector<std::vector<float>> AccGyro { {0.f}, {0.f}, {0.f}, {0.f}, {0.f}, {0.f} };
  IMU.readAcceleration(AccGyro.at(0).at(0), AccGyro.at(1).at(0), AccGyro.at(2).at(0));
  IMU.readGyroscope(AccGyro.at(3).at(0), AccGyro.at(4).at(0), AccGyro.at(5).at(0));
  return AccGyro;
}


//-------------------------------------------------
// INITIAL state estimate, x_hat = [ position_x position_y position_z velocity_x velocity_y velocity_z]'
std::vector<std::vector<float>> x_hat {
    {0.f}, 
    {0.f}, 
    {0.f}, 
    {0.f}, 
    {0.f}, 
    {0.f}, 
    {0.f}, 
    {0.f}, 
    {0.f}};

// Control input vector (measurement), u = [acceleration_x acceleration_y acceleration_z]'
std::vector<std::vector<float>> u {
    {0.f}, 
    {0.f}, 
    {0.f}};

// State transition matrix, F
float dt = 0.01f;
std::vector<std::vector<float>> F {
    {1.f, 0.f, 0.f,  dt, 0.f, 0.f},
    {0.f, 1.f, 0.f, 0.f,  dt, 0.f},
    {0.f, 0.f, 1.f, 0.f, 0.f,  dt},
    {0.f, 0.f, 0.f, 1.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f, 1.f, 0.f},
    {0.f, 0.f, 0.f, 0.f, 0.f, 1.f}};

// Control matrix, G
std::vector<std::vector<float>> G {
    {0.5f * dt * dt,            0.f,            0.f},
    {           0.f, 0.5f * dt * dt,            0.f},
    {           0.f,            0.f, 0.5f * dt * dt},
    {            dt,            0.f,            0.f},
    {           0.f,             dt,            0.f},
    {           0.f,            0.f,             dt}};

// INITIAL uncertainty of estimate (covariance matrix) of the current state, P
std::vector<std::vector<float>> P { 
    {1.f, 0.f, 0.f, 0.f, 0.f, 0.f}, 
    {0.f, 1.f, 0.f, 0.f, 0.f, 0.f}, 
    {0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, 
    {0.f, 0.f, 0.f, 1.f, 0.f, 0.f}, 
    {0.f, 0.f, 0.f, 0.f, 1.f, 0.f}, 
    {0.f, 0.f, 0.f, 0.f, 0.f, 1.f}};

// process noise is a covariance matrix denoted by,  Q
std::vector<std::vector<float>> Q { 

    };

// Measurement matrix, H (used as state selection???)
 std::vector<std::vector<float>> H { 
    {1.f, 0.f, 0.f},
    {0.f, 1.f, 0.f},
    {0.f, 0.f, 1.f}};

// Identity matrix, I
std::vector<std::vector<float>> I_6x6 { 
    {1.f, 0.f, 0.f, 0.f, 0.f, 0.f}, 
    {0.f, 1.f, 0.f, 0.f, 0.f, 0.f}, 
    {0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, 
    {0.f, 0.f, 0.f, 1.f, 0.f, 0.f}, 
    {0.f, 0.f, 0.f, 0.f, 1.f, 0.f}, 
    {0.f, 0.f, 0.f, 0.f, 0.f, 1.f}};

// Measurement noise is a covariance matrix denoted by,  R (should be measured)
std::vector<std::vector<float>> R {
    {0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f}};


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
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.print("Gyro sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
}


IMUreader IMUobject;

void loop()
{
  // Initial Estimate is done when initializing x_hat and P

  // Predict (Time update)
    // 1. extrapolate the current state estimate to obtain a priori estimate for the next time step
      // x_hat = F * x_hat + G * u;
    x_hat = sum(MatrixProduct(F, x_hat), MatrixProduct(G, u));

    // 2. extrapolate the error covariance to obtain a priori estimate covariance
      // P = F * P * F' + Q;
    P = sum(MatrixProduct(F, MatrixProduct(P, transpose(F))), Q);

  // Update (Measurement update)
    // 0. get measurement
    std::vector<std::vector<float>> z = SampleAccGyro();

    // 1. compute the Kalman gain
      // K = P * H' * inv(H * P * H' + R);
    std::vector<std::vector<float>> K = MatrixProduct(P, MatrixProduct(transpose(H), inverse(sum(MatrixProduct(H, MatrixProduct(P, transpose(H))), R))));

    // 2. update estimate with measurement z
      // x_hat = x_hat + K * (z - H * x_hat);
    x_hat = sum(x_hat, MatrixProduct(K, diff(z, MatrixProduct(H, x_hat))));

    // 3. update the error covariance
      // P = (I - K * H) * P * (I - K * H)' + K * R * K';
    P = sum(MatrixProduct(diff(I_6x6, MatrixProduct(K, H)), MatrixProduct(P, transpose(diff(I_6x6, MatrixProduct(K, H))))), MatrixProduct(K, MatrixProduct(R, transpose(K))));

  // Print results
  printMatrix(x_hat);
}