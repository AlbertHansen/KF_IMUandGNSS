#include <iostream>
#include <vector>
#include <LinearAlgebra.h>
// #include "IMUReader.h"

constexpr struct Settings
{
  float TimeStep { 0.1 };
  size_t BAUDRATE { 115200 };
} KF_SETTINGS;

// Define system variables
std::vector<float> Acc {0.2f, 0.f, 0.f};
std::vector<float> Vel { 0.f, 0.f, 0.f};
std::vector<float> Pos { 0.f, 0.f, 0.f};

// Define measurement and process noise
float noise_meas    { 0.2f };
float noise_process { 0.1f };

// Define measurement and process covariance matrices
std::vector<std::vector<float>> R { // Measurement covariance matrix 
    {noise_meas * noise_meas, 0, 0},        
    {0, noise_meas * noise_meas, 0}, 
    {0, 0, noise_meas * noise_meas}};
std::vector<std::vector<float>> Q { // Process covariance matrix
    {noise_process * noise_process * KF_SETTINGS.TimeStep * KF_SETTINGS.TimeStep, 0, 0, noise_process*KF_SETTINGS.TimeStep, 0, 0},
    {0, noise_process * noise_process * KF_SETTINGS.TimeStep * KF_SETTINGS.TimeStep, 0, 0, noise_process*KF_SETTINGS.TimeStep, 0},  
    {0, 0, noise_process * noise_process * KF_SETTINGS.TimeStep * KF_SETTINGS.TimeStep, 0, 0, noise_process*KF_SETTINGS.TimeStep},  
    {noise_process*KF_SETTINGS.TimeStep, 0, 0, noise_process * noise_process, 0, 0},  
    {0, noise_process*KF_SETTINGS.TimeStep, 0, 0, noise_process * noise_process, 0},  
    {0, 0, noise_process*KF_SETTINGS.TimeStep, 0, 0, noise_process * noise_process}};

// Define state variables
std::vector<std::vector<float>> x_hat { 
    {Vel.at(0)}, 
    {Vel.at(1)}, 
    {Vel.at(2)}, 
    {Pos.at(0)}, 
    {Pos.at(1)}, 
    {Pos.at(2)}}; // State estimate [velocity_x, velocity_y, velocity_z, position_x, position_y, position_z]

std::vector<std::vector<float>> P { // State covariance matrix
    {1, 0, 0, 0, 0, 0},
    {0, 1, 0, 0, 0, 0},
    {0, 0, 1, 0, 0, 0},
    {0, 0, 0, 1, 0, 0},
    {0, 0, 0, 0, 1, 0},
    {0, 0, 0, 0, 0, 1}};

// Define measurement and process models
std::vector<std::vector<float>> H { // Measurement model
    {1, 0, 0, 0, 0, 0},
    {0, 1, 0, 0, 0, 0},
    {0, 0, 1, 0, 0, 0}};
std::vector<std::vector<float>> F { // Process model
    {1, 0, 0, KF_SETTINGS.TimeStep, 0, 0},
    {0, 1, 0, 0, KF_SETTINGS.TimeStep, 0},
    {0, 0, 1, 0, 0, KF_SETTINGS.TimeStep},
    {0, 0, 0, 1, 0, 0},
    {0, 0, 0, 0, 1, 0},
    {0, 0, 0, 0, 0, 1}};

// Initialize the filter
std::vector<std::vector<float>> z {{0}, {0}, {0}};  // Measurement vector, Initialize with zero measurement
std::vector<float> u = {Acc.at(0), Acc.at(1), Acc.at(2)};    // Control input vector
std::vector<std::vector<float>> I {  // Identity matrix
    {1, 0, 0, 0, 0, 0},
    {0, 1, 0, 0, 0, 0},
    {0, 0, 1, 0, 0, 0},
    {0, 0, 0, 1, 0, 0},
    {0, 0, 0, 0, 1, 0},
    {0, 0, 0, 0, 0, 1}};



void setup()
{
  Serial.begin(KF_SETTINGS.BAUDRATE);
  while(!Serial);
  // std::cout << "Velocity: " <<std:endl;
}

// init previous time
float t_prev = 0;

void loop()
{
  delay(1000);
  Serial.println("TEST DUKKER DET OP???");
  
  // std::cout << "Velocity: " <<std:endl;
  // Get current time
    float t_curr = t_prev + KF_SETTINGS.TimeStep; // getTime();

  
  // Calculate time step
    float dt = t_curr - t_prev;
  
  // Predict next state estimate and covariance
  std::vector<std::vector<float>> A {
      {1, 0, 0, dt, 0, 0},
      {0, 1, 0, 0, dt, 0},
      {0, 0, 1, 0, 0, dt},
      {0, 0, 0, 1, 0, 0},
      {0, 0, 0, 0, 1, 0},
      {0, 0, 0, 0, 0, 1}};  
  
  x_hat = MatrixProduct(F, x_hat);  
  
  P = sum(MatrixProduct(A, MatrixProduct(P, transpose(A))), Q);
  Serial.println("Check 0");

  // Get measurement
    // std::vector<std::vector<float>> z = getMeasurement();

  // Update state estimate and covariance

  std::vector<std::vector<float>> y = diff(z, MatrixProduct(H, x_hat));
  std::vector<std::vector<float>> S = sum(MatrixProduct(H, MatrixProduct(P, transpose(H))), R);
  std::vector<std::vector<float>> K = MatrixProduct(P, MatrixProduct(transpose(H), inverse(S)));
  x_hat = sum(x_hat, MatrixProduct(K, y));
  P = MatrixProduct(diff(I, MatrixProduct(K, H)), P);

  // Output state estimate
  //    std::cout << "Velocity: [" << x_hat(0) << ", " << x_hat(1) << ", " << x_hat(2) << "]" << std::endl;
  //    std::cout << "Position: [" << x_hat(3) << ", " << x_hat(4) << ", " << x_hat(5) << "]" << std::endl;

  // Update previous time
  t_prev = t_curr;
  
}
