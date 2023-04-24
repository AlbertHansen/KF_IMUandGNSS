// www.moodle.aau.dk/mod/page/view.php?id=1535313
#include <IMUclass.h>

std::string s;


bool IMUReader::AddMeasurement(){
  float x { 0 };
  float y { 0 };
  float z { 0 };
  float theta { 0 };
  float phi { 0 };
  float psi { 0 };
  
  if (IMU.accelerationAvailable()) 
  {
    IMU.readAcceleration(x, y, z);
  }
  if (IMU.gyroscopeAvailable()) 
  {
    IMU.readGyroscope(theta, phi, psi);
  }
  Measurements.at(index).AccData = {x, y, z};
  Measurements.at(index).GyroData = {theta, phi, psi};
  return((index < N - 1) ? index++ : index = 0); // måske virker

  // The method must return true if the index is wrapped (starting again at 0). False otherwise.
  //return(index == 0 ? :)
};

void setup()
{
  Serial.begin(115200);
  while (!Serial);
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
  s.reserve(200);
}

IMUReader<200> ReadIMU;

void loop()
{

  ReadIMU.AddMeasurement();
  Console.Writeline(AccGyroMeasurement.at(index).AccData.AccX);

}