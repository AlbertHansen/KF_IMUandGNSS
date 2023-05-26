#include <ArduinoIoTCloud.h>
#include <SNU.h>




float vel_x;
float vel_y;
float yaw;
float acc_x;
float acc_y;
/* ... */
void initProperties() {
  ArduinoCloud.setThingId("08014ac0-7347-4638-a2e3-8f2a0efe0dca");
  //ArduinoCloud.addProperty(led, WRITE, ON_CHANGE, onLedChange);
  //ArduinoCloud.addProperty(seconds, READ, ON_CHANGE);
  ArduinoCloud.addProperty(vel_x, READ, ON_CHANGE);
  ArduinoCloud.addProperty(vel_y, READ, ON_CHANGE);
  ArduinoCloud.addProperty(yaw, READ, ON_CHANGE);
  ArduinoCloud.addProperty(acc_x, READ, ON_CHANGE);
  ArduinoCloud.addProperty(acc_y, READ, ON_CHANGE);

}
/* ... */
//WiFiConnectionHandler ArduinoIoTPreferredConnection("Lars VRKlo", "krisergrim");
WiFiConnectionHandler ArduinoIoTPreferredConnection("Kelvin", "Celsius16");