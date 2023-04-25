#include <ArduinoIoTCloud.h>
#include <SNU.h>


void onLedChange();
/* ... */
bool led;
int seconds;
/* ... */
void initProperties() {
  ArduinoCloud.setThingId("13c87fe0-a6c9-4f6a-9123-49ce36fd7fa9");
  ArduinoCloud.addProperty(led, WRITE, ON_CHANGE, onLedChange);
  ArduinoCloud.addProperty(seconds, READ, ON_CHANGE);
}
/* ... */
WiFiConnectionHandler ArduinoIoTPreferredConnection("Lars VRKlo", "krisergrim");