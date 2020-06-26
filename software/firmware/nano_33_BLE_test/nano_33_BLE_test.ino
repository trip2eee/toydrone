//#include "typedef.h"
#include <ArduinoBLE.h>

BLEService controlService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE toydrone Service

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic controlCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

void setup()
{
  // put your setup code here, to run once:

  Serial.begin(9600);
  while(!Serial);
  
  // initialize the BLE hardware
  if(!BLE.begin())
  {
    Serial.println("starting BLE failed!");
  }

  BLE.setLocalName("ToyDrone");
  BLE.setDeviceName("ToyDrone");
  
  BLE.setAdvertisedService(controlService);
  controlService.addCharacteristic(controlCharacteristic);

  // add service
  BLE.addService(controlService);

  // set the initial value for the characeristic:
  controlCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.print("BLE Initialized.\n");
  
}

void loop()
{
  // put your main code here, to run repeatedly:
  
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central)
  {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    Serial.print("Stop advertise.\n");
    BLE.stopAdvertise();

    // while the central is still connected to peripheral:
    while (central.connected()) 
    {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (controlCharacteristic.written()) 
      {
        if (controlCharacteristic.value())
        {   // any value other than 0
          Serial.println("LED on");
        } 
        else
        {                              // a 0 value
          Serial.println(F("LED off"));          
        }
      }
    }

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
    BLE.end();
  }
  else
  {
    ;//Serial.print("waiting for connectin\n");
  }
  
  //Serial.print("test\n");

}
