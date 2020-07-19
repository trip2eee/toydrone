#include "typedef.h"
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include "mbed.h"

#include "ToyIMU.h"

BLEService controlService("180F"); // BLE toydrone Service

BLEByteCharacteristic  charaCommand("2A10", BLERead | BLEWrite);
BLEShortCharacteristic charaRoll("2A20", BLERead | BLEWrite);
BLEShortCharacteristic charaPitch("2A30", BLERead | BLEWrite);
BLEShortCharacteristic charaYaw("2A40", BLERead | BLEWrite);

mbed::Timer t;

mbed::Timer t_imu;
CToyIMU oToyIMU;

uint8_t g_u8Initialized = 0U;

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
  BLE.setAdvertisedService(controlService);

  controlService.addCharacteristic(charaCommand);
  controlService.addCharacteristic(charaRoll);
  controlService.addCharacteristic(charaPitch);
  controlService.addCharacteristic(charaYaw);


  BLE.addService(controlService);
  //charaCommand.writeValue(0);
  BLE.advertise();

  Serial.print("BLE Initialized.\n");

  // Initialize the IMU
  if (!IMU.begin())
  {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  
  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println("Hz");

  oToyIMU.Initialize();
  
  t.start();
}

void loop()
{  

  // put your main code here, to run repeatedly:
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable())
  {
    float ax, ay, az;   // G
    float wx, wy, wz;   // degree/sec
    float mx, my, mz;   // uT
    int dt; // delta T (us)
    
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(wx, wy, wz);
    IMU.readMagneticField(mx, my, mz);

    
    t.stop();
    dt = t.read_us();
    t.reset();
    t.start();

    t_imu.reset();
    t_imu.start();

    const float32_t arf32A[] = {ax, ay, -az};
    const float32_t arf32W[] = {DEG2RAD(wx), DEG2RAD(wy), -DEG2RAD(wz)};
    const float32_t arf32M[] = {-mx, my, -mz};
    const float32_t f32T = static_cast<float32_t>(dt) * 1e-6F;
    
    oToyIMU.Update(arf32A, arf32W, arf32M, f32T);
    
    float32_t arf32Angles[3U];
    oToyIMU.GetAngles(arf32Angles);

    arf32Angles[0] = RAD2DEG(arf32Angles[0]);
    arf32Angles[1] = RAD2DEG(arf32Angles[1]);
    arf32Angles[2] = RAD2DEG(arf32Angles[2]);
    
    t_imu.stop();
    int proc_time_imu = t_imu.read_us() / 1000;

#if 0
    Serial.print("proc_time_imu: ");
    Serial.print(proc_time_imu);
    Serial.print("ms\n");
#endif

#if 1
    Serial.print(ax);
    Serial.print('\t');
    Serial.print(ay);
    Serial.print('\t');
    Serial.print(az);
    Serial.print('\t');
    
    Serial.print(wx);
    Serial.print('\t');
    Serial.print(wy);
    Serial.print('\t');
    Serial.print(wz);
    Serial.print('\t');
    
    Serial.print(mx);
    Serial.print('\t');
    Serial.print(my);
    Serial.print('\t');
    Serial.print(mz);

    Serial.print('\t');
    Serial.print(dt);

    Serial.print("\t");
    Serial.print(arf32Angles[0]);
    Serial.print("\t");
    Serial.print(arf32Angles[1]);
    Serial.print("\t");
    Serial.print(arf32Angles[2]);
    Serial.print("\n");
#endif
  }

  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central)
  {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    //Serial.print("Stop advertise.\n");
    //BLE.stopAdvertise();

    unsigned char data[5];
    data[0] = 'h';
    charaCommand.writeValue(data[0]);
    
    // while the central is still connected to peripheral:
    while (central.connected()) 
    {
      //Serial.println("while");
      
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (charaCommand.written())
      {
        data[0] = charaCommand.value();

        for(int i = 0; i < 1; i++)
        {
          Serial.print(data[i]);
          Serial.print(", ");
        }
        
        
        if (data[0] == 'r')
        {   // any value other than 0
          charaCommand.writeValue(data[0]);
          Serial.println("LED on");
        } 
        else
        {                              // a 0 value
          Serial.println(F("LED off"));          
        }
      }

      if(charaRoll.written())
      {
        int16_t roll = charaRoll.value();
        Serial.print("roll: ");
        Serial.println(roll);
      }
    }

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
    //BLE.end();
  }
  else
  {
    ;//Serial.print("waiting for connectin\n");
  }
  
  //Serial.print("test\n");


}
