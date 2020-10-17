#include "typedef.h"
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

#include "mbed.h"

#include "ToyIMU.h"

// Definitions
#define _ENABLE_UART 0

#if _ENABLE_UART == 1
#define _DEBUG_PRINT 1
#define _DEBUG_IMU   0
#define _DEBUG_BLE   1
#else
#define _DEBUG_PRINT 0
#define _DEBUG_IMU   0
#define _DEBUG_BLE   0
#endif


#define NUM_MOTORS 4U

BLEService controlService("180F"); // BLE toydrone Service

BLEByteCharacteristic  charaCommand("2A10", BLERead | BLEWrite);
BLEShortCharacteristic charaRoll("2A20", BLERead | BLEWrite);
BLEShortCharacteristic charaPitch("2A30", BLERead | BLEWrite);
BLEShortCharacteristic charaYaw("2A40", BLERead | BLEWrite);
BLEShortCharacteristic charaZ("2A50", BLERead | BLEWrite);

// Global variables.
mbed::Timer t;
mbed::Timer t_imu;

CToyIMU oToyIMU;

uint8_t g_u8Initialized = 0U;

int g_Time_10us = 0;
uint8_t g_aru8DutyRatio[NUM_MOTORS] = {0U, 0U, 0U, 0U};

int g_s32TargetZ = 0;   // target height z. unit:mm, range:0~2000, default:0

uint8_t g_u8PWMCount = 0U;

extern "C"
{ 
  void TIMER3_IRQHandler_v(void)
  {
    if (NRF_TIMER3->EVENTS_COMPARE[0] != 0)
    {
      NRF_TIMER3->EVENTS_COMPARE[0] = 0;    
      g_Time_10us ++;


      if(g_u8PWMCount == 255U)
      {
        g_u8PWMCount = 0U;
#if 1
        if(g_aru8DutyRatio[0] > g_u8PWMCount)
        {
          // rotor forward, left.
          digitalWrite(4, LOW);
          digitalWrite(5, HIGH);    // H
          //g_Time_10us ++;
        }
        
        if(g_aru8DutyRatio[1] > g_u8PWMCount)
        {
            // rotor backward, left
            digitalWrite(6, HIGH);  // H
            digitalWrite(7, LOW);
            //g_Time_10us ++;
        }
        
        if(g_aru8DutyRatio[2] > g_u8PWMCount)
        {
            // rotor forward, right
            digitalWrite(8, HIGH); // H
            digitalWrite(9, LOW);
            //g_Time_10us ++;
        }
        
        if(g_aru8DutyRatio[3] > g_u8PWMCount)
        {
          // rotor backward, right
          digitalWrite(10, LOW);
          digitalWrite(11, HIGH);    // H
          //g_Time_10us ++;
        }
#endif
      }
      else
      {
        g_u8PWMCount++;

#if 1
        if(g_aru8DutyRatio[0] == g_u8PWMCount)
        {
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);    // H
          //g_Time_10us ++;
        }
  
        if(g_aru8DutyRatio[1] == g_u8PWMCount)      
        {
            digitalWrite(6, LOW);  // H
            digitalWrite(7, LOW);
            //g_Time_10us ++;
        }
  
        if(g_aru8DutyRatio[2] == g_u8PWMCount)      
        {
            digitalWrite(8, LOW); // H
            digitalWrite(9, LOW);
            //g_Time_10us ++;
        }
  
        if(g_aru8DutyRatio[3] == g_u8PWMCount)      
        {
          digitalWrite(10, LOW);
          digitalWrite(11, LOW);    // H
          //g_Time_10us ++;
        }
#endif        
      }
      
    }
    else
    {
      ;   // no statement.
    }
  }
}

void config_timer(void)
{
  //Serial.print("TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos: "); 
  //Serial.println(TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos); 
  
  NRF_TIMER3->TASKS_STOP = 1;                 // Stop timer
  
  //delay(10);
  NRF_TIMER3->TASKS_CLEAR = 1;    // Clear timer
  NVIC_SetPriority(TIMER3_IRQn, 1);
    
  //NRF_TIMER3->TASKS_SHUTDOWN = 1;

  NRF_TIMER3->MODE = TIMER_MODE_MODE_Timer;   // taken from Nordic dev zone. 0: timer mode, 1: counter mode, 2: low power counter mode.
  NRF_TIMER3->BITMODE = (TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos);   // Configure the number of bits used by the timer (0:16-bit, 1:8-bit, 2:24-bit, 3:32-bit).
  NRF_TIMER3->PRESCALER = 4;  // 1/16   Timer prescaler register. (2^n, n=0..9)
  
  NRF_TIMER3->CC[0] = 10; // 10 us.
  
  NRF_TIMER3->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos) & TIMER_INTENSET_COMPARE0_Msk;  // Enable interrupt. taken from Nordic dev zone
  NRF_TIMER3->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos) & TIMER_SHORTS_COMPARE0_CLEAR_Msk;    // Shortcuts between local events and tasks.  

  NRF_TIMER3->EVENTS_COMPARE[0] = 0;  // Compare event on CC[n] match. (n=0..5)
  
  //delay(10);
  NRF_TIMER3->TASKS_START = 1;  // Start TIMER

  NVIC_EnableIRQ(TIMER3_IRQn);
}

void config_BLE()
{
  // initialize the BLE hardware
  if(!BLE.begin())
  {
#if _DEBUG_BLE == 1      
    Serial.println("starting BLE failed!");
#endif    
  }

  BLE.setLocalName("ToyDrone");
  BLE.setAdvertisedService(controlService);

  controlService.addCharacteristic(charaCommand);
  controlService.addCharacteristic(charaRoll);
  controlService.addCharacteristic(charaPitch);
  controlService.addCharacteristic(charaYaw);
  controlService.addCharacteristic(charaZ);

  BLE.addService(controlService);
  //charaCommand.writeValue(0);
  BLE.advertise();

  
#if _DEBUG_BLE == 1
  Serial.print("BLE Initialized.\n");
#endif
}

void setup()
{
  // put your setup code here, to run once:
  pinMode(4, OUTPUT) ;
  pinMode(5, OUTPUT) ;
  pinMode(6, OUTPUT) ;
  pinMode(7, OUTPUT) ;
  pinMode(8, OUTPUT) ;
  pinMode(9, OUTPUT) ;
  pinMode(10, OUTPUT) ;
  pinMode(11, OUTPUT) ;

#if _ENABLE_UART == 1  
  Serial.begin(9600);
  while(!Serial);

  Serial.println("Toydrone");
#endif
    
  config_BLE();

  // Initialize the IMU
  if (!IMU.begin())
  {
#if _DEBUG_IMU == 1      
    Serial.println("Failed to initialize IMU!");
#endif
    while (1);
  }

#if _DEBUG_IMU == 1
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  
  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println("Hz");
#endif

  oToyIMU.Initialize();

  t.start();

  
  // turn off the rotors.
  // rotor forward, left.
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);    // H

  // rotor backward, left
  digitalWrite(6, LOW);  // H
  digitalWrite(7, LOW);

  // rotor forward, right
  digitalWrite(8, LOW); // H
  digitalWrite(9, LOW);

  // rotor backward, right
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);    // H

  
  config_timer();

}

void loop()
{
  
#if 0
  Serial.println(g_Time_10us);
  delay(1000);
#endif
  
  //Serial.println(NRF_TIMER3->EVENTS_COMPARE[0]);

  // put your main code here, to run repeatedly:
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable())
  {
    Serial.println(g_Time_10us);
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

#if _DEBUG_IMU == 1
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
  // while the central is still connected to peripheral:
  if (central && central.connected())
  {
    unsigned char data[5];
    data[0] = 'h';
    //charaCommand.writeValue(data[0]);
      

    //Serial.println("while");
    
    // if the remote device wrote to the characteristic,
    // use the value to control the LED:
    if (charaCommand.written())
    {
      data[0] = charaCommand.value();

#if _DEBUG_BLE == 1
      Serial.print(data[0]);
      Serial.print(", ");
#endif
      
      if (data[0] == 'r')
      {   // any value other than 0
        charaCommand.writeValue(data[0]);
#if _DEBUG_BLE == 1      
        Serial.println("LED on");
#endif        
      } 
      else
      {
#if _DEBUG_BLE == 1
        Serial.println(F("LED off"));
#endif
      }
    }

    if(charaRoll.written())
    {
      int16_t roll = charaRoll.value();
#if _DEBUG_BLE == 1      
      Serial.print("roll: ");
      Serial.println(roll);
#endif      
    }

    if(charaZ.written())
    {
      g_s32TargetZ = static_cast<int32_t>(charaZ.value());
#if _DEBUG_BLE == 1
      Serial.print("z: ");
      Serial.println(g_s32TargetZ);
#endif

      NRF_TIMER3->TASKS_STOP = 1;                 // Stop timer
      for(uint8_t u8IdxMotor = 0U; u8IdxMotor < NUM_MOTORS; u8IdxMotor++)
      {
        g_aru8DutyRatio[u8IdxMotor] = static_cast<uint8_t>(g_s32TargetZ);
      }
      NRF_TIMER3->TASKS_START = 1;  // Start TIMER
/*
      analogWrite(4, 0);
      analogWrite(5, g_aru8DutyRatio[0]);    // H
     
      analogWrite(6, g_aru8DutyRatio[1]);  // H
      analogWrite(7, 0);
      
      analogWrite(8, g_aru8DutyRatio[2]); // H
      analogWrite(9, 0);
      
      analogWrite(10, 0);
      analogWrite(11, g_aru8DutyRatio[3]);    // H
*/
        
    }

  }
  else
  {
    ;   // no statement.
  }

}
