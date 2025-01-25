/*  This is the code for the original Bat Caddy cart (Batman) .  This program will run on an XIAO ESP32-S3 board using an Xtensa microprocessor. 
 the golf cart also has a GY 521 IMU on the MPU 6050 board.  z axis gyro data is read from the GY521 to measure the orientation of the cart.
*/

#include "BLEDevice.h"
#include <HardwareSerial.h>  //support Roboclaw interface#include "RoboClaw.h"        //  Use roboclaw commands
#include <Adafruit_MPU6050.h>  //support IMU
#include <Adafruit_Sensor.h>//support IMU
#include <Wire.h>             //support IMU
#include <RoboClaw.h>         // library for the Roboclaw motor controller.  The Roboclaw motor controller is designed to drive 2 motors 
                              //  with 15 amps continuos each.

Adafruit_MPU6050 mpu;

static BLEUUID    serviceUUID("ebdddb72-32f6-495f-aaf1-358ddba09f46");  //establish service "univeral unique identifier"
static BLEUUID    controlCharUUID("ebdddb73-32f6-495f-aaf1-358ddba09f46");

static boolean doConnect = false; // static means it doesn't change memory location?
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pControlChar;
static BLERemoteCharacteristic* pStopChar;
static BLEAdvertisedDevice* myDevice;

HardwareSerial serial(0);	  // Roboclaw setup
RoboClaw roboclaw(&serial, 10000);

float desired_heading = 0;   // this is the goal heading of the cart relative to initial cart orientation
float current_heading = 0;  //heading is always in radians and relative to the last requested orientation
float heading_error = 0;
float turn_correction = 0;   // this is the output of a PID function used to drive the motors to eliminate heading error
float turn_bias = .018;   //this value does the best job of not accumulating heading error while static (rads/sec)
float previous_heading_error = 0;
float error_sum = 0;        // this is the integration of heading_error used in the PID function
float error_delta = 0;      // This represents the rate of change in heading error used in the PID function
float turn_command_in_radians = 0;

//  These are the tuning constants for PID control of the heading error
float Kp = 100;   // proportional gain
float Ki = 4;     // integral gain
float Kd = 0;     // derivative gain

int previous_forward = 0;  //the most recent past forward speed command from Robin
int previous_left_wheel = 0;
int previous_right_wheel = 0;
int speed_change_limit = 15;
int stop_command_timer = 0;
int time_of_last_correction = 0;

bool stop = false;

#define address 0x80  //0x means hex follows  80 is the default address of Roboclaw in hex

/* this function is executed each time that Batman receives a Bluetooth message from Robin, which occurs at approx 10Hz. */

static void controlCallback(                           //
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    if(length != 8) return;
    char command[length];   //create 8 element char array named command used to parse the incoming message
    for(int i = 0; i < length; i++){
      command[i] = pData[i];
    }
    // Serial.println(command_str.length());
    String command_str(command);  //String is a class; command_str is an instantiation.  command is the char array we are passing in
    // Serial.println(command);
    int forward = command_str.substring(0,4).toInt();  //.toInt is a function that exists within String class.
    int right = command_str.substring(4,8).toInt();
    
    /* This block is meant to make the cart stop trying to turn if the cart has not received any commands from the operator for a while*/

    

    
    
    /* 
    The integer 'right' passed from Robin to Batman is intended to be a command to adjust the orientation the cart will seek (desired_heading).
    the joystick that generates the input in Robin code is read by a 12 bit ADC and subsequently converted to 8 bit, so +/- 128 full scale.  The robin code further
    divides that number by 25 to generate a full scale of 0-5.  This integer is treated as a heading change each time it is passed to Batman.
    Since data is passed to Batman at 10 Hz, one full second of full displacement of the joystick should result in approx 5x10=50 degrees of
    heading change.    
    */
    
    turn_command_in_radians = right/56.0;  //convert the turn command from degrees to radians (approx). right turn is positive; neg = left
    
    desired_heading = desired_heading + turn_command_in_radians;      //update desired heading; positive desired heading is clockwise from current
    heading_error = desired_heading - current_heading;      // define the heading error as difference between current heading and desired heading
    error_delta = previous_heading_error - heading_error;
    error_sum = error_sum + heading_error;
    turn_correction = (Kp * heading_error) + (Ki * error_sum) + (Kd * error_delta);     //scale factor was determined empirically; turn correction is neg if turning clkwise  
    previous_heading_error = heading_error;             //save the heading error to calculate the change in next loop

   if (abs(forward) < 3 && right == 0 )
      {                                                  // if the cart is stopped
        if (stop_command_time == 0) { stop_command_timer = millis();}                    // start timer if the cart has stopped
        if (millis() - stop_command_timer > 4000)       // if no motion is commanded for 3 secs, zero out turns
          {                                 
            desired_heading = 0;     // zero out everything if Robin hasn't requested motion for 4 seconds
            current_heading = 0;
            heading_error = 0;
            previous_heading_error = 0;
            turn_correction   = 0;
            stop_command_timer = 0;           //  reset timer to zero    
            error_sum = 0;                   // reset the integral in the PID control of heading correction
            stop = true;      // this is an important feature. 'stop' prevents the cart from moving until Robin requests motion          
          }
      }
     else {stop = false;}      // make sure that motor commands can be issued if the joystick is used

    int left_wheel =  forward + (int)turn_correction;  // this is where the requests from Robin for forward motion and turning are translated 
    int right_wheel = forward - (int)turn_correction;  // into commands for the wheels

   /*  These 2 lines serve 2 functions: 1) prevent any motion if 'stop' is true; 2) limit how much motor speed can change per time step   */

    left_wheel =  !stop*constrain(left_wheel, previous_left_wheel - speed_change_limit, previouslast_left_wheel + speed_change_limit);  // -128  to +128;  correction is neg for clkwise turn, so 
    right_wheel = !stop*constrain(right_wheel, previous_right_wheel - speed_change_limit, previous_right_wheel + speed_change_limit);

    /*  Roboclaw uses an unsigned 7 bit int for control; 0-63 is reverse; 65-127 forward; 64 is stop  */

    int left_wheel_cmd = left_wheel/2 + 64;   
    int right_wheel_cmd = right_wheel/2 + 64;

    left_wheel_cmd  = constrain( left_wheel_cmd, 0, 125);     //ensure the motor commands do not go out of range
    right_wheel_cmd = constrain(right_wheel_cmd, 0, 125);

    roboclaw.ForwardBackwardM1(address, left_wheel_cmd);  //send the motor commands to the motor controller (Roboclaw)
    roboclaw.ForwardBackwardM2(address, right_wheel_cmd);
    
    previous_left_wheel = left_wheel;      // remember last values to limit the change next time
    previous_right_wheel =  right_wheel;

    Serial.print(left_wheel_cmd);
    Serial.print(", ");
    Serial.println(right_wheel_cmd);
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice); 
    Serial.println(" - Connected to server");
    pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pControlChar = pRemoteService->getCharacteristic(controlCharUUID);
    if (pControlChar == nullptr) {
      Serial.print("Failed to find control characteristic UUID: ");
      Serial.println(controlCharUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found control characteristic");

    if(!pControlChar->canNotify()){
      Serial.println("Not able to register control characteristic notification");
      return false;
    }

    Serial.println(" - Registered notification");
    pControlChar->registerForNotify(controlCallback);

    connected = true;
    return true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    
    Serial.println(advertisedDevice.toString().c_str());
    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

void setup() {
  Serial.begin(115200);
  serial.begin(38400, SERIAL_8N1, -1, -1);  //small s because this is an instance of the class hardwareserial
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  Serial.println("BLE initialized. Starting scan");

  if (!mpu.begin())      //  initialize the IMU board
  {         
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
      }
  }
   
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);

  
} // End of setup.

// This is the Arduino main loop function.
void loop() {
  /*
  this is the design philosophy for how Batman(the cart) will handle steering of the cart.  
  The operator will press the remote control joystick left/right to change the desired heading relative to the current heading.
  Batman code will store the desired heading (in radians) relative to the cart orientation and seek to make the cart heading equal to the desired heading.
  The initial cart heading will be set to zero radians.  
  The cart will also detect any uncommanded drift and seek to maintain straight line travel if no turn command is in force.
  The goal orientation will be desired_heading (in radians, relative to a starting orientation set to zero)
  */

  int loop_delay_time = 10;   //sample the gyro at 100 Hz
  float readings_per_sec = 1000/loop_delay_time;
  int num_of_readings = 200;
  sensors_event_t a, g, temp;

  for(int i = 0; i < num_of_readings; i++)
  {
    mpu.getEvent(&a, &g, &temp);
    current_heading = current_heading + !stop*(g.gyro.z + turn_bias)/readings_per_sec ;   //integrate the z axis gyro to estimate current heading relative to desired heading
    delay(loop_delay_time);                                                  // '!stop' prevents any drift from being accumulated while cart is stopped    
  }
  int16_t Lcurrent, Rcurrent;
  roboclaw.ReadCurrents(address, Lcurrent, Rcurrent);

  Serial.print(" current_heading is ");
  Serial.print(current_heading);
  Serial.println(" rad");
  Serial.println("");
  Serial.print(" desired_heading is ");
  Serial.print(desired_heading);
  Serial.println(" rad");
  Serial.println("");
  Serial.print("motor currents L/R are ");
  Serial.print(Lcurrent);
  Serial.print("   ");
  Serial.println(Rcurrent);

  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("Connected to the BLE Server.");
    } else {
      Serial.println("Failed to connect to server");
    }
    doConnect = false;
  }

  if (!connected) {
    BLEDevice::getScan()->start(0);
  }
  
  //delay(1000); // Delay a second between loops.     I don't need a delay here now since I am taking a second to read turnrate from MPU6050
} // End of loop