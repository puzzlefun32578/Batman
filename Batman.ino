// Client Code
#include "BLEDevice.h"
#include <HardwareSerial.h>  //support Roboclaw interface
#include "RoboClaw.h"        //  Use roboclaw commands

static BLEUUID    serviceUUID("ebdddb72-32f6-495f-aaf1-358ddba09f46");  //establish service "univeral unique identifier"
static BLEUUID    controlCharUUID("ebdddb73-32f6-495f-aaf1-358ddba09f46");

static boolean doConnect = false; // static means it doesn't change memory location?
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pControlChar;
static BLERemoteCharacteristic* pStopChar;
static BLEAdvertisedDevice* myDevice;

HardwareSerial serial(0);	  //HardwareSerial is a data class?  serial is an instantiation of the hardware  class. 
RoboClaw roboclaw(&serial,10000);

#define address 0x80  //0x means hex follows  80 is the default address of Roboclaw in hex

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

    int left_wheel = forward + right;  // -128  to +128
    int right_wheel = forward - right;

    int left_wheel_cmd = left_wheel/2 + 64;   //roboclaw uses and unsigned 7 bit int for control; 0-63 is reverse; 65-127 forward; 64 is stop
    int right_wheel_cmd = right_wheel/2 + 64;

    left_wheel_cmd = constrain(left_wheel_cmd, 0, 127);
    right_wheel_cmd = constrain(right_wheel_cmd, 0, 127);

    Serial.print(left_wheel_cmd);
    Serial.print(", ");
    Serial.println(right_wheel_cmd);


    roboclaw.ForwardBackwardM1(address,left_wheel_cmd);
    roboclaw.ForwardBackwardM2(address,right_wheel_cmd);
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
  Serial.begin(9600);
  serial.begin(38400, SERIAL_8N1, -1, -1);  //small s because this is an instance of the class hardwareserial
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  Serial.println("BLE initialized. Starting scan");

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
  
  delay(1000); // Delay a second between loops.
} // End of loop