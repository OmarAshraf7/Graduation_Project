#include "BLEDevice.h"
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <CAN.h>
#include "esp32-hal-cpu.h"

//=======================================================================================================//
//                                           Definitions                                                 //
//=======================================================================================================//

#define TX_GPIO_NUM   4  // Connects to CTX
#define RX_GPIO_NUM   5  // Connects to CRX
#define CAN_ID        0x14 

//=======================================================================================================//
//                                           BLE UUIDs                                                   //
//=======================================================================================================//

// The remote service we wish to connect to.
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

//=======================================================================================================//
//                                          Global Variables                                             //
//=======================================================================================================//

static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;
BLEClient* pClient = BLEDevice::createClient();

int scanTime = 0;        //in seconds
int rssi = 0;            //variable to store RSSI value (in dBm)
const float A = -67;     // Measured RSSI at 1 meter from BLE device
const int n = 2.8 ;      // Path-loss exponent (depends on the environment)
double distance_m = 0;   //distance in meters
int distance_cm = 0;     //distance in centimeters
int samples_arr[1000] ;  //1000 elements array to compute average value of RSSI 
int samples_counter = 0; 
int average_value;

// Extracting digits
int thousands = 0, tens = 0, hundreds = 0, ones = 0 ;

// Characters to send on CAN Bus
char d_thousands = 0 , d_hundreds = 0 , d_tens = 0 , d_ones = 0 ;

//=======================================================================================================//
//                                    APP Functions Prototypes                                           //
//=======================================================================================================//

void   canSender(void);
double computeAverage(int arr[], int size);
void   calculateDistance(int rssi);
void   digits_extractor(int number);

//=======================================================================================================//
//                                     Functions Definitions                                             //
//=======================================================================================================//

void canSender(void) 
{
  digits_extractor(distance_cm);

  Serial.print ("Sending packet ... ");

  CAN.beginPacket (CAN_ID);  //sets the ID and clears the transmit buffer

  //write data to buffer. data is not sent until endPacket() is called.
  CAN.write (d_thousands);
  CAN.write (d_hundreds); 
  CAN.write (d_tens);
  CAN.write (d_ones);

  CAN.endPacket();
  Serial.println ("done");
}
//=======================================================================================================//
void digits_extractor(int number)
{
    thousands = (number / 1000) % 10;
    hundreds = (number / 100) % 10;
    tens = (number / 10) % 10;
    ones = number % 10;

    //Converting digits into Characters
    d_thousands = thousands + '0';
    d_hundreds = hundreds + '0';
    d_tens = tens + '0';
    d_ones = ones + '0';
}
//=======================================================================================================//
void calculateDistance(int rssi)
{
  // Calculate distance using the empirical formula 
  distance_m = pow(10.0, ((A - rssi) / (10.0 * n)) ); // In meters
  distance_cm = distance_m * 100; //Convert it to centimeters 

  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");
}
//=======================================================================================================//
double computeAverage(int arr[], int size)
{
    int sum = 0;
    for(int i = 0; i < size; i++)
    {
        sum += arr[i];
    }
    return (double)sum / size;
}

//=======================================================================================================//
//                                      BLE Notification Callback                                        //
//=======================================================================================================//

static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
 {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    Serial.write(pData, length);
    Serial.println();
}

//=======================================================================================================//
//                                        BLE Client Callbacks                                           //
//=======================================================================================================//

class MyClientCallback : public BLEClientCallbacks 
{
  void onConnect(BLEClient* pClient) {}

  void onDisconnect(BLEClient* pClient)
  {
    connected = false;
    Serial.println("onDisconnect");
  }
};

//=======================================================================================================//
//                                       BLE Server Connection                                           //
//=======================================================================================================//

bool connectToServer()
{
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");

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
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    if(pRemoteCharacteristic->canNotify()){
      pRemoteCharacteristic->registerForNotify(notifyCallback);
    connected = true;
    return true;
    }
}

//=======================================================================================================//
//                                 BLE Advertised Device Callbacks                                       //
//=======================================================================================================//

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

//=======================================================================================================//
//                                          Setup Function                                               //
//=======================================================================================================//

void setup() 
{
  Serial.begin(115200);

  //BLE Intitialization
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning.
  // Not entirely sure what the setInterval or setWindow options are, but they work as is.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  pBLEScan->setActiveScan(true);  

  //CAN Initialization
  while (!Serial);

  Serial.println ("CAN Receiver/Receiver");

  // Set the pins
  CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);

  // start the CAN bus at 500 kbps
  if (!CAN.begin (1000E3))
  {
    Serial.println ("Starting CAN failed!");
    while (1);
  }
  else 
  {
    Serial.println ("CAN Initialized");
  }
}

//=======================================================================================================//
//                                           Loop Function                                               //
//=======================================================================================================//

void loop() 
{
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true)
  {
    if (connectToServer())
    {
      Serial.println("We are now connected to the BLE Server.");
    }
    else
    {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  if (connected == false)
  {
    BLEDevice::getScan()->start(scanTime,false);
  }

  if (connected)
  {
    int _counter = 0;
    rssi = pClient->getRssi();
    samples_arr[samples_counter] = rssi;
    samples_counter++;
    if(samples_counter==999)
    {     
      samples_counter = 0 ;
      for (int i = 0; i <samples_counter;i++){
        if (samples_arr[i] >= -70)
        {
          _counter++;
        }
         if (_counter == 100)
        {
          Serial.print("done");
          break;
        }
      }  
    
      average_value = computeAverage(samples_arr , 1000);
      Serial.println(average_value);      
      calculateDistance(average_value);
    }

  }
  canSender();
//    delay(5); 

}
