/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updated by chegewara
   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 4fafc201-1fb5-459e-8fcc-c5c9c331914b
   And has a characteristic of: beb5483e-36e1-4688-b7f5-ea07361b26a8
   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.
   A connect hander associated with the server starts a background task that performs notification
   every couple of seconds.
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer* pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;
uint8_t rxBleValue = 0;
uint8_t rxBuffer[10];

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "9f73c1ba-7483-11ea-bc55-0242ac130003" // UART service UUID
#define CHARACTERISTIC_UUID_RX "9f73c408-7483-11ea-bc55-0242ac130003"
#define CHARACTERISTIC_UUID_TX "9f73c534-7483-11ea-bc55-0242ac130003"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
          rxBuffer[i] = rxValue[i];
          rxBleValue = rxValue[i];
        }
        
        Serial.println();
        Serial.println("*********");
      }
    }
};

void BLEInit() {
  if (!Serial) {
    Serial.begin(115200);
  }
  
  // Create the BLE Device
  BLEDevice::init("OptiQit");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
  // Create a BLE Descriptor
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}
