#ifndef BLE_PREPROCESS_H
#define BLE_PREPROCESS_H

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Custom UUID (Use your own generated UUID)
// you can generate your own via https://www.uuidgenerator.net/
#define SERVICE_UUID        "7009fc28-788a-4bef-a0f5-107beb6ed419"
#define CHARACTERISTIC_UUID "d6ec3c26-8bad-4c7d-9cce-bf0097f277ae"

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

// Server callback to monitor connections
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; }
};


#endif // BLE_PREPROCESS_H