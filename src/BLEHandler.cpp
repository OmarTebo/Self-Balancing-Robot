#include "BLEHandler.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// UUIDs (random pick)
#define SERVICE_UUID        "d1c6f3e0-9d3b-11ee-be56-0242ac120002"
#define CHAR_KP_UUID        "d1c6f3e1-9d3b-11ee-be56-0242ac120002"
#define CHAR_KI_UUID        "d1c6f3e2-9d3b-11ee-be56-0242ac120002"
#define CHAR_KD_UUID        "d1c6f3e3-9d3b-11ee-be56-0242ac120002"

static BLECharacteristic *kpChar;
static BLECharacteristic *kiChar;
static BLECharacteristic *kdChar;

BLEHandler::BLEHandler() {
  _hasPending = false;
  mux = portMUX_INITIALIZER_UNLOCKED;
}

void BLEHandler::begin() {
  setupBleServer();
}

void BLEHandler::setupBleServer() {
  BLEDevice::init("SBR-Bot");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  kpChar = pService->createCharacteristic(CHAR_KP_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);
  kiChar = pService->createCharacteristic(CHAR_KI_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);
  kdChar = pService->createCharacteristic(CHAR_KD_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);

  kpChar->setCallbacks(new BLECharacteristicCallbacks());
  kiChar->setCallbacks(new BLECharacteristicCallbacks());
  kdChar->setCallbacks(new BLECharacteristicCallbacks());

  // onWrite callback lambda
  struct WriteCB : public BLECharacteristicCallbacks {
    BLEHandler *owner;
    WriteCB(BLEHandler *o): owner(o) {}
    void onWrite(BLECharacteristic *chr) {
      portENTER_CRITICAL(&owner->mux);
      if (chr == kpChar) {
        std::string v = chr->getValue();
        owner->_pending.kp = atof(v.c_str());
      } else if (chr == kiChar) {
        std::string v = chr->getValue();
        owner->_pending.ki = atof(v.c_str());
      } else if (chr == kdChar) {
        std::string v = chr->getValue();
        owner->_pending.kd = atof(v.c_str());
      }
      owner->_hasPending = true;
      portEXIT_CRITICAL(&owner->mux);
    }
  };

  // set callbacks properly (recreate with owner pointer)
  kpChar->setCallbacks(new WriteCB(this));
  kiChar->setCallbacks(new WriteCB(this));
  kdChar->setCallbacks(new WriteCB(this));

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
}

bool BLEHandler::takePending(PIDParams &out) {
  bool rv = false;
  portENTER_CRITICAL(&mux);
  if (_hasPending) {
    out.kp = _pending.kp;
    out.ki = _pending.ki;
    out.kd = _pending.kd;
    _hasPending = false;
    rv = true;
  }
  portEXIT_CRITICAL(&mux);
  return rv;
}
