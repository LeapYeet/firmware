#pragma once

#include "MeshModule.h"
#include "concurrency/OSThread.h"
#include <Wire.h> // Required for TwoWire object
#include <Adafruit_LIS3DH.h>
#include <Adafruit_AHRS.h>
#include <QMC5883LCompass.h>

class MagnetometerModule
  : public MeshModule,
    public concurrency::OSThread
{
public:
    MagnetometerModule();
    void setup() override;
    int32_t runOnce() override;
    
    // Required function for any MeshModule
    bool wantPacket(const meshtastic_MeshPacket *p) override { return false; };
    
    bool hasHeading();
    float getHeading();

private:
    float headingDegrees = 0.0f;
    bool headingIsValid = false;
    uint32_t lastLogTime = 0;
    
    // Sensor and Filter Objects
    QMC5883LCompass compass;
    Adafruit_LIS3DH lis;
    Adafruit_Madgwick filter;

    void initSensors();
};

extern MagnetometerModule *magnetometerModule;