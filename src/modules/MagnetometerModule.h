#pragma once

#include "MeshModule.h"
#include "concurrency/OSThread.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_AHRS.h>

#if defined(ARDUINO_ARCH_ESP32)
  #include <Preferences.h>
  #define MAGMOD_HAVE_NVS 1
#else
  #define MAGMOD_HAVE_NVS 0
#endif

// ------------ Active bus pins (Heltec V3 defaults) ------------
// NOTE: The platform firmware already starts these two I2C masters at boot.
#ifndef I2C0_SDA_PIN   // Wire (I2C0)
#define I2C0_SDA_PIN 41
#endif
#ifndef I2C0_SCL_PIN
#define I2C0_SCL_PIN 42
#endif

#ifndef I2C1_SDA_PIN   // Wire1 (I2C1)
#define I2C1_SDA_PIN 17
#endif
#ifndef I2C1_SCL_PIN
#define I2C1_SCL_PIN 18
#endif
// --------------------------------------------------------------

class MagnetometerModule
  : public MeshModule,
    public concurrency::OSThread
{
public:
    MagnetometerModule();
    void setup() override;
    int32_t runOnce() override;

    bool wantPacket(const meshtastic_MeshPacket *p) override { return false; };

    // Public API for other modules
    bool  hasHeading();
    float getHeading();

    // Figure-8 calibration (3D hard-iron + per-axis scales)
    void    startFigure8Calibration(uint32_t durationMs = 15000);
    bool    isCalibrating() const { return calibrating; }
    uint8_t getCalibrationPercent() const;

    // Flat-spin calibration (2D soft-iron on XY plane)
    void    startFlatSpinCalibration(uint32_t durationMs = 12000);
    bool    isFlatCalibrating() const { return flatCalibrating; }
    uint8_t getFlatCalPercent() const;

    bool    hasSoftIron2D() const { return siValid; }
    void    clearSoftIron2D();

    // “North here” (user zero) controls
    void    setNorthHere();     // Treat current heading as 0°
    void    clearNorthOffset(); // Clear any user north offset

    // Debug dump of calibration state
    void    dumpCalToLog();

private:
    // ---------- Runtime state ----------
    float headingDegrees = 0.0f;
    bool  headingIsValid = false;
    bool  setupCalled = false;
    bool  sensorsInitialized = false;

    // Presence flags
    bool haveMag = false;
    bool haveAccel = false;

    // Which bus/address the QMC5883L is on
    TwoWire* magBus = nullptr;  // points to Wire or Wire1 once detected
    uint8_t  magAddr = 0x0D;    // 0x0D official, 0x0C clones, 0x1E HMC/LIS3MDL-like

    // Sensor instances
    Adafruit_LIS3DH lis;       // bound to Wire1 (I2C1)
    Adafruit_Madgwick filter;

    // Logging cadence
    uint32_t lastLogMs = 0;

    // ---------- Figure-8 calibration / correction (existing) ----------
    float biasX = 0, biasY = 0, biasZ = 0; // hard-iron (center)
    float scaleX = 1, scaleY = 1, scaleZ = 1; // per-axis equalization
    float userZeroDeg = 0;                    // user “North here” offset (deg)

    bool     calibrating = false;
    uint32_t calStartMs = 0;
    uint32_t calDurationMs = 15000;
    bool     calHasData = false;
    int16_t  minX = 0, maxX = 0, minY = 0, maxY = 0, minZ = 0, maxZ = 0;

    // ---------- Flat-spin (2D soft-iron) ----------
    bool     flatCalibrating = false;
    uint32_t flatStartMs = 0;
    uint32_t flatDurationMs = 12000;
    bool     flatHasData = false;

    // Online moments for XY (on already figure-8-calibrated data)
    uint32_t nXY = 0;
    double   sumX = 0.0, sumY = 0.0;
    double   sXX  = 0.0, sXY  = 0.0, sYY  = 0.0;

    // 2D soft-iron parameters (apply AFTER figure-8 cal)
    bool  siValid = false;
    float siBx = 0.0f, siBy = 0.0f;          // 2D bias (on post-cal XY)
    float siSxx = 1.0f, siSxy = 0.0f,        // 2x2 whitening matrix
          siSyx = 0.0f, siSyy = 1.0f;

    // EMA smoothing of heading (vector form)
    bool  emaHave = false;
    float emaCos = 1.0f, emaSin = 0.0f;
    float emaAlpha = 0.2f; // 0.0..1.0, higher = quicker response

    // ---------- Helpers ----------
    void initSensors();
    void scanI2CBus(TwoWire &bus, const char *tag, int sda, int scl);
    bool probeAddr(TwoWire &bus, uint8_t addr);
    void logPlatformAndPinsOnce();
    void explainWhyHeadingInvalidOnce();
    bool loggedWhyInvalid = false;
    bool loggedPlatformPins = false;

    // ---------- Minimal QMC5883L driver (bus-agnostic) ----------
    bool qmcInit(TwoWire &bus, uint8_t addr);
    bool qmcReadRaw(TwoWire &bus, uint8_t addr, int16_t &x, int16_t &y, int16_t &z);
    bool selectMagOnEitherBus(); // decides magBus+magAddr and calls qmcInit

    // Apply calibration to raw magnetometer reading
    void applyCal(int16_t rx, int16_t ry, int16_t rz, float &fx, float &fy, float &fz);
    void applySoftIron2D(float &fx, float &fy); // in-place on post-cal XY

    // Persistent storage
    void loadPrefs();
    void saveCalPrefs();
    void saveNorthPrefs();
    void saveSoftIronPrefs();

#if MAGMOD_HAVE_NVS
    Preferences prefs;
#endif
};

// Global pointer so other modules (e.g., FriendFinder) can query heading
extern MagnetometerModule *magnetometerModule;
