#include "MagnetometerModule.h"
#include <math.h>

// Use the already-started system I2C instances
extern TwoWire Wire;   // I2C0  (SDA=41, SCL=42)
extern TwoWire Wire1;  // I2C1  (SDA=17, SCL=18)

MagnetometerModule *magnetometerModule = nullptr;

// Pretty names for diagnostics
static const char* guessDeviceName(uint8_t addr) {
    switch (addr) {
        case 0x0D: return "QMC5883L MAG (official)";
        case 0x0C: return "QMC5883L MAG (alt/clone)";
        case 0x1E: return "HMC5883L/LIS3MDL MAG";
        case 0x18: return "LIS3DH ACC (0x18)";
        case 0x19: return "LIS3DH ACC (0x19)";
        case 0x3C: return "SSD1306 OLED (0x3C)";
        default: return "";
    }
}

static float wrap360(float d) {
    while (d < 0)   d += 360.0f;
    while (d >= 360.0f) d -= 360.0f;
    return d;
}

MagnetometerModule::MagnetometerModule()
    : MeshModule("Magnetometer"),
      OSThread("Mag"),
      lis(&Wire1) // LIS3DH on I2C1 if present
{
    magnetometerModule = this;
    LOG_INFO("[Magnetometer] Module constructed");
}

void MagnetometerModule::logPlatformAndPinsOnce() {
    if (loggedPlatformPins) return;
    loggedPlatformPins = true;

#if defined(ARDUINO_ARCH_ESP32)
    LOG_INFO("[Magnetometer] Build target: ESP32 family (S3 per boot log).");
#else
    LOG_INFO("[Magnetometer] Build target: non-ESP32.");
#endif
    LOG_INFO("[Magnetometer] Active I2C masters (already started by platform):");
    LOG_INFO("[Magnetometer]   Wire  (I2C0): SDA=%d SCL=%d", I2C0_SDA_PIN, I2C0_SCL_PIN);
    LOG_INFO("[Magnetometer]   Wire1 (I2C1): SDA=%d SCL=%d", I2C1_SDA_PIN, I2C1_SCL_PIN);
}

bool MagnetometerModule::probeAddr(TwoWire &bus, uint8_t addr) {
    bus.beginTransmission(addr);
    int err = bus.endTransmission();
    return (err == 0);
}

void MagnetometerModule::scanI2CBus(TwoWire &bus, const char *tag, int sda, int scl) {
    LOG_INFO("[Magnetometer] I2C scan on %s (SDA=%d, SCL=%d) begin.", tag, sda, scl);
    int found = 0;
    for (uint8_t addr = 1; addr < 127; ++addr) {
        bus.beginTransmission(addr);
        uint8_t err = bus.endTransmission();
        if (err == 0) {
            ++found;
            const char* pretty = guessDeviceName(addr);
            if (pretty && pretty[0]) {
                LOG_INFO("[Magnetometer]  - ACK at 0x%02X (%s)", addr, pretty);
            } else {
                LOG_INFO("[Magnetometer]  - ACK at 0x%02X", addr);
            }
        }
    }
    if (!found) {
        LOG_INFO("[Magnetometer]  - No devices ACK on %s.", tag);
    }
    LOG_INFO("[Magnetometer] I2C scan on %s complete. Found=%d", tag, found);
}

// ---------------- Minimal QMC5883L driver ----------------
static constexpr uint8_t QMC_REG_X_L   = 0x00;
static constexpr uint8_t QMC_REG_CTRL1 = 0x09;
static constexpr uint8_t QMC_REG_CTRL2 = 0x0A;
static constexpr uint8_t QMC_REG_SET   = 0x0B;

static bool qmcWriteReg(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t val) {
    bus.beginTransmission(addr);
    bus.write(reg);
    bus.write(val);
    return bus.endTransmission() == 0;
}

static bool qmcReadRegs(TwoWire &bus, uint8_t addr, uint8_t startReg, uint8_t *buf, size_t n) {
    bus.beginTransmission(addr);
    bus.write(startReg);
    if (bus.endTransmission(false) != 0) return false;
    size_t got = bus.requestFrom((int)addr, (int)n);
    if (got != n) return false;
    for (size_t i = 0; i < n; ++i) buf[i] = bus.read();
    return true;
}

bool MagnetometerModule::qmcInit(TwoWire &bus, uint8_t addr) {
    LOG_INFO("[Magnetometer] QMC init on %s @0x%02X",
             (&bus == &Wire) ? "Wire" : "Wire1", addr);

    // Soft reset
    if (!qmcWriteReg(bus, addr, QMC_REG_CTRL2, 0x80)) {
        LOG_INFO("[Magnetometer] QMC write CTRL2 (soft reset) failed");
        return false;
    }
    delay(10);

    // Set/Reset period
    if (!qmcWriteReg(bus, addr, QMC_REG_SET, 0x01)) {
        LOG_INFO("[Magnetometer] QMC write SET/RESET failed");
        return false;
    }

    // CTRL1: OSR=512 (00), RNG=2G (01), ODR=200Hz (11), MODE=continuous (01) -> 0x1D
    if (!qmcWriteReg(bus, addr, QMC_REG_CTRL1, 0x1D)) {
        LOG_INFO("[Magnetometer] QMC write CTRL1 failed");
        return false;
    }

    // CTRL2: pointer rollover enable (0x40)
    if (!qmcWriteReg(bus, addr, QMC_REG_CTRL2, 0x40)) {
        LOG_INFO("[Magnetometer] QMC write CTRL2 failed");
        return false;
    }

    LOG_INFO("[Magnetometer] QMC configured (CONT mode, 200Hz, 2G, OSR512).");
    return true;
}

bool MagnetometerModule::qmcReadRaw(TwoWire &bus, uint8_t addr, int16_t &x, int16_t &y, int16_t &z) {
    uint8_t raw[6];
    if (!qmcReadRegs(bus, addr, QMC_REG_X_L, raw, sizeof(raw))) return false;
    // QMC is little-endian: X_L, X_H, Y_L, Y_H, Z_L, Z_H
    x = (int16_t)((raw[1] << 8) | raw[0]);
    y = (int16_t)((raw[3] << 8) | raw[2]);
    z = (int16_t)((raw[5] << 8) | raw[4]);
    return true;
}

bool MagnetometerModule::selectMagOnEitherBus() {
    struct Probe {
        TwoWire* bus;
        const char* tag;
        uint8_t addr;
    } probes[] = {
        { &Wire,  "Wire",  0x0D }, { &Wire,  "Wire",  0x0C }, { &Wire,  "Wire",  0x1E },
        { &Wire1, "Wire1", 0x0D }, { &Wire1, "Wire1", 0x0C }, { &Wire1, "Wire1", 0x1E },
    };

    for (auto &p : probes) {
        if (probeAddr(*p.bus, p.addr)) {
            LOG_INFO("[Magnetometer] MAG probe ACK @0x%02X on %s", p.addr, p.tag);
            if (qmcInit(*p.bus, p.addr)) {
                magBus  = p.bus;
                magAddr = p.addr;
                return true;
            } else {
                LOG_INFO("[Magnetometer] QMC init failed on %s @0x%02X, trying next...", p.tag, p.addr);
            }
        }
    }
    return false;
}

void MagnetometerModule::applyCal(int16_t rx, int16_t ry, int16_t rz, float &fx, float &fy, float &fz) {
    // Remove hard-iron bias
    float x = (float)rx - biasX;
    float y = (float)ry - biasY;
    float z = (float)rz - biasZ;

    // Equalize axes (soft-iron approx: per-axis scale to average radius)
    x *= scaleX;
    y *= scaleY;
    z *= scaleZ;

    fx = x; fy = y; fz = z;
}

void MagnetometerModule::applySoftIron2D(float &fx, float &fy) {
    if (!siValid) return;
    // Apply 2-D whitening matrix after subtracting 2D bias (on already figure-8-calibrated XY)
    float vx = fx - siBx;
    float vy = fy - siBy;
    float ox = siSxx * vx + siSxy * vy;
    float oy = siSyx * vx + siSyy * vy;
    fx = ox; fy = oy;
}

void MagnetometerModule::initSensors() {
    sensorsInitialized = false;

    LOG_INFO("[Magnetometer] ===== initSensors() START =====");
    logPlatformAndPinsOnce();

    scanI2CBus(Wire,  "Wire  (I2C0 / OLED+MAG?)",  I2C0_SDA_PIN, I2C0_SCL_PIN);
    scanI2CBus(Wire1, "Wire1 (I2C1 / ACC+MAG?)",  I2C1_SDA_PIN, I2C1_SCL_PIN);

    // Detect and bind QMC on either bus
    haveMag = selectMagOnEitherBus();
    if (haveMag) {
        LOG_INFO("[Magnetometer] Using %s for MAG @0x%02X",
                 (magBus == &Wire) ? "Wire" : "Wire1", magAddr);
    } else {
        LOG_INFO("[Magnetometer] No magnetometer detected on Wire or Wire1.");
    }

    // Detect accelerometer (LIS3DH) on Wire1 (0x18/0x19)
    LOG_INFO("[Magnetometer] Trying LIS3DH on Wire1 @0x18...");
    bool accOK = lis.begin(0x18);
    if (!accOK) {
        LOG_INFO("[Magnetometer] Trying LIS3DH on Wire1 @0x19...");
        accOK = lis.begin(0x19);
    }
    haveAccel = accOK;

    if (haveAccel) {
        LOG_INFO("[Magnetometer] LIS3DH detected on Wire1. Start Madgwick @20 Hz.");
        filter.begin(20);
    } else {
        LOG_INFO("[Magnetometer] LIS3DH NOT detected on Wire1; tilt compensation disabled (mag-only fallback OK).");
    }

    loadPrefs(); // load bias/scale, 2D soft-iron, and north offset from NVS if present

    // Heading is valid if MAG exists
    headingIsValid = haveMag;
    LOG_INFO("[Magnetometer] headingIsValid = %s (haveMag=%s, haveAccel=%s)",
             headingIsValid ? "TRUE" : "FALSE",
             haveMag ? "TRUE" : "FALSE",
             haveAccel ? "TRUE" : "FALSE");

    sensorsInitialized = true;
    LOG_INFO("[Magnetometer] ===== initSensors() END =====");
}

void MagnetometerModule::setup() {
    setupCalled = true;
    LOG_INFO("[Magnetometer] setup() called.");
    initSensors();
}

int32_t MagnetometerModule::runOnce() {
    // If the manager didn’t call setup(), self-init on first run.
    static bool warnedOnce = false;
    if (!setupCalled && !warnedOnce) {
        LOG_INFO("[Magnetometer] WARNING: runOnce() executing before setup()! Module manager likely did not call setup().");
        warnedOnce = true;
    }

    if (!sensorsInitialized) {
        LOG_INFO("[Magnetometer] Sensors not initialized yet; will try initSensors() now.");
        initSensors();
    }

    if (!headingIsValid) {
        explainWhyHeadingInvalidOnce();
        return 500; // wait a bit and try again
    }

    // Read MAG (bus-agnostic)
    int16_t rx, ry, rz;
    if (!qmcReadRaw(*magBus, magAddr, rx, ry, rz)) {
        LOG_INFO("[Magnetometer] QMC read failed; will retry.");
        return 100;
    }

    // If collecting figure-8 calibration, update min/max window on RAW
    if (calibrating) {
        if (!calHasData) {
            minX = maxX = rx; minY = maxY = ry; minZ = maxZ = rz;
            calHasData = true;
        } else {
            if (rx < minX) minX = rx; if (rx > maxX) maxX = rx;
            if (ry < minY) minY = ry; if (ry > maxY) maxY = ry;
            if (rz < minZ) minZ = rz; if (rz > maxZ) maxZ = rz;
        }

        const uint32_t now = millis();
        const uint32_t elapsed = now - calStartMs;
        if (elapsed % 1000 < 50) { // roughly 1Hz log
            LOG_INFO("[Magnetometer] Cal sample: X[%d..%d] Y[%d..%d] Z[%d..%d] %u%%",
                     (int)minX, (int)maxX, (int)minY, (int)maxY, (int)minZ, (int)maxZ,
                     (unsigned)getCalibrationPercent());
        }

        if (elapsed >= calDurationMs) {
            calibrating = false;
            if (calHasData) {
                // Calculate bias (center) and radius per axis
                const float cX = 0.5f * ((float)maxX + (float)minX);
                const float cY = 0.5f * ((float)maxY + (float)minY);
                const float cZ = 0.5f * ((float)maxZ + (float)minZ);
                const float rX = 0.5f * ((float)maxX - (float)minX);
                const float rY = 0.5f * ((float)maxY - (float)minY);
                const float rZ = 0.5f * ((float)maxZ - (float)minZ);
                const float rAvg = (rX + rY + rZ) / 3.0f;

                biasX = cX; biasY = cY; biasZ = cZ;
                // Scale so each axis radius matches average (avoid divide-by-zero)
                scaleX = (rX > 1e-3f) ? (rAvg / rX) : 1.0f;
                scaleY = (rY > 1e-3f) ? (rAvg / rY) : 1.0f;
                scaleZ = (rZ > 1e-3f) ? (rAvg / rZ) : 1.0f;

                saveCalPrefs();

                LOG_INFO("[Magnetometer] Calibration DONE. Bias(%.2f, %.2f, %.2f) Scale(%.3f, %.3f, %.3f)",
                         biasX, biasY, biasZ, scaleX, scaleY, scaleZ);
            } else {
                LOG_INFO("[Magnetometer] Calibration finished but no samples collected?");
            }
        }
    }

    // Apply figure-8 calibration to raw mag
    float fx, fy, fz;
    applyCal(rx, ry, rz, fx, fy, fz);

    // If collecting flat-spin (2D) samples, accumulate ON post-figure-8 XY
    if (flatCalibrating) {
        if (!flatHasData) {
            flatHasData = true;
            nXY = 0; sumX = sumY = 0.0; sXX = sXY = sYY = 0.0;
        }
        // Online accumulation (Welford-like)
        nXY++;
        double dx = (double)fx;
        double dy = (double)fy;
        sumX += dx;
        sumY += dy;
        sXX  += dx * dx;
        sXY  += dx * dy;
        sYY  += dy * dy;

        const uint32_t now = millis();
        const uint32_t elapsed = now - flatStartMs;
        if (elapsed % 1000 < 50) {
            LOG_INFO("[Magnetometer] Flat-spin %u%% (n=%lu)", (unsigned)getFlatCalPercent(), (unsigned long)nXY);
        }

        if (elapsed >= flatDurationMs) {
            flatCalibrating = false;

            if (nXY >= 25) {
                // Compute mean and covariance
                const double invN = 1.0 / (double)nXY;
                const double mx = sumX * invN;
                const double my = sumY * invN;
                const double cXX = sXX * invN - mx*mx;
                const double cXY = sXY * invN - mx*my;
                const double cYY = sYY * invN - my*my;

                // Eigen-decomp of 2x2 symmetric covariance
                // Rotate by theta = 0.5 * atan2(2*b, a-c)
                const double a = cXX, b = cXY, c = cYY;
                double theta = 0.5 * atan2(2.0*b, a - c);
                double cs = cos(theta), sn = sin(theta);

                // Eigenvalues (projected)
                // l1 along [cs; sn], l2 along [-sn; cs]
                const double cs2 = cs*cs, sn2 = sn*sn, csn = cs*sn;
                double l1 = a*cs2 + 2.0*b*csn + c*sn2;
                double l2 = a*sn2 - 2.0*b*csn + c*cs2;

                const float kEPS = 1e-6f;
                l1 = fmaxf((float)l1, kEPS);
                l2 = fmaxf((float)l2, kEPS);

                const double invS1 = 1.0 / sqrt(l1);
                const double invS2 = 1.0 / sqrt(l2);

                // S = R * diag(invS1, invS2) * R^T (whitening)
                // R = [cs -sn; sn cs]
                // Compute S entries explicitly
                const double a11 =  cs*invS1*cs + (-sn)*invS2*(-sn); // cs^2 * invS1 + sn^2 * invS2
                const double a12 =  cs*invS1*sn + (-sn)*invS2*cs;    // cs*sn*(invS1 - invS2)
                const double a21 =  sn*invS1*cs +  cs*invS2*(-sn);   // same as a12
                const double a22 =  sn*invS1*sn +  cs*invS2*cs;      // sn^2 * invS1 + cs^2 * invS2

                // Store as floats
                siBx  = (float)mx;
                siBy  = (float)my;
                siSxx = (float)a11;
                siSxy = (float)a12;
                siSyx = (float)a21;
                siSyy = (float)a22;
                siValid = true;

                saveSoftIronPrefs();

                LOG_INFO("[Magnetometer] Flat-spin DONE. n=%lu", (unsigned long)nXY);
                LOG_INFO("[Magnetometer]   mean=(%.2f, %.2f)", siBx, siBy);
                LOG_INFO("[Magnetometer]   S=[[%.5f %.5f][%.5f %.5f]]",
                         siSxx, siSxy, siSyx, siSyy);
            } else {
                LOG_INFO("[Magnetometer] Flat-spin finished but not enough samples (n=%lu).", (unsigned long)nXY);
            }
        }
    }

    // Apply 2D soft-iron (if available)
    applySoftIron2D(fx, fy);

    float headingDeg = 0.0f;

    if (haveAccel) {
        sensors_event_t accel;
        lis.getEvent(&accel);

        filter.update(
            0, 0, 0,
            accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
            fx, fy, fz);

        headingDeg = filter.getYaw();
    } else {
        // Mag-only fallback (atan2 Y/X)
        headingDeg = atan2f(fy, fx) * 180.0f / (float)M_PI;
    }

    // Normalize to [0..360)
    headingDeg = wrap360(headingDeg);

    // Apply user “north here” offset (so that if user pointed to true N and pressed Set North, we output ~0°)
    headingDeg = wrap360(headingDeg - userZeroDeg);

    // EMA smoothing in unit-vector domain to avoid wrap glitches
    {
        const float c = cosf(headingDeg * (float)M_PI / 180.0f);
        const float s = sinf(headingDeg * (float)M_PI / 180.0f);
        if (!emaHave) {
            emaCos = c; emaSin = s; emaHave = true;
        } else {
            emaCos = (1.0f - emaAlpha) * emaCos + emaAlpha * c;
            emaSin = (1.0f - emaAlpha) * emaSin + emaAlpha * s;
            const float n = sqrtf(emaCos*emaCos + emaSin*emaSin);
            if (n > 1e-6f) { emaCos /= n; emaSin /= n; }
        }
        headingDegrees = wrap360(atan2f(emaSin, emaCos) * 180.0f / (float)M_PI);
    }

    const uint32_t now = millis();
    if (now - lastLogMs > 2000) {
        LOG_INFO("[Magnetometer] (mag-only) MAG x=%d y=%d z=%d | fx=%.1f fy=%.1f fz=%.1f | heading=%.2f deg (bus=%s @0x%02X)",
                 rx, ry, rz, fx, fy, fz, headingDegrees,
                 (magBus == &Wire) ? "Wire" : "Wire1", magAddr);
        lastLogMs = now;
    }

    return 50;
}

bool MagnetometerModule::hasHeading() {
    LOG_INFO("[Magnetometer] hasHeading() -> %s", headingIsValid ? "TRUE" : "FALSE");
    return headingIsValid;
}

float MagnetometerModule::getHeading() {
    LOG_INFO("[Magnetometer] getHeading() -> %.2f deg (valid=%s)", headingDegrees, headingIsValid ? "TRUE" : "FALSE");
    return headingDegrees;
}

void MagnetometerModule::explainWhyHeadingInvalidOnce() {
    if (loggedWhyInvalid) return;
    loggedWhyInvalid = true;

    LOG_INFO("[Magnetometer] Heading INVALID — diagnostics:");
    LOG_INFO("[Magnetometer]   setupCalled=%s sensorsInitialized=%s haveMag=%s haveAccel=%s",
             setupCalled ? "TRUE" : "FALSE",
             sensorsInitialized ? "TRUE" : "FALSE",
             haveMag ? "TRUE" : "FALSE",
             haveAccel ? "TRUE" : "FALSE");

    LOG_INFO("[Magnetometer]   Buses in use:");
    LOG_INFO("[Magnetometer]     Wire  (I2C0): SDA=%d SCL=%d", I2C0_SDA_PIN, I2C0_SCL_PIN);
    LOG_INFO("[Magnetometer]     Wire1 (I2C1): SDA=%d SCL=%d", I2C1_SDA_PIN, I2C1_SCL_PIN);

    scanI2CBus(Wire,  "Wire (diag)",  I2C0_SDA_PIN, I2C0_SCL_PIN);
    scanI2CBus(Wire1, "Wire1 (diag)", I2C1_SDA_PIN, I2C1_SCL_PIN);

    LOG_INFO("[Magnetometer]   Expect MAG at 0x0D/0x0C/0x1E. If not present on either bus, check power/SDA/SCL.");
}

/* ---------- Calibration Control ---------- */

// Figure-8
void MagnetometerModule::startFigure8Calibration(uint32_t durationMs) {
    if (!haveMag) {
        LOG_INFO("[Magnetometer] startFigure8Calibration(): no MAG present.");
        return;
    }
    calibrating = true;
    calHasData = false;
    calStartMs = millis();
    calDurationMs = durationMs ? durationMs : 15000;
    LOG_INFO("[Magnetometer] FIGURE-8 calibration started for %u ms. Move device in a wide 8 in all orientations.", (unsigned)calDurationMs);
}

uint8_t MagnetometerModule::getCalibrationPercent() const {
    if (!calibrating) return 100;
    uint32_t elapsed = millis() - calStartMs;
    if (elapsed >= calDurationMs) return 100;
    return (uint8_t) ((elapsed * 100UL) / calDurationMs);
}

// Flat-spin
void MagnetometerModule::startFlatSpinCalibration(uint32_t durationMs) {
    if (!haveMag) {
        LOG_INFO("[Magnetometer] startFlatSpinCalibration(): no MAG present.");
        return;
    }
    flatCalibrating = true;
    flatHasData = false;
    flatStartMs = millis();
    flatDurationMs = durationMs ? durationMs : 12000;
    nXY = 0; sumX = sumY = 0.0; sXX = sXY = sYY = 0.0;
    LOG_INFO("[Magnetometer] FLAT-SPIN calibration started for %u ms. Spin device flat on a table at constant speed.", (unsigned)flatDurationMs);
}

uint8_t MagnetometerModule::getFlatCalPercent() const {
    if (!flatCalibrating) return 100;
    uint32_t elapsed = millis() - flatStartMs;
    if (elapsed >= flatDurationMs) return 100;
    return (uint8_t)((elapsed * 100UL) / flatDurationMs);
}

void MagnetometerModule::setNorthHere() {
    if (!headingIsValid) {
        LOG_INFO("[Magnetometer] setNorthHere(): heading not valid yet.");
        return;
    }
    // Current displayed heading should become 0°
    userZeroDeg = headingDegrees;
    saveNorthPrefs();
    LOG_INFO("[Magnetometer] Set North Here: userZeroDeg=%.2f", userZeroDeg);
}

void MagnetometerModule::clearNorthOffset() {
    userZeroDeg = 0.0f;
    saveNorthPrefs();
    LOG_INFO("[Magnetometer] Cleared North offset.");
}

void MagnetometerModule::clearSoftIron2D() {
    siValid = false;
    siBx = siBy = 0.0f;
    siSxx = siSyy = 1.0f; siSxy = siSyx = 0.0f;
    saveSoftIronPrefs();
    LOG_INFO("[Magnetometer] Cleared 2D soft-iron matrix.");
}

/* ---------- Prefs ---------- */
void MagnetometerModule::loadPrefs() {
#if MAGMOD_HAVE_NVS
    if (!prefs.begin("magmod", /*rw=*/true)) {
        LOG_INFO("[Magnetometer] NVS open failed; using defaults.");
        return;
    }
    biasX = prefs.getFloat("bx", 0.0f);
    biasY = prefs.getFloat("by", 0.0f);
    biasZ = prefs.getFloat("bz", 0.0f);
    scaleX = prefs.getFloat("sx", 1.0f);
    scaleY = prefs.getFloat("sy", 1.0f);
    scaleZ = prefs.getFloat("sz", 1.0f);
    userZeroDeg = prefs.getFloat("north", 0.0f);

    // 2D soft-iron
    siValid = prefs.getBool("si_ok", false);
    siBx    = prefs.getFloat("si_bx", 0.0f);
    siBy    = prefs.getFloat("si_by", 0.0f);
    siSxx   = prefs.getFloat("si_sxx", 1.0f);
    siSxy   = prefs.getFloat("si_sxy", 0.0f);
    siSyx   = prefs.getFloat("si_syx", 0.0f);
    siSyy   = prefs.getFloat("si_syy", 1.0f);

    prefs.end();

    LOG_INFO("[Magnetometer] Loaded cal Bias(%.2f, %.2f, %.2f) Scale(%.3f, %.3f, %.3f) North=%.2f",
             biasX, biasY, biasZ, scaleX, scaleY, scaleZ, userZeroDeg);
    if (siValid) {
        LOG_INFO("[Magnetometer] Loaded 2D soft-iron: bx=%.2f by=%.2f S=[[%.5f %.5f][%.5f %.5f]]",
                 siBx, siBy, siSxx, siSxy, siSyx, siSyy);
    }
#endif
}

void MagnetometerModule::saveCalPrefs() {
#if MAGMOD_HAVE_NVS
    if (!prefs.begin("magmod", /*rw=*/false)) return;
    prefs.putFloat("bx", biasX);
    prefs.putFloat("by", biasY);
    prefs.putFloat("bz", biasZ);
    prefs.putFloat("sx", scaleX);
    prefs.putFloat("sy", scaleY);
    prefs.putFloat("sz", scaleZ);
    prefs.end();
#endif
}

void MagnetometerModule::saveNorthPrefs() {
#if MAGMOD_HAVE_NVS
    if (!prefs.begin("magmod", /*rw=*/false)) return;
    prefs.putFloat("north", userZeroDeg);
    prefs.end();
#endif
}

void MagnetometerModule::saveSoftIronPrefs() {
#if MAGMOD_HAVE_NVS
    if (!prefs.begin("magmod", /*rw=*/false)) return;
    prefs.putBool ("si_ok", siValid);
    prefs.putFloat("si_bx", siBx);
    prefs.putFloat("si_by", siBy);
    prefs.putFloat("si_sxx", siSxx);
    prefs.putFloat("si_sxy", siSxy);
    prefs.putFloat("si_syx", siSyx);
    prefs.putFloat("si_syy", siSyy);
    prefs.end();
#endif
}

void MagnetometerModule::dumpCalToLog() {
    LOG_INFO("[DUMP] cal_ok=1 off=(%.2f,%.2f,%.2f) sc=(%.2f,%.2f,%.2f)",
             biasX, biasY, biasZ, scaleX, scaleY, scaleZ);
    LOG_INFO("[DUMP] align_ok=1 align_deg=%.2f", userZeroDeg);
    LOG_INFO("[DUMP] si_ok=%d bx=%.2f by=%.2f S=[[%.5f %.5f][%.5f %.5f]]",
             siValid ? 1 : 0, siBx, siBy, siSxx, siSxy, siSyx, siSyy);
}
