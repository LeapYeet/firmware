#include "MagnetometerModule.h"

MagnetometerModule *magnetometerModule = nullptr;

// Create a dedicated object for the second I2C bus
TwoWire I2C_ACC = TwoWire(1);

MagnetometerModule::MagnetometerModule()
    : MeshModule("Magnetometer"), 
      OSThread("Mag"),
      lis(&I2C_ACC) // Tell the LIS3DH to use our second I2C bus
{
    magnetometerModule = this;
    // NEW LOGGING
    LOG_DEBUG("[Magnetometer] Module created.");
}

void MagnetometerModule::initSensors()
{
    // NEW LOGGING
    LOG_INFO("[Magnetometer] Initializing sensors...");
    
    // Initialize the default 'Wire' bus with the magnetometer's pins
    LOG_DEBUG("[Magnetometer] Initializing MAG I2C on SDA=35, SCL=36");
    Wire.begin(35, 36);

    // Initialize the second bus with the accelerometer's pins
    LOG_DEBUG("[Magnetometer] Initializing ACC I2C on SDA=17, SCL=18");
    I2C_ACC.begin(17, 18);

    // The compass library automatically uses the default 'Wire' bus, which we've just configured
    LOG_DEBUG("[Magnetometer] Initializing QMC5883L compass...");
    compass.init();

    // The 'lis' object was already told to use I2C_ACC in the constructor.
    // Now we just call begin() on it.
    bool accOK = lis.begin(0x18);
    if (!accOK) {
        LOG_ERROR("[Magnetometer] LIS3DH accelerometer not found on port 17/18!");
    } else {
        LOG_INFO("[Magnetometer] Accelerometer found.");
    }

    filter.begin(20);
    headingIsValid = accOK;
}

void MagnetometerModule::setup()
{
    // NEW LOGGING
    LOG_INFO("[Magnetometer] Module setup started.");
    initSensors();
}

int32_t MagnetometerModule::runOnce()
{
    if (headingIsValid) {
        // Read raw sensor data
        compass.read();
        sensors_event_t accel_event;
        lis.getEvent(&accel_event);

        // Get magnetometer values
        float cal_mag_x = compass.getX();
        float cal_mag_y = compass.getY();
        float cal_mag_z = compass.getZ();

        // Update the filter with the latest sensor data
        filter.update(0, 0, 0, // No gyroscope
                      accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                      cal_mag_x, cal_mag_y, cal_mag_z);
        
        // The "Yaw" value from the filter is our compass heading
        headingDegrees = filter.getYaw();

        // NEW LOGGING: Replaced the spammy log with a periodic one
        if (millis() - lastLogTime > 2000) { // Log every 2 seconds
            LOG_DEBUG("[Magnetometer] Accel X:%.2f Y:%.2f Z:%.2f",
                      accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z);
            LOG_DEBUG("[Magnetometer] Mag X:%.2f Y:%.2f Z:%.2f", cal_mag_x, cal_mag_y, cal_mag_z);
            LOG_INFO("[Magnetometer] Calculated Heading: %.2f degrees", headingDegrees);
            lastLogTime = millis();
        }
    }
    
    return 50;
}

bool MagnetometerModule::hasHeading()
{
    return headingIsValid;
}

float MagnetometerModule::getHeading()
{
    // NEW LOGGING (set to DEBUG to avoid spam, but useful for deep debugging)
    LOG_DEBUG("[Magnetometer] Heading of %.2f degrees was requested by another module.", headingDegrees);
    return headingDegrees;
}