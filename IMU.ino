void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial2.println("------------------------------------");
    Serial2.print("Sensor:       "); Serial2.println(sensor.name);
    Serial2.print("Driver Ver:   "); Serial2.println(sensor.version);
    Serial2.print("Unique ID:    "); Serial2.println(sensor.sensor_id);
    Serial2.print("Max Value:    "); Serial2.print(sensor.max_value); Serial2.println(" xxx");
    Serial2.print("Min Value:    "); Serial2.print(sensor.min_value); Serial2.println(" xxx");
    Serial2.print("Resolution:   "); Serial2.print(sensor.resolution); Serial2.println(" xxx");
    Serial2.println("------------------------------------");
    Serial2.println("");
    delay(500);
}

void displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial2 Monitor */
    Serial2.println("");
    Serial2.print("System Status: 0x");
    Serial2.println(system_status, HEX);
    Serial2.print("Self Test:     0x");
    Serial2.println(self_test_results, HEX);
    Serial2.print("System Error:  0x");
    Serial2.println(system_error, HEX);
    Serial2.println("");
    delay(500);
}

void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    /* The data should be ignored until the system calibration is > 0 */
    Serial2.print("\t");
    if (!system)
    {
        Serial2.print("! ");
    }
    /* Display the individual values */
    Serial2.print("Sys:");
    Serial2.print(system, DEC);
    Serial2.print(" G:");
    Serial2.print(gyro, DEC);
    Serial2.print(" A:");
    Serial2.print(accel, DEC);
    Serial2.print(" M:");
    Serial2.print(mag, DEC);
    magCalib = mag;
}

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial2.print("Accelerometer: ");
    Serial2.print(calibData.accel_offset_x); Serial2.print(" ");
    Serial2.print(calibData.accel_offset_y); Serial2.print(" ");
    Serial2.print(calibData.accel_offset_z); Serial2.print(" ");

    Serial2.print(" Gyro: ");
    Serial2.print(calibData.gyro_offset_x); Serial2.print(" ");
    Serial2.print(calibData.gyro_offset_y); Serial2.print(" ");
    Serial2.print(calibData.gyro_offset_z); Serial2.print(" ");

    Serial2.print(" Mag: ");
    Serial2.print(calibData.mag_offset_x); Serial2.print(" ");
    Serial2.print(calibData.mag_offset_y); Serial2.print(" ");
    Serial2.print(calibData.mag_offset_z); Serial2.print(" ");

    Serial2.print(" Accel Radius: ");
    Serial2.print(calibData.accel_radius);

    Serial2.print(" Mag Radius: ");
    Serial2.print(calibData.mag_radius);
}
