#include "sensor_read.h"

void process_imu_packets(sensor_dev_t *sensor_dev, uint16_t *packets, uint8_t *raw_data, float *raw_m, int *g_count, int *a_count, float *a_sum, int *processed_packets, bool *mag_read)
{

    uint16_t total_packets = *packets;
    uint16_t max_possible_packets = 1024 / 12;
    if (total_packets > max_possible_packets)
    {
        return;
    }
    for (uint16_t j = 0; j < total_packets; j++)
    {
        float raw_a[3] = {0};
        float raw_g[3] = {0};
        sensor_data_attrs_t attrs = sensor_dev->sensor_imu->data_process(j, raw_data, raw_a, raw_g, raw_m);
        if (attrs & DATA_INVALID)
            continue; // skip on error

        // TODO: split into separate functions

        if (attrs & DATA_VALID_GYRO)
        {
#if DEBUG
            if (valid_acquisition)
                total_gyro_samples++;
#endif
            sensor_calibration_process_gyro(raw_g);
            float gx = raw_g[0];
            float gy = raw_g[1];
            float gz = raw_g[2];
            float g[] = {gx, gy, gz};

            if (attrs & DATA_VALID_ACCEL)
            {

                sensor_calibration_process_accel(raw_a);
                float ax = raw_a[0];
                float ay = raw_a[1];
                float az = raw_a[2];
                float a[] = {ax, ay, az};

                // Process fusion
                sensor_dev->sensor_fusion->update_accel(a, sensor_dev->accel_actual_time);

                for (int k = 0; k < 3; k++)
                    a_sum[k] += a[k];
                a_count++;
            }
        }
        if (attrs & DATA_VALID_MAG)
        {
            mag_read = true;
        }

        processed_packets++;
    }
}

void process_imu_orientation()
{
}

void process_magnetometer_calibration()
{
}
