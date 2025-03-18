/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "unity.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "bmi270.h"
#include "common/common.h"

#define TEST_MEMORY_LEAK_THRESHOLD (-400)

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH       (9.80665f)

/*! Macros to select the sensors                   */
#define ACCEL               UINT8_C(0x00)
#define GYRO                UINT8_C(0x01)

// #define I2C_MASTER_SCL_IO   CONFIG_I2C_MASTER_SCL   /*!< gpio number for I2C master clock */
// #define I2C_MASTER_SDA_IO   CONFIG_I2C_MASTER_SDA   /*!< gpio number for I2C master data  */

#define UE_SW_I2C           1

#define HW_ESP_SPOT_C5      1
#define HW_ESP_SPOT_S3      0
#define HW_ESP_ASTOM_S3     0

#if HW_ESP_SPOT_C5
#define I2C_INT_IO              3
#define I2C_MASTER_SCL_IO       26
#define I2C_MASTER_SDA_IO       25
#elif HW_ESP_SPOT_S3
#define I2C_INT_IO              5
#define I2C_MASTER_SCL_IO       1
#define I2C_MASTER_SDA_IO       2
#elif HW_ESP_ASTOM_S3
#define I2C_INT_IO              16
#define I2C_MASTER_SCL_IO       0
#define I2C_MASTER_SDA_IO       45
#endif

#define I2C_MASTER_NUM          I2C_NUM_0               /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ      100*1000                /*!< I2C master clock frequency */

static bmi270_handle_t bmi_handle = NULL;
static i2c_bus_handle_t i2c_bus;

bool interrupt_status = false;

static void IRAM_ATTR gpio_isr_edge_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    interrupt_status = true;
    esp_rom_printf("GPIO[%"PRIu32"] intr, val: %d\n", gpio_num, gpio_get_level(gpio_num));
}

/**
 * @brief i2c master initialization
 */
static void i2c_sensor_bmi270_init(void)
{
    const i2c_config_t i2c_bus_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
#if UE_SW_I2C
    i2c_bus = i2c_bus_create(I2C_NUM_SW_1, &i2c_bus_conf);
#else
    i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &i2c_bus_conf);
#endif
    TEST_ASSERT_NOT_NULL_MESSAGE(i2c_bus, "i2c_bus create returned NULL");

    bmi270_i2c_config_t i2c_bmi270_conf = {
        .i2c_handle = i2c_bus,
        .i2c_addr = BMI270_I2C_ADDRESS,
    };
    bmi270_sensor_create(&i2c_bmi270_conf, &bmi_handle);
    TEST_ASSERT_NOT_NULL_MESSAGE(bmi_handle, "BMI270 create returned NULL");
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}

/*!
 * @brief This internal API sets the sensor configuration
 */
static int8_t set_wrist_gesture_config(struct bmi2_dev *bmi2_dev)
{
    /* Variable to define result */
    int8_t rslt;

    /* List the sensors which are required to enable */
    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_WRIST_GESTURE };

    /* Structure to define the type of the sensor and its configurations */
    struct bmi2_sens_config config;

    /* Configure type of feature */
    config.type = BMI2_WRIST_GESTURE;

    /* Enable the selected sensors */
    rslt = bmi270_sensor_enable(sens_list, 2, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK) {
        /* Get default configurations for the type of feature selected */
        rslt = bmi270_get_sensor_config(&config, 1, bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        if (rslt == BMI2_OK) {
            config.cfg.wrist_gest.wearable_arm = BMI2_ARM_LEFT;

            /* Set the new configuration along with interrupt mapping */
            rslt = bmi270_set_sensor_config(&config, 1, bmi2_dev);
            bmi2_error_codes_print_result(rslt);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for accel and gyro.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *bmi)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer and gyro configuration. */
    struct bmi2_sens_config config[2];

    /* Configure the type of feature. */
    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(config, 2, bmi);
    bmi2_error_codes_print_result(rslt);

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK) {
        /* NOTE: The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_200HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_2G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples.
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;

        /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config[GYRO].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set the accel and gyro configurations. */
        rslt = bmi2_set_sensor_config(config, 2, bmi);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}

static int8_t bmi270_enable_wrist_gesture(struct bmi2_dev *bmi2_dev)
{
    /* Variable to define result */
    int8_t rslt;

    /* Initialize status of wrist gesture interrupt */
    uint16_t int_status = 0;

    /* Select features and their pins to be mapped to */
    struct bmi2_sens_int_config sens_int = { .type = BMI2_WRIST_GESTURE, .hw_int_pin = BMI2_INT1 };

    /* Sensor data structure */
    struct bmi2_feat_sensor_data sens_data = { .type = BMI2_WRIST_GESTURE };

    /* The gesture movements are listed in array */
    const char *gesture_output[6] =
    { "unknown_gesture", "push_arm_down", "pivot_up", "wrist_shake_jiggle", "flick_in", "flick_out" };

    /* Set the sensor configuration */
    rslt = set_wrist_gesture_config(bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK) {
        /* Map the feature interrupt */
        rslt = bmi270_map_feat_int(&sens_int, 1, bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        if (rslt == BMI2_OK) {
            printf("Flip the board in portrait/landscape mode:\n");

            /* Loop to print the wrist gesture data when interrupt occurs */
            for (;;) {
                /* Get the interrupt status of the wrist gesture */
                rslt = bmi2_get_int_status(&int_status, bmi2_dev);
                bmi2_error_codes_print_result(rslt);

                if ((rslt == BMI2_OK) && (int_status & BMI270_WRIST_GEST_STATUS_MASK)) {
                    printf("Wrist gesture detected\n");

                    /* Get wrist gesture output */
                    rslt = bmi270_get_feature_data(&sens_data, 1, bmi2_dev);
                    bmi2_error_codes_print_result(rslt);

                    printf("Wrist gesture = %d\r\n", sens_data.sens_data.wrist_gesture_output);

                    printf("Gesture output = %s\n", gesture_output[sens_data.sens_data.wrist_gesture_output]);
                    break;
                }
            }
        }
    }
    return rslt;
}

int8_t bmi270_enable_accel_gyro(struct bmi2_dev *bmi2_dev)
{
    int8_t rslt;

    /* Assign accel and gyro sensor to variable. */
    uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };

    /* Structure to define type of sensor and their respective data. */
    struct bmi2_sens_data sensor_data;

    uint8_t indx = 1;

    float acc_x = 0, acc_y = 0, acc_z = 0;
    float gyr_x = 0, gyr_y = 0, gyr_z = 0;
    struct bmi2_sens_config config;
    /* Accel and gyro configuration settings. */
    rslt = set_accel_gyro_config(bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK) {
        /* NOTE:
         * Accel and Gyro enable must be done after setting configurations
         */
        rslt = bmi2_sensor_enable(sensor_list, 2, bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        if (rslt == BMI2_OK) {
            config.type = BMI2_ACCEL;

            /* Get the accel configurations. */
            rslt = bmi2_get_sensor_config(&config, 1, bmi2_dev);
            bmi2_error_codes_print_result(rslt);

            printf(
                "\nData set, Accel Range, Acc_Raw_X, Acc_Raw_Y, Acc_Raw_Z, Acc_ms2_X, Acc_ms2_Y, Acc_ms2_Z, Gyr_Raw_X, Gyr_Raw_Y, Gyr_Raw_Z, Gyro_DPS_X, Gyro_DPS_Y, Gyro_DPS_Z\n\n");

            while (indx <= 10) {
                rslt = bmi2_get_sensor_data(&sensor_data, bmi2_dev);
                bmi2_error_codes_print_result(rslt);

                if ((rslt == BMI2_OK) && (sensor_data.status & BMI2_DRDY_ACC) &&
                        (sensor_data.status & BMI2_DRDY_GYR)) {
                    /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
                    acc_x = lsb_to_mps2(sensor_data.acc.x, (float)2, bmi2_dev->resolution);
                    acc_y = lsb_to_mps2(sensor_data.acc.y, (float)2, bmi2_dev->resolution);
                    acc_z = lsb_to_mps2(sensor_data.acc.z, (float)2, bmi2_dev->resolution);

                    /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
                    gyr_x = lsb_to_dps(sensor_data.gyr.x, (float)2000, bmi2_dev->resolution);
                    gyr_y = lsb_to_dps(sensor_data.gyr.y, (float)2000, bmi2_dev->resolution);
                    gyr_z = lsb_to_dps(sensor_data.gyr.z, (float)2000, bmi2_dev->resolution);

                    printf("%d, acc:(%6d, %6d, %6d) (%8.2f, %8.2f, %8.2f) gyr:(%6d, %6d, %6d) (%8.2f, %8.2f, %8.2f)\n",
                           config.cfg.acc.range,
                           sensor_data.acc.x,
                           sensor_data.acc.y,
                           sensor_data.acc.z,
                           acc_x,
                           acc_y,
                           acc_z,
                           sensor_data.gyr.x,
                           sensor_data.gyr.y,
                           sensor_data.gyr.z,
                           gyr_x,
                           gyr_y,
                           gyr_z);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    indx++;
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for any-motion.
 */
static int8_t set_feature_config(struct bmi2_dev *bmi2_dev)
{

    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi2_sens_config config;

    /* Interrupt pin configuration */
    struct bmi2_int_pin_config pin_config = { 0 };

    /* Configure the type of feature. */
    config.type = BMI2_ANY_MOTION;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi270_get_sensor_config(&config, 1, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    rslt = bmi2_get_int_pin_config(&pin_config, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK) {
        /* NOTE: The user can change the following configuration parameters according to their requirement. */
        /* 1LSB equals 20ms. Default is 100ms, setting to 80ms. */
        config.cfg.any_motion.duration = 0x04;

        /* 1LSB equals to 0.48mg. Default is 83mg, setting to 50mg. */
        config.cfg.any_motion.threshold = 0x68;

        /* Set new configurations. */
        rslt = bmi270_set_sensor_config(&config, 1, bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        /* Interrupt pin configuration */
        pin_config.pin_type = BMI2_INT1;
        pin_config.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;
        pin_config.pin_cfg[0].lvl = BMI2_INT_ACTIVE_LOW;
        pin_config.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
        pin_config.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
        pin_config.int_latch = BMI2_INT_NON_LATCH;

        rslt = bmi2_set_int_pin_config(&pin_config, bmi2_dev);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}

int8_t bmi270_enable_any_motion_int(struct bmi2_dev *bmi2_dev)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Accel sensor and no-motion feature are listed in array. */
    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_ANY_MOTION };

    /* Variable to get no-motion interrupt status. */
    uint16_t int_status = 0;

    /* Select features and their pins to be mapped to. */
    struct bmi2_sens_int_config sens_int = { .type = BMI2_ANY_MOTION, .hw_int_pin = BMI2_INT1 };

    /* Enable the selected sensors. */
    rslt = bmi270_sensor_enable(sens_list, 2, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK) {
        /* Set feature configurations for no-motion. */
        rslt = set_feature_config(bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        if (rslt == BMI2_OK) {
            /* Map the feature interrupt for no-motion. */
            rslt = bmi270_map_feat_int(&sens_int, 1, bmi2_dev);
            bmi2_error_codes_print_result(rslt);
            printf("Move the board\n");

            /* Loop to get no-motion interrupt. */
            do {
                if (interrupt_status == 1) {
                    interrupt_status = 0;
                    /* Clear buffer. */
                    int_status = 0;

                    /* To get the interrupt status of any-motion. */
                    rslt = bmi2_get_int_status(&int_status, bmi2_dev);
                    bmi2_error_codes_print_result(rslt);

                    /* To check the interrupt status of any-motion. */
                    if (int_status & BMI270_ANY_MOT_STATUS_MASK) {
                        printf("Any-motion interrupt is generated\n");
                        break;
                    }
                } else {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            } while (rslt == BMI2_OK);
        }
    }

    return rslt;
}

TEST_CASE("sensor Bmi270 test", "[Bmi270][sensor][wrist_gesture]")
{
    esp_err_t ret = ESP_OK;

    i2c_sensor_bmi270_init();

    int8_t rslt = bmi270_enable_wrist_gesture(bmi_handle);
    bmi2_error_codes_print_result(rslt);
    TEST_ASSERT_EQUAL(BMI2_OK, rslt);

    bmi270_sensor_del(bmi_handle);
    ret = i2c_bus_delete(&i2c_bus);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

TEST_CASE("sensor Bmi270 test", "[Bmi270][sensor][accel_gyro]")
{
    esp_err_t ret = ESP_OK;

    i2c_sensor_bmi270_init();

    int8_t rslt = bmi270_enable_accel_gyro(bmi_handle);
    bmi2_error_codes_print_result(rslt);
    TEST_ASSERT_EQUAL(BMI2_OK, rslt);

    bmi270_sensor_del(bmi_handle);
    ret = i2c_bus_delete(&i2c_bus);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

TEST_CASE("sensor Bmi270 test", "[Bmi270][sensor][BMI2_ANY_MOTION][BMI2_INT1]")
{
    esp_err_t ret = ESP_OK;

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = (1ULL << I2C_INT_IO);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(I2C_INT_IO, gpio_isr_edge_handler, (void*) I2C_INT_IO);

    i2c_sensor_bmi270_init();

    int8_t rslt = bmi270_enable_any_motion_int(bmi_handle);
    bmi2_error_codes_print_result(rslt);
    TEST_ASSERT_EQUAL(BMI2_OK, rslt);

    gpio_isr_handler_remove(I2C_INT_IO);
    gpio_uninstall_isr_service();

    bmi270_sensor_del(bmi_handle);
    ret = i2c_bus_delete(&i2c_bus);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

static size_t before_free_8bit;
static size_t before_free_32bit;

static void check_leak(size_t before_free, size_t after_free, const char *type)
{
    ssize_t delta = after_free - before_free;
    printf("MALLOC_CAP_%s: Before %u bytes free, After %u bytes free (delta %d)\n", type, before_free, after_free, delta);
    TEST_ASSERT_MESSAGE(delta >= TEST_MEMORY_LEAK_THRESHOLD, "memory leak");
}

void setUp(void)
{
    before_free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    before_free_32bit = heap_caps_get_free_size(MALLOC_CAP_32BIT);
}

void tearDown(void)
{
    size_t after_free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t after_free_32bit = heap_caps_get_free_size(MALLOC_CAP_32BIT);
    check_leak(before_free_8bit, after_free_8bit, "8BIT");
    check_leak(before_free_32bit, after_free_32bit, "32BIT");
}

void app_main(void)
{
    printf("BMI270 TEST \n");
    unity_run_menu();
}
