/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "AP_InertialSensor_MPU6000.h"

extern const AP_HAL::HAL& hal;

// MPU6000 accelerometer scaling
#define MPU6000_ACCEL_SCALE_1G    (GRAVITY_MSS / 4096.0f)

// MPU 6000 registers
#define MPUREG_XG_OFFS_TC                               0x00
#define MPUREG_YG_OFFS_TC                               0x01
#define MPUREG_ZG_OFFS_TC                               0x02
#define MPUREG_X_FINE_GAIN                              0x03
#define MPUREG_Y_FINE_GAIN                              0x04
#define MPUREG_Z_FINE_GAIN                              0x05
#define MPUREG_XA_OFFS_H                                0x06    // X axis accelerometer offset (high byte)
#define MPUREG_XA_OFFS_L                                0x07    // X axis accelerometer offset (low byte)
#define MPUREG_YA_OFFS_H                                0x08    // Y axis accelerometer offset (high byte)
#define MPUREG_YA_OFFS_L                                0x09    // Y axis accelerometer offset (low byte)
#define MPUREG_ZA_OFFS_H                                0x0A    // Z axis accelerometer offset (high byte)
#define MPUREG_ZA_OFFS_L                                0x0B    // Z axis accelerometer offset (low byte)
#define MPUREG_PRODUCT_ID                               0x0C    // Product ID Register
#define MPUREG_XG_OFFS_USRH                     0x13    // X axis gyro offset (high byte)
#define MPUREG_XG_OFFS_USRL                     0x14    // X axis gyro offset (low byte)
#define MPUREG_YG_OFFS_USRH                     0x15    // Y axis gyro offset (high byte)
#define MPUREG_YG_OFFS_USRL                     0x16    // Y axis gyro offset (low byte)
#define MPUREG_ZG_OFFS_USRH                     0x17    // Z axis gyro offset (high byte)
#define MPUREG_ZG_OFFS_USRL                     0x18    // Z axis gyro offset (low byte)
#define MPUREG_SMPLRT_DIV                               0x19    // sample rate.  Fsample= 1Khz/(<this value>+1) = 200Hz
#       define MPUREG_SMPLRT_1000HZ                             0x00
#       define MPUREG_SMPLRT_500HZ                              0x01
#       define MPUREG_SMPLRT_250HZ                              0x03
#       define MPUREG_SMPLRT_200HZ                              0x04
#       define MPUREG_SMPLRT_100HZ                              0x09
#       define MPUREG_SMPLRT_50HZ                               0x13
#define MPUREG_CONFIG                                           0x1A
#define MPUREG_GYRO_CONFIG                                      0x1B
// bit definitions for MPUREG_GYRO_CONFIG
#       define BITS_GYRO_FS_250DPS                              0x00
#       define BITS_GYRO_FS_500DPS                              0x08
#       define BITS_GYRO_FS_1000DPS                             0x10
#       define BITS_GYRO_FS_2000DPS                             0x18
#       define BITS_GYRO_FS_MASK                                0x18    // only bits 3 and 4 are used for gyro full scale so use this to mask off other bits
#       define BITS_GYRO_ZGYRO_SELFTEST                 0x20
#       define BITS_GYRO_YGYRO_SELFTEST                 0x40
#       define BITS_GYRO_XGYRO_SELFTEST                 0x80
#define MPUREG_ACCEL_CONFIG                             0x1C
#define MPUREG_MOT_THR                                  0x1F    // detection threshold for Motion interrupt generation.  Motion is detected when the absolute value of any of the accelerometer measurements exceeds this
#define MPUREG_MOT_DUR                                  0x20    // duration counter threshold for Motion interrupt generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit of 1 LSB = 1 ms
#define MPUREG_ZRMOT_THR                                0x21    // detection threshold for Zero Motion interrupt generation.
#define MPUREG_ZRMOT_DUR                                0x22    // duration counter threshold for Zero Motion interrupt generation. The duration counter ticks at 16 Hz, therefore ZRMOT_DUR has a unit of 1 LSB = 64 ms.
#define MPUREG_FIFO_EN                                  0x23
#define MPUREG_INT_PIN_CFG                              0x37
#       define BIT_INT_RD_CLEAR                                 0x10    // clear the interrupt when any read occurs
#       define BIT_LATCH_INT_EN                                 0x20    // latch data ready pin 
#define MPUREG_INT_ENABLE                               0x38
// bit definitions for MPUREG_INT_ENABLE
#       define BIT_RAW_RDY_EN                                   0x01
#       define BIT_DMP_INT_EN                                   0x02    // enabling this bit (DMP_INT_EN) also enables RAW_RDY_EN it seems
#       define BIT_UNKNOWN_INT_EN                               0x04
#       define BIT_I2C_MST_INT_EN                               0x08
#       define BIT_FIFO_OFLOW_EN                                0x10
#       define BIT_ZMOT_EN                                              0x20
#       define BIT_MOT_EN                                               0x40
#       define BIT_FF_EN                                                0x80
#define MPUREG_INT_STATUS                               0x3A
// bit definitions for MPUREG_INT_STATUS (same bit pattern as above because this register shows what interrupt actually fired)
#       define BIT_RAW_RDY_INT                                  0x01
#       define BIT_DMP_INT                                              0x02
#       define BIT_UNKNOWN_INT                                  0x04
#       define BIT_I2C_MST_INT                                  0x08
#       define BIT_FIFO_OFLOW_INT                               0x10
#       define BIT_ZMOT_INT                                             0x20
#       define BIT_MOT_INT                                              0x40
#       define BIT_FF_INT                                               0x80
#define MPUREG_ACCEL_XOUT_H                             0x3B
#define MPUREG_ACCEL_XOUT_L                             0x3C
#define MPUREG_ACCEL_YOUT_H                             0x3D
#define MPUREG_ACCEL_YOUT_L                             0x3E
#define MPUREG_ACCEL_ZOUT_H                             0x3F
#define MPUREG_ACCEL_ZOUT_L                             0x40
#define MPUREG_TEMP_OUT_H                               0x41
#define MPUREG_TEMP_OUT_L                               0x42
#define MPUREG_GYRO_XOUT_H                              0x43
#define MPUREG_GYRO_XOUT_L                              0x44
#define MPUREG_GYRO_YOUT_H                              0x45
#define MPUREG_GYRO_YOUT_L                              0x46
#define MPUREG_GYRO_ZOUT_H                              0x47
#define MPUREG_GYRO_ZOUT_L                              0x48
#define MPUREG_USER_CTRL                                0x6A
// bit definitions for MPUREG_USER_CTRL
#       define BIT_USER_CTRL_SIG_COND_RESET             0x01            // resets signal paths and results registers for all sensors (gyros, accel, temp)
#       define BIT_USER_CTRL_I2C_MST_RESET              0x02            // reset I2C Master (only applicable if I2C_MST_EN bit is set)
#       define BIT_USER_CTRL_FIFO_RESET                 0x04            // Reset (i.e. clear) FIFO buffer
#       define BIT_USER_CTRL_DMP_RESET                  0x08            // Reset DMP
#       define BIT_USER_CTRL_I2C_IF_DIS                 0x10            // Disable primary I2C interface and enable hal.spi->interface
#       define BIT_USER_CTRL_I2C_MST_EN                 0x20            // Enable MPU to act as the I2C Master to external slave sensors
#       define BIT_USER_CTRL_FIFO_EN                    0x40            // Enable FIFO operations
#       define BIT_USER_CTRL_DMP_EN                             0x80            // Enable DMP operations
#define MPUREG_PWR_MGMT_1                               0x6B
#       define BIT_PWR_MGMT_1_CLK_INTERNAL              0x00            // clock set to internal 8Mhz oscillator
#       define BIT_PWR_MGMT_1_CLK_XGYRO                 0x01            // PLL with X axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_YGYRO                 0x02            // PLL with Y axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_ZGYRO                 0x03            // PLL with Z axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_EXT32KHZ              0x04            // PLL with external 32.768kHz reference
#       define BIT_PWR_MGMT_1_CLK_EXT19MHZ              0x05            // PLL with external 19.2MHz reference
#       define BIT_PWR_MGMT_1_CLK_STOP                  0x07            // Stops the clock and keeps the timing generator in reset
#       define BIT_PWR_MGMT_1_TEMP_DIS                  0x08            // disable temperature sensor
#       define BIT_PWR_MGMT_1_CYCLE                             0x20            // put sensor into cycle mode.  cycles between sleep mode and waking up to take a single sample of data from active sensors at a rate determined by LP_WAKE_CTRL
#       define BIT_PWR_MGMT_1_SLEEP                             0x40            // put sensor into low power sleep mode
#       define BIT_PWR_MGMT_1_DEVICE_RESET              0x80            // reset entire device
#define MPUREG_PWR_MGMT_2                               0x6C            // allows the user to configure the frequency of wake-ups in Accelerometer Only Low Power Mode
#define MPUREG_BANK_SEL                                 0x6D            // DMP bank selection register (used to indirectly access DMP registers)
#define MPUREG_MEM_START_ADDR                   0x6E            // DMP memory start address (used to indirectly write to dmp memory)
#define MPUREG_MEM_R_W                                  0x6F            // DMP related register
#define MPUREG_DMP_CFG_1                                0x70            // DMP related register
#define MPUREG_DMP_CFG_2                                0x71            // DMP related register
#define MPUREG_FIFO_COUNTH                              0x72
#define MPUREG_FIFO_COUNTL                              0x73
#define MPUREG_FIFO_R_W                                 0x74
#define MPUREG_WHOAMI                                   0x75


// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BITS_DLPF_CFG_256HZ_NOLPF2              0x00
#define BITS_DLPF_CFG_188HZ                             0x01
#define BITS_DLPF_CFG_98HZ                              0x02
#define BITS_DLPF_CFG_42HZ                              0x03
#define BITS_DLPF_CFG_20HZ                              0x04
#define BITS_DLPF_CFG_10HZ                              0x05
#define BITS_DLPF_CFG_5HZ                               0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF              0x07
#define BITS_DLPF_CFG_MASK                              0x07

// Product ID Description for MPU6000
// high 4 bits  low 4 bits
// Product Name	Product Revision
#define MPU6000ES_REV_C4                        0x14    // 0001			0100
#define MPU6000ES_REV_C5                        0x15    // 0001			0101
#define MPU6000ES_REV_D6                        0x16    // 0001			0110
#define MPU6000ES_REV_D7                        0x17    // 0001			0111
#define MPU6000ES_REV_D8                        0x18    // 0001			1000
#define MPU6000_REV_C4                          0x54    // 0101			0100
#define MPU6000_REV_C5                          0x55    // 0101			0101
#define MPU6000_REV_D6                          0x56    // 0101			0110
#define MPU6000_REV_D7                          0x57    // 0101			0111
#define MPU6000_REV_D8                          0x58    // 0101			1000
#define MPU6000_REV_D9                          0x59    // 0101			1001

// DMP output rate constants
#define MPU6000_200HZ                           0x00    // default value
#define MPU6000_100HZ                           0x01
#define MPU6000_66HZ                            0x02
#define MPU6000_50HZ                            0x03

// DMP FIFO constants
// Default quaternion FIFO size (4*4) + Footer(2)
#define FIFO_PACKET_SIZE 18 
// Rate of the gyro bias from gravity correction (200Hz/4) => 50Hz
#define GYRO_BIAS_FROM_GRAVITY_RATE 4
// Default gain for accel fusion (with gyros)
#define DEFAULT_ACCEL_FUSION_GAIN       0x80

/*
 *  RM-MPU-6000A-00.pdf, page 33, section 4.25 lists LSB sensitivity of
 *  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
 */
const float AP_InertialSensor_MPU6000::_gyro_scale = (0.0174532 / 16.4);

/* pch: I believe the accel and gyro indicies are correct
 *      but somone else should please confirm.
 *
 * jamesjb: Y and Z axes are flipped on the PX4FMU
 */
const uint8_t AP_InertialSensor_MPU6000::_gyro_data_index[3]  = { 5, 4, 6 };
const uint8_t AP_InertialSensor_MPU6000::_accel_data_index[3] = { 1, 0, 2 };

#if CONFIG_HAL_BOARD == HAL_BOARD_SMACCM
const int8_t AP_InertialSensor_MPU6000::_gyro_data_sign[3]  = { 1, -1, 1 };
const int8_t AP_InertialSensor_MPU6000::_accel_data_sign[3] = { 1, -1, 1 };
#else
const int8_t AP_InertialSensor_MPU6000::_gyro_data_sign[3]   = { 1, 1, -1 };
const int8_t AP_InertialSensor_MPU6000::_accel_data_sign[3]  = { 1, 1, -1 };
#endif

const uint8_t AP_InertialSensor_MPU6000::_temp_data_index = 3;

int16_t AP_InertialSensor_MPU6000::_mpu6000_product_id = AP_PRODUCT_ID_NONE;
AP_HAL::DigitalSource *AP_InertialSensor_MPU6000::_drdy_pin = NULL;

// time we start collecting sample (reset on update)
// time latest sample was collected
static volatile uint32_t _last_sample_time_micros = 0;

// DMP related static variables
bool AP_InertialSensor_MPU6000::_dmp_initialised = false;
// high byte of number of elements in fifo buffer
uint8_t AP_InertialSensor_MPU6000::_fifoCountH;
// low byte of number of elements in fifo buffer
uint8_t AP_InertialSensor_MPU6000::_fifoCountL;
// holds the 4 quaternions representing attitude taken directly from the DMP
Quaternion AP_InertialSensor_MPU6000::quaternion;

/* Static SPI device driver */
AP_HAL::SPIDeviceDriver* AP_InertialSensor_MPU6000::_spi = NULL;
AP_HAL::Semaphore* AP_InertialSensor_MPU6000::_spi_sem = NULL;

/*
 *  RM-MPU-6000A-00.pdf, page 31, section 4.23 lists LSB sensitivity of
 *  accel as 4096 LSB/mg at scale factor of +/- 8g (AFS_SEL==2)
 *
 *  See note below about accel scaling of engineering sample MPU6k
 *  variants however
 */

AP_InertialSensor_MPU6000::AP_InertialSensor_MPU6000() : AP_InertialSensor()
{
    _temp = 0;
    _initialised = false;
    _dmp_initialised = false;
}

uint16_t AP_InertialSensor_MPU6000::_init_sensor( Sample_rate sample_rate )
{
    if (_initialised) return _mpu6000_product_id;
    _initialised = true;

    _spi = hal.spi->device(AP_HAL::SPIDevice_MPU6000);
    _spi_sem = _spi->get_semaphore();

    /* Pin 70 defined especially to hook
       up PE6 to the hal.gpio abstraction.
       (It is not a valid pin under Arduino.) */
    _drdy_pin = hal.gpio->channel(70);

    hal.scheduler->suspend_timer_procs();

    uint8_t tries = 0;
    do {
        bool success = hardware_init(sample_rate);
        if (success) {
            hal.scheduler->delay(5+2);
            if (_data_ready()) {
                break;
            } else {
                hal.console->println_P(
                        PSTR("MPU6000 startup failed: no data ready"));
            }
        }
        if (tries++ > 5) {
            hal.scheduler->panic(PSTR("PANIC: failed to boot MPU6000 5 times")); 
        }
    } while (1);

    hal.scheduler->resume_timer_procs();
    

    /* read the first lot of data.
     * _read_data_transaction requires the spi semaphore to be taken by
     * its caller. */
    _last_sample_time_micros = hal.scheduler->micros();
    _read_data_transaction();

    // start the timer process to read samples
    hal.scheduler->register_timer_process(_poll_data);

#if MPU6000_DEBUG
    _dump_registers();
#endif
    return _mpu6000_product_id;
}

// accumulation in ISR - must be read with interrupts disabled
// the sum of the values since last read
static volatile int32_t _sum[7];

// how many values we've accumulated since last read
static volatile uint16_t _count;

/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

void AP_InertialSensor_MPU6000::wait_for_sample()
{
    uint32_t tstart = hal.scheduler->micros();
    while (num_samples_available() == 0) {
        uint32_t now = hal.scheduler->micros();
        uint32_t dt = now - tstart;
        if (dt > 50000) {
            hal.scheduler->panic(
                    PSTR("PANIC: AP_InertialSensor_MPU6000::update "
                        "waited 50ms for data from interrupt"));
        }
    }
}

bool AP_InertialSensor_MPU6000::update( void )
{
    int32_t sum[7];
    float count_scale;
    Vector3f accel_scale = _accel_scale.get();

    // wait for at least 1 sample
    wait_for_sample();

    // disable timer procs for mininum time
    hal.scheduler->suspend_timer_procs();
    /** ATOMIC SECTION w/r/t TIMER PROCESS */
    {
        for (int i=0; i<7; i++) {
            sum[i] = _sum[i];
            _sum[i] = 0;
        }

        _num_samples = _count;
        _count = 0;
    }
    hal.scheduler->resume_timer_procs();

    count_scale = 1.0f / _num_samples;

    _gyro  = Vector3f(_gyro_data_sign[0] * sum[_gyro_data_index[0]],
                      _gyro_data_sign[1] * sum[_gyro_data_index[1]],
                      _gyro_data_sign[2] * sum[_gyro_data_index[2]]);
    _gyro.rotate(_board_orientation);
    _gyro *= _gyro_scale * count_scale;
    _gyro -= _gyro_offset;

    _accel   = Vector3f(_accel_data_sign[0] * sum[_accel_data_index[0]],
                        _accel_data_sign[1] * sum[_accel_data_index[1]],
                        _accel_data_sign[2] * sum[_accel_data_index[2]]);
    _accel.rotate(_board_orientation);
    _accel *= count_scale * MPU6000_ACCEL_SCALE_1G;
    _accel.x *= accel_scale.x;
    _accel.y *= accel_scale.y;
    _accel.z *= accel_scale.z;
    _accel -= _accel_offset;

    _temp    = _temp_to_celsius(sum[_temp_data_index] * count_scale);

    if (_last_filter_hz != _mpu6000_filter) {
        if (_spi_sem->take(10)) {
            _set_filter_register(_mpu6000_filter, 0);
            _spi_sem->give();
        }
    }

    return true;
}

/*================ HARDWARE FUNCTIONS ==================== */

/**
 * Return true if the MPU6000 has new data available for reading.
 *
 * We use the data ready pin if it is available.  Otherwise, read the
 * status register.
 */
bool AP_InertialSensor_MPU6000::_data_ready()
{
    if (_drdy_pin) {
        return _drdy_pin->read() != 0;
    }

    if (hal.scheduler->in_timerprocess()) {
        bool got = _spi_sem->take_nonblocking();
        if (got) {
            uint8_t status = _register_read(MPUREG_INT_STATUS);
            _spi_sem->give();
            return (status & BIT_RAW_RDY_INT) != 0;
        } else {
            return false;
        }
    } else {
        bool got = _spi_sem->take(10);
        if (got) {
            uint8_t status = _register_read(MPUREG_INT_STATUS);
            _spi_sem->give();
            return (status & BIT_RAW_RDY_INT) != 0;
        } else {
            hal.scheduler->panic(
                PSTR("PANIC: AP_InertialSensor_MPU6000::_data_ready failed to "
                     "take SPI semaphore synchronously"));
        }
    }
    return false;
}

/**
 * Timer process to poll for new data from the MPU6000.
 */
void AP_InertialSensor_MPU6000::_poll_data(uint32_t now)
{
    if (_data_ready()) {
        if (hal.scheduler->in_timerprocess()) {
            _read_data_from_timerprocess();
        } else {
            /* Synchronous read - take semaphore */
            bool got = _spi_sem->take(10);
            if (got) {
                _last_sample_time_micros = hal.scheduler->micros();
                _read_data_transaction(); 
                _spi_sem->give();
            } else {
                hal.scheduler->panic(
                        PSTR("PANIC: AP_InertialSensor_MPU6000::_poll_data "
                             "failed to take SPI semaphore synchronously"));
            }
        }
    }
}

/*
 *  this is called from the _poll_data, in the timer process context.
 *  when the MPU6000 has new sensor data available and add it to _sum[] to
 *  ensure this is the case, these other devices must perform their spi reads
 *  after being called by the AP_TimerProcess.
 */
void AP_InertialSensor_MPU6000::_read_data_from_timerprocess()
{
    static uint8_t semfail_ctr = 0;
    bool got = _spi_sem->take_nonblocking();
    if (!got) { 
        semfail_ctr++;
        if (semfail_ctr > 100) {
            hal.scheduler->panic(PSTR("PANIC: failed to take SPI semaphore "
                        "100 times in AP_InertialSensor_MPU6000::"
                        "_read_data_from_timerprocess"));
        }
        return;
    } else {
        semfail_ctr = 0;
    }   

    _last_sample_time_micros = hal.scheduler->micros();
    _read_data_transaction();

    _spi_sem->give();
}

void AP_InertialSensor_MPU6000::_read_data_transaction() {
    /* one resister address followed by seven 2-byte registers */
    uint8_t tx[15];
    uint8_t rx[15];
    memset(tx,0,15);
    tx[0] = MPUREG_ACCEL_XOUT_H | 0x80;
    _spi->transaction(tx, rx, 15);

    for (uint8_t i = 0; i < 7; i++) {
        _sum[i] += (int16_t)(((uint16_t)rx[2*i+1] << 8) | rx[2*i+2]);
    }   
    
    _count++;
    if (_count == 0) {
        // rollover - v unlikely
        memset((void*)_sum, 0, sizeof(_sum));
    }

    // should also read FIFO data if enabled
    if( _dmp_initialised ) {
        if( FIFO_ready() ) {
            FIFO_getPacket();
        }
    }
}

uint8_t AP_InertialSensor_MPU6000::_register_read( uint8_t reg )
{
    uint8_t addr = reg | 0x80; // Set most significant bit

    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = addr;
    tx[1] = 0;
    _spi->transaction(tx, rx, 2);

    return rx[1];
}

void AP_InertialSensor_MPU6000::register_write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg;
    tx[1] = val;
    _spi->transaction(tx, rx, 2);
}

/*
  set the DLPF filter frequency. Assumes caller has taken semaphore
 */
void AP_InertialSensor_MPU6000::_set_filter_register(uint8_t filter_hz, uint8_t default_filter)
{
    uint8_t filter = default_filter;
    // choose filtering frequency
    switch (filter_hz) {
    case 5:
        filter = BITS_DLPF_CFG_5HZ;
        break;
    case 10:
        filter = BITS_DLPF_CFG_10HZ;
        break;
    case 20:
        filter = BITS_DLPF_CFG_20HZ;
        break;
    case 42:
        filter = BITS_DLPF_CFG_42HZ;
        break;
    case 98:
        filter = BITS_DLPF_CFG_98HZ;
        break;
    }

    if (filter != 0) {
        _last_filter_hz = filter_hz;

        register_write(MPUREG_CONFIG, filter);
    }
}


bool AP_InertialSensor_MPU6000::hardware_init(Sample_rate sample_rate)
{
    if (!_spi_sem->take(100)) {
        hal.scheduler->panic(PSTR("MPU6000: Unable to get semaphore"));
    }

    // Chip reset
    uint8_t tries;
    for (tries = 0; tries<5; tries++) {
        register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);
        hal.scheduler->delay(100);

        // Wake up device and select GyroZ clock. Note that the
        // MPU6000 starts up in sleep mode, and it can take some time
        // for it to come out of sleep
        register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_ZGYRO);
        hal.scheduler->delay(5);

        // check it has woken up
        if (_register_read(MPUREG_PWR_MGMT_1) == BIT_PWR_MGMT_1_CLK_ZGYRO) {
            break;
        }
#if MPU6000_DEBUG
        _dump_registers();
#endif
    }
    if (tries == 5) {
        hal.console->println_P(PSTR("Failed to boot MPU6000 5 times"));
        _spi_sem->give();
        return false;
    }

    register_write(MPUREG_PWR_MGMT_2, 0x00);            // only used for wake-up in accelerometer only low power mode
    hal.scheduler->delay(1);

    // Disable I2C bus (recommended on datasheet)
    register_write(MPUREG_USER_CTRL, BIT_USER_CTRL_I2C_IF_DIS);
    hal.scheduler->delay(1);

    uint8_t default_filter;

    // sample rate and filtering
    // to minimise the effects of aliasing we choose a filter
    // that is less than half of the sample rate
    switch (sample_rate) {
    case RATE_50HZ:
        // this is used for plane and rover, where noise resistance is
        // more important than update rate. Tests on an aerobatic plane
        // show that 10Hz is fine, and makes it very noise resistant
        default_filter = BITS_DLPF_CFG_10HZ;
        _sample_shift = 2;
        break;
    case RATE_100HZ:
        default_filter = BITS_DLPF_CFG_20HZ;
        _sample_shift = 1;
        break;
    case RATE_200HZ:
    default:
        default_filter = BITS_DLPF_CFG_20HZ;
        _sample_shift = 0;
        break;
    }

    _set_filter_register(_mpu6000_filter, default_filter);

    // set sample rate to 200Hz, and use _sample_divider to give
    // the requested rate to the application
    register_write(MPUREG_SMPLRT_DIV, MPUREG_SMPLRT_200HZ);
    hal.scheduler->delay(1);

    register_write(MPUREG_GYRO_CONFIG, BITS_GYRO_FS_2000DPS);  // Gyro scale 2000º/s
    hal.scheduler->delay(1);

    // read the product ID rev c has 1/2 the sensitivity of rev d
    _mpu6000_product_id = _register_read(MPUREG_PRODUCT_ID);
    //Serial.printf("Product_ID= 0x%x\n", (unsigned) _mpu6000_product_id);

    if ((_mpu6000_product_id == MPU6000ES_REV_C4) || (_mpu6000_product_id == MPU6000ES_REV_C5) ||
        (_mpu6000_product_id == MPU6000_REV_C4)   || (_mpu6000_product_id == MPU6000_REV_C5)) {
        // Accel scale 8g (4096 LSB/g)
        // Rev C has different scaling than rev D
        register_write(MPUREG_ACCEL_CONFIG,1<<3);
    } else {
        // Accel scale 8g (4096 LSB/g)
        register_write(MPUREG_ACCEL_CONFIG,2<<3);
    }
    hal.scheduler->delay(1);

    // configure interrupt to fire when new data arrives
    register_write(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);
    hal.scheduler->delay(1);

    // clear interrupt on any read, and hold the data ready pin high
    // until we clear the interrupt
    register_write(MPUREG_INT_PIN_CFG, BIT_INT_RD_CLEAR | BIT_LATCH_INT_EN);
    hal.scheduler->delay(1);

    _spi_sem->give();

    return true;
}

float AP_InertialSensor_MPU6000::_temp_to_celsius ( uint16_t regval )
{
    /* TODO */
    return 20.0;
}

// return the MPU6k gyro drift rate in radian/s/s
// note that this is much better than the oilpan gyros
float AP_InertialSensor_MPU6000::get_gyro_drift_rate(void)
{
    // 0.5 degrees/second/minute
    return ToRad(0.5/60);
}

// get number of samples read from the sensors
uint16_t AP_InertialSensor_MPU6000::num_samples_available()
{
    _poll_data(0);
    return _count >> _sample_shift;
}


#if MPU6000_DEBUG
// dump all config registers - used for debug
void AP_InertialSensor_MPU6000::_dump_registers(void)
{
    hal.console->println_P(PSTR("MPU6000 registers"));
    for (uint8_t reg=MPUREG_PRODUCT_ID; reg<=108; reg++) {
        uint8_t v = _register_read(reg);
        hal.console->printf_P(PSTR("%02x:%02x "), (unsigned)reg, (unsigned)v);
        if ((reg - (MPUREG_PRODUCT_ID-1)) % 16 == 0) {
            hal.console->println();
        }
    }
    hal.console->println();
}
#endif


// get_delta_time returns the time period in seconds overwhich the sensor data was collected
float AP_InertialSensor_MPU6000::get_delta_time() 
{
    // the sensor runs at 200Hz
    return 0.005 * _num_samples;
}

// Update gyro offsets with new values.  Offsets provided in as scaled deg/sec values
void AP_InertialSensor_MPU6000::push_gyro_offsets_to_dmp()
{
    Vector3f gyro_offsets = _gyro_offset.get();

    int16_t offsetX = gyro_offsets.x / _gyro_scale * _gyro_data_sign[0];
    int16_t offsetY = gyro_offsets.y / _gyro_scale * _gyro_data_sign[1];
    int16_t offsetZ = gyro_offsets.z / _gyro_scale * _gyro_data_sign[2];

    set_dmp_gyro_offsets(offsetX, offsetY, offsetZ);

    // remove ins level offsets to avoid double counting
    gyro_offsets.x = 0;
    gyro_offsets.y = 0;
    gyro_offsets.z = 0;
    _gyro_offset = gyro_offsets;
}

// Update gyro offsets with new values. New offset values are substracted to actual offset values.
// offset values in gyro LSB units (as read from registers)
void AP_InertialSensor_MPU6000::set_dmp_gyro_offsets(int16_t offsetX, int16_t offsetY, int16_t offsetZ)
{
    int16_t aux_int;

    if (offsetX != 0) {
        // Read actual value
        aux_int = (_register_read(MPUREG_XG_OFFS_USRH)<<8) | _register_read(MPUREG_XG_OFFS_USRL);
        aux_int -= offsetX<<1;           // Adjust to internal units
        // Write to MPU registers
        register_write(MPUREG_XG_OFFS_USRH, (aux_int>>8)&0xFF);
        register_write(MPUREG_XG_OFFS_USRL, aux_int&0xFF);
    }
    if (offsetY != 0) {
        aux_int = (_register_read(MPUREG_YG_OFFS_USRH)<<8) | _register_read(MPUREG_YG_OFFS_USRL);
        aux_int -= offsetY<<1;           // Adjust to internal units
        // Write to MPU registers
        register_write(MPUREG_YG_OFFS_USRH, (aux_int>>8)&0xFF);
        register_write(MPUREG_YG_OFFS_USRL, aux_int&0xFF);
    }
    if (offsetZ != 0) {
        aux_int = (_register_read(MPUREG_ZG_OFFS_USRH)<<8) | _register_read(MPUREG_ZG_OFFS_USRL);
        aux_int -= offsetZ<<1;           // Adjust to internal units
        // Write to MPU registers
        register_write(MPUREG_ZG_OFFS_USRH, (aux_int>>8)&0xFF);
        register_write(MPUREG_ZG_OFFS_USRL, aux_int&0xFF);
    }
}

// Update accel offsets with new values.  Offsets provided in as scaled values (1G)
void AP_InertialSensor_MPU6000::push_accel_offsets_to_dmp()
{
    Vector3f accel_offset = _accel_offset.get();
    Vector3f accel_scale = _accel_scale.get();
    int16_t offsetX = accel_offset.x / (accel_scale.x * _accel_data_sign[0] * MPU6000_ACCEL_SCALE_1G);
    int16_t offsetY = accel_offset.y / (accel_scale.y * _accel_data_sign[1] * MPU6000_ACCEL_SCALE_1G);
    int16_t offsetZ = accel_offset.z / (accel_scale.z * _accel_data_sign[2] * MPU6000_ACCEL_SCALE_1G);

    // strangely x and y are reversed
    set_dmp_accel_offsets(offsetY, offsetX, offsetZ);
}

// set_accel_offsets - adds an offset to acceleromter readings
// This is useful for dynamic acceleration correction (for example centripetal force correction)
// and for the initial offset calibration
// Input, accel offsets for X,Y and Z in LSB units (as read from raw values)
void AP_InertialSensor_MPU6000::set_dmp_accel_offsets(int16_t offsetX, int16_t offsetY, int16_t offsetZ)
{
    int aux_int;
    uint8_t regs[2];

    // Write accel offsets to DMP memory...
    // TO-DO: why don't we write to main accel offset registries? i.e. MPUREG_XA_OFFS_H
    aux_int = offsetX>>1;                       // Transform to internal units
    regs[0]=(aux_int>>8)&0xFF;
    regs[1]=aux_int&0xFF;
    dmp_register_write(0x01,0x08,2,regs);               // key KEY_D_1_8  Accel X offset

    aux_int = offsetY>>1;
    regs[0]=(aux_int>>8)&0xFF;
    regs[1]=aux_int&0xFF;
    dmp_register_write(0x01,0x0A,2,regs);               // key KEY_D_1_10  Accel Y offset

    aux_int = offsetZ>>1;
    regs[0]=(aux_int>>8)&0xFF;
    regs[1]=aux_int&0xFF;
    dmp_register_write(0x01,0x02,2,regs);               // key KEY_D_1_2  Accel Z offset
}

// dmp_register_write - method to write to dmp's registers
//    the dmp is logically separated from the main mpu6000.  To write a block of memory to the DMP's memory you
//    write the "bank" and starting address into two of the main MPU's registers, then write the data one byte
//    at a time into the MPUREG_MEM_R_W register
void AP_InertialSensor_MPU6000::dmp_register_write(uint8_t bank, uint8_t address, uint8_t num_bytes, uint8_t data[])
{
    register_write(MPUREG_BANK_SEL,bank);
    register_write(MPUREG_MEM_START_ADDR,address);

    _spi->cs_assert();
    _spi->transfer(MPUREG_MEM_R_W);
    for (uint8_t i=0; i<num_bytes; i++) {
        _spi->transfer(data[i]);
    }
    _spi->cs_release();
}

// MPU6000 DMP initialization
// this should be called after hardware_init if you wish to enable the dmp
void AP_InertialSensor_MPU6000::dmp_init()
{
    uint8_t regs[4];    // for writing to dmp

    // ensure we only initialise once
    if( _dmp_initialised ) {
        return;
    }

    // load initial values into DMP memory
    dmp_load_mem();

    dmp_set_gyro_calibration();
    dmp_set_accel_calibration();
    dmp_apply_endian_accel();
    dmp_set_mpu_sensors();
    dmp_set_bias_none();
    dmp_set_fifo_interrupt();
    dmp_send_quaternion();                      // By default we only send the quaternion to the FIFO (18 bytes packet size)
    dmp_set_fifo_rate(MPU6000_200HZ);                   // 200Hz DMP output rate

    register_write(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN | BIT_DMP_INT_EN ); // configure interrupts to fire only when new data arrives from DMP (in fifo buffer)

    // Randy: no idea what this does
    register_write(MPUREG_DMP_CFG_1, 0x03);             //MPUREG_DMP_CFG_1, 0x03
    register_write(MPUREG_DMP_CFG_2, 0x00);             //MPUREG_DMP_CFG_2, 0x00

    //inv_state_change_fifo
    regs[0] = 0xFF;
    regs[1] = 0xFF;
    dmp_register_write(0x01, 0xB2, 0x02, regs); // D_1_178

    // ?? FIFO ??
    regs[0] = 0x09;
    regs[1] = 0x23;
    regs[2] = 0xA1;
    regs[3] = 0x35;
    dmp_register_write(0x01, 0x90, 0x04, regs); // D_1_144

    //register_write(MPUREG_USER_CTRL, BIT_USER_CTRL_FIFO_RESET);		//MPUREG_USER_CTRL, BIT_FIFO_RST
    FIFO_reset();

    FIFO_ready();

    //register_write(MPUREG_USER_CTRL, 0x00);		// MPUREG_USER_CTRL, 0.  TO-DO: is all this setting of USER_CTRL really necessary?

    register_write(MPUREG_USER_CTRL, BIT_USER_CTRL_FIFO_RESET);         //MPUREG_USER_CTRL, BIT_FIFO_RST.  TO-DO: replace this call with FIFO_reset()?
    register_write(MPUREG_USER_CTRL, 0x00);             // MPUREG_USER_CTRL: 0
    register_write(MPUREG_USER_CTRL, BIT_USER_CTRL_DMP_EN | BIT_USER_CTRL_FIFO_EN | BIT_USER_CTRL_DMP_RESET);

    // Set the gain of the accel in the sensor fusion
    dmp_set_sensor_fusion_accel_gain(DEFAULT_ACCEL_FUSION_GAIN); // default value

    // dmp initialisation complete
    _dmp_initialised = true;

}

// dmp_reset - reset dmp (required for changes in gains or offsets to take effect)
void AP_InertialSensor_MPU6000::dmp_reset()
{
    //uint8_t tmp = register_read(MPUREG_USER_CTRL);
    //tmp |= BIT_USER_CTRL_DMP_RESET;
    //register_write(MPUREG_USER_CTRL,tmp);
    register_write(MPUREG_USER_CTRL, BIT_USER_CTRL_FIFO_RESET);                 //MPUREG_USER_CTRL, BIT_FIFO_RST.  TO-DO: replace this call with FIFO_reset()?
    register_write(MPUREG_USER_CTRL, 0x00);             // MPUREG_USER_CTRL: 0
    register_write(MPUREG_USER_CTRL, BIT_USER_CTRL_DMP_EN | BIT_USER_CTRL_FIFO_EN | BIT_USER_CTRL_DMP_RESET);
}

// New data packet in FIFO?
bool AP_InertialSensor_MPU6000::FIFO_ready()
{
    _fifoCountH = _register_read(MPUREG_FIFO_COUNTH);
    _fifoCountL = _register_read(MPUREG_FIFO_COUNTL);
    if(_fifoCountL == FIFO_PACKET_SIZE) {
        return 1;
    }
    else{
        //We should not reach this point or maybe we have more than one packet (we should manage this!)
        FIFO_reset();
        return 0;
    }
}

// FIFO_reset - reset/clear FIFO buffer used to capture attitude information from DMP
void AP_InertialSensor_MPU6000::FIFO_reset()
{
    uint8_t temp;
    temp = _register_read(MPUREG_USER_CTRL);
    temp = temp | BIT_USER_CTRL_FIFO_RESET;             // FIFO RESET BIT
    register_write(MPUREG_USER_CTRL, temp);
}

// FIFO_getPacket - read an attitude packet from FIFO buffer
// TO-DO: interpret results instead of just dumping into a buffer
void AP_InertialSensor_MPU6000::FIFO_getPacket()
{
    uint8_t i;
    int16_t q_data[4];
    uint8_t addr = MPUREG_FIFO_R_W | 0x80;      // Set most significant bit to indicate a read
    uint8_t received_packet[DMP_FIFO_BUFFER_SIZE];    // FIFO packet buffer
    _spi->cs_assert();
    _spi->transfer(addr);                                 // send address we want to read from
    for(i = 0; i < _fifoCountL; i++) {
        received_packet[i] = _spi->transfer(0);          // request value
    }
    _spi->cs_release();

    // we are using 16 bits resolution
    q_data[0] = (int16_t) ((((uint16_t) received_packet[0]) << 8) + ((uint16_t) received_packet[1]));
    q_data[1] = (int16_t) ((((uint16_t) received_packet[4]) << 8) + ((uint16_t) received_packet[5]));
    q_data[2] = (int16_t) ((((uint16_t) received_packet[8]) << 8) + ((uint16_t) received_packet[9]));
    q_data[3] = (int16_t) ((((uint16_t) received_packet[12]) << 8) + ((uint16_t) received_packet[13]));

    quaternion.q1 = ((float)q_data[0]) / 16384.0f;       // convert from fixed point to float
    quaternion.q2 = ((float)q_data[2]) / 16384.0f;       // convert from fixed point to float
    quaternion.q3 = ((float)q_data[1]) / 16384.0f;       // convert from fixed point to float
    quaternion.q4 = ((float)-q_data[3]) / 16384.0f;       // convert from fixed point to float
}

// dmp_set_gyro_calibration - apply default gyro calibration FS=2000dps and default orientation
void AP_InertialSensor_MPU6000::dmp_set_gyro_calibration()
{
    uint8_t regs[4];
    regs[0]=0x4C;
    regs[1]=0xCD;
    regs[2]=0x6C;
    dmp_register_write(0x03, 0x7B, 0x03, regs);                 //FCFG_1 inv_set_gyro_calibration
    regs[0]=0x36;
    regs[1]=0x56;
    regs[2]=0x76;
    dmp_register_write(0x03, 0xAB, 0x03, regs);                 //FCFG_3 inv_set_gyro_calibration
    regs[0]=0x02;
    regs[1]=0xCB;
    regs[2]=0x47;
    regs[3]=0xA2;
    dmp_register_write(0x00, 0x68, 0x04, regs);                 //D_0_104 inv_set_gyro_calibration
    regs[0]=0x00;
    regs[1]=0x05;
    regs[2]=0x8B;
    regs[3]=0xC1;
    dmp_register_write(0x02, 0x18, 0x04, regs);                 //D_0_24 inv_set_gyro_calibration
}

// dmp_set_accel_calibration - apply default accel calibration scale=8g and default orientation
void AP_InertialSensor_MPU6000::dmp_set_accel_calibration()
{
    uint8_t regs[6];
    regs[0]=0x00;
    regs[1]=0x00;
    regs[2]=0x00;
    regs[3]=0x00;
    dmp_register_write(0x01, 0x0C, 0x04, regs);                 //D_1_152 inv_set_accel_calibration
    regs[0]=0x0C;
    regs[1]=0xC9;
    regs[2]=0x2C;
    regs[3]=0x97;
    regs[4]=0x97;
    regs[5]=0x97;
    dmp_register_write(0x03, 0x7F, 0x06, regs);                 //FCFG_2 inv_set_accel_calibration
    regs[0]=0x26;
    regs[1]=0x46;
    regs[2]=0x66;
    dmp_register_write(0x03, 0x89, 0x03, regs);                 //FCFG_7 inv_set_accel_calibration
    // accel range, 0x20,0x00 => 2g, 0x10,0x00=>4g    regs= (1073741824/accel_scale*65536)
    //regs[0]=0x20;	// 2g
    regs[0]=0x08;               // 8g
    regs[1]=0x00;
    dmp_register_write(0x00, 0x6C, 0x02, regs);                 //D_0_108 inv_set_accel_calibration
}

// dmp_apply_endian_accel - set byte order of accelerometer values?
void AP_InertialSensor_MPU6000::dmp_apply_endian_accel()
{
    uint8_t regs[4];
    regs[0]=0x00;
    regs[1]=0x00;
    regs[2]=0x40;
    regs[3]=0x00;
    dmp_register_write(0x01, 0xEC, 0x04, regs);       //D_1_236 inv_apply_endian_accel
}

// dmp_set_mpu_sensors - to configure for SIX_AXIS output
void AP_InertialSensor_MPU6000::dmp_set_mpu_sensors()
{
    uint8_t regs[6];
    regs[0]=0x0C;
    regs[1]=0xC9;
    regs[2]=0x2C;
    regs[3]=0x97;
    regs[4]=0x97;
    regs[5]=0x97;
    dmp_register_write(0x03, 0x7F, 0x06, regs);       //FCFG_2  inv_set_mpu_sensors(INV_SIX_AXIS_GYRO_ACCEL);
}

// dmp_set_bias_from_no_motion - turn on bias from no motion
void AP_InertialSensor_MPU6000::dmp_set_bias_from_no_motion()
{
    uint8_t regs[4];
    regs[0]=0x0D;
    regs[1]=0x35;
    regs[2]=0x5D;
    dmp_register_write(0x04, 0x02, 0x03, regs);       //CFG_MOTION_BIAS inv_turn_on_bias_from_no_motion
    regs[0]=0x87;
    regs[1]=0x2D;
    regs[2]=0x35;
    regs[3]=0x3D;
    dmp_register_write(0x04, 0x09, 0x04, regs);       //FCFG_5 inv_set_bias_update( INV_BIAS_FROM_NO_MOTION );
}

// dmp_set_bias_none - turn off internal bias correction (we will use this and we handle the gyro bias correction externally)
void AP_InertialSensor_MPU6000::dmp_set_bias_none()
{
    uint8_t regs[4];
    regs[0]=0x98;
    regs[1]=0x98;
    regs[2]=0x98;
    dmp_register_write(0x04, 0x02, 0x03, regs);       //CFG_MOTION_BIAS inv_turn_off_bias_from_no_motion
    regs[0]=0x87;
    regs[1]=0x2D;
    regs[2]=0x35;
    regs[3]=0x3D;
    dmp_register_write(0x04, 0x09, 0x04, regs);       //FCFG_5 inv_set_bias_update( INV_BIAS_FROM_NO_MOTION );
}

// dmp_set_fifo_interrupt
void AP_InertialSensor_MPU6000::dmp_set_fifo_interrupt()
{
    uint8_t regs[1];
    regs[0]=0xFE;
    dmp_register_write(0x07, 0x86, 0x01, regs);       //CFG_6 inv_set_fifo_interupt
}

// dmp_send_quaternion - send quaternion data to FIFO
void AP_InertialSensor_MPU6000::dmp_send_quaternion()
{
    uint8_t regs[5];
    regs[0]=0xF1;
    regs[1]=0x20;
    regs[2]=0x28;
    regs[3]=0x30;
    regs[4]=0x38;
    dmp_register_write(0x07, 0x41, 0x05, regs);       //CFG_8 inv_send_quaternion
    regs[0]=0x30;
    dmp_register_write(0x07, 0x7E, 0x01, regs);       //CFG_16 inv_set_footer
}

// dmp_send_gyro - send gyro data to FIFO
void AP_InertialSensor_MPU6000::dmp_send_gyro()
{
    uint8_t regs[4];
    regs[0]=0xF1;
    regs[1]=0x28;
    regs[2]=0x30;
    regs[3]=0x38;
    dmp_register_write(0x07, 0x47, 0x04, regs);       //CFG_9 inv_send_gyro
}

// dmp_send_accel - send accel data to FIFO
void AP_InertialSensor_MPU6000::dmp_send_accel()
{
    uint8_t regs[54];
    regs[0]=0xF1;
    regs[1]=0x28;
    regs[2]=0x30;
    regs[3]=0x38;
    dmp_register_write(0x07, 0x6C, 0x04, regs);       //CFG_12 inv_send_accel
}

// This functions defines the rate at wich attitude data is send to FIFO
// Rate: 0 => SAMPLE_RATE(ex:200Hz), 1=> SAMPLE_RATE/2 (ex:100Hz), 2=> SAMPLE_RATE/3 (ex:66Hz)
// rate constant definitions in MPU6000.h
void AP_InertialSensor_MPU6000::dmp_set_fifo_rate(uint8_t rate)
{
    uint8_t regs[2];
    regs[0]=0x00;
    regs[1]=rate;
    dmp_register_write(0x02, 0x16, 0x02, regs);                 //D_0_22 inv_set_fifo_rate
}

// This function defines the weight of the accel on the sensor fusion
// default value is 0x80
// The official invensense name is inv_key_0_96 (??)
void AP_InertialSensor_MPU6000::dmp_set_sensor_fusion_accel_gain(uint8_t gain)
{
    //inv_key_0_96
    register_write(MPUREG_BANK_SEL,0x00);
    register_write(MPUREG_MEM_START_ADDR, 0x60);
    _spi->cs_assert();
    _spi->transfer(MPUREG_MEM_R_W);
    _spi->transfer(0x00);
    _spi->transfer(gain);      // Original : 0x80    To test: 0x40,  0x20 (too less)
    _spi->transfer(0x00);
    _spi->transfer(0x00);
    _spi->cs_release();
}

// Load initial memory values into DMP memory banks
void AP_InertialSensor_MPU6000::dmp_load_mem()
{

    for(int i = 0; i < 7; i++) {
        register_write(MPUREG_BANK_SEL,i);              //MPUREG_BANK_SEL
        for(uint8_t j = 0; j < 16; j++) {
            uint8_t start_addy = j * 0x10;
            register_write(MPUREG_MEM_START_ADDR,start_addy);
            _spi->cs_assert();
            _spi->transfer(MPUREG_MEM_R_W);
            for(int k = 0; k < 16; k++) {
                uint8_t byteToSend = pgm_read_byte((const prog_char *)&(dmpMem[i][j][k]));
                _spi->transfer((uint8_t) byteToSend);
            }
            _spi->cs_release();
        }
    }

    register_write(MPUREG_BANK_SEL,7);          //MPUREG_BANK_SEL
    for(uint8_t j = 0; j < 8; j++) {
        uint8_t start_addy = j * 0x10;
        register_write(MPUREG_MEM_START_ADDR,start_addy);
        _spi->cs_assert();
        _spi->transfer(MPUREG_MEM_R_W);
        for(int k = 0; k < 16; k++) {
            uint8_t byteToSend = pgm_read_byte((const prog_char *)&(dmpMem[7][j][k]));
            _spi->transfer((uint8_t) byteToSend);
        }
        _spi->cs_release();
    }

    register_write(MPUREG_MEM_START_ADDR,0x80);
    _spi->cs_assert();
    _spi->transfer(MPUREG_MEM_R_W);
    for(int k = 0; k < 9; k++) {
        uint8_t byteToSend = pgm_read_byte((const prog_char *)&(dmpMem[7][8][k]));
        _spi->transfer((uint8_t) byteToSend);
    }
    _spi->cs_release();
}

// ========= DMP MEMORY ================================

const uint8_t dmpMem[8][16][16] PROGMEM = {
    {
        {
            0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00
        }
        ,
        {
            0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01
        }
        ,
        {
            0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01
        }
        ,
        {
            0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00
        }
        ,
        {
            0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00
        }
        ,
        {
            0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82
        }
        ,
        {
            0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00
        }
        ,
        {
            0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0
        }
        ,
        {
            0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC
        }
        ,
        {
            0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4
        }
        ,
        {
            0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10
        }
    }
    ,
    {
        {
            0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8
        }
        ,
        {
            0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7
        }
        ,
        {
            0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C
        }
        ,
        {
            0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C
        }
        ,
        {
            0x80, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0
        }
    }
    ,
    {
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
    }
    ,
    {
        {
            0xD8, 0xDC, 0xBA, 0xA2, 0xF1, 0xDE, 0xB2, 0xB8, 0xB4, 0xA8, 0x81, 0x91, 0xF7, 0x4A, 0x90, 0x7F
        }
        ,
        {
            0x91, 0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA, 0xA0, 0x80, 0xF2, 0xCE, 0x81, 0xF3, 0xC2
        }
        ,
        {
            0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80, 0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF
        }
        ,
        {
            0xDF, 0xDF, 0xF2, 0xA7, 0xC3, 0xCB, 0xC5, 0xB6, 0xF0, 0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C
        }
        ,
        {
            0x95, 0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1, 0x83, 0x92, 0x2D, 0x55, 0x7D, 0xD8, 0xB1
        }
        ,
        {
            0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3, 0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01
        }
        ,
        {
            0xB0, 0x98, 0x87, 0xD9, 0x43, 0xD8, 0x86, 0xC9, 0x88, 0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80
        }
        ,
        {
            0xF1, 0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0xC5, 0xCD, 0xC7, 0xA9, 0x0C
        }
        ,
        {
            0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89, 0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80
        }
        ,
        {
            0xAC, 0xDE, 0xF2, 0xCA, 0xF1, 0xB2, 0x8C, 0x02, 0xA9, 0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E
        }
        ,
        {
            0xB8, 0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A, 0xA8, 0x96, 0x36, 0x56, 0x76, 0xF1, 0xB9
        }
        ,
        {
            0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24
        }
        ,
        {
            0x08, 0x44, 0x10, 0x64, 0x18, 0xF1, 0xA3, 0x29, 0x55, 0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xAF, 0xF0
        }
        ,
        {
            0x00, 0x28, 0x50, 0xF1, 0xA3, 0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xD9, 0xFA, 0xA3, 0x86
        }
        ,
        {
            0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5, 0xC7, 0xB2, 0x8C, 0xC1
        }
        ,
        {
            0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86
        }
    }
    ,
    {
        {
            0xB4, 0x98, 0x0D, 0x35, 0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA
        }
        ,
        {
            0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A, 0xB8, 0xAA, 0x87, 0x2C
        }
        ,
        {
            0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3, 0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8
        }
        ,
        {
            0xA9, 0xB4, 0x99, 0x83, 0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3
        }
        ,
        {
            0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97, 0x83, 0xA8, 0x11, 0x84
        }
        ,
        {
            0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5
        }
        ,
        {
            0x29, 0x55, 0x7D, 0xA5, 0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3
        }
        ,
        {
            0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0x97, 0x82, 0xA8, 0xF1
        }
        ,
        {
            0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5
        }
        ,
        {
            0x94, 0x01, 0xD9, 0xA3, 0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D
        }
        ,
        {
            0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9
        }
        ,
        {
            0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D
        }
        ,
        {
            0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9
        }
        ,
        {
            0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0xA8, 0x8A, 0x9A
        }
        ,
        {
            0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98, 0xA8, 0xD9, 0x08, 0xD8
        }
        ,
        {
            0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA, 0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87
        }
    }
    ,
    {
        {
            0x34, 0xB5, 0xB9, 0x94, 0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8
        }
        ,
        {
            0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9, 0x61, 0xD8, 0x6C, 0x68
        }
        ,
        {
            0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9, 0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D
        }
        ,
        {
            0xA3, 0x83, 0x1A, 0x3E, 0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94
        }
        ,
        {
            0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x35, 0xDA
        }
        ,
        {
            0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8, 0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56
        }
        ,
        {
            0xA5, 0x81, 0x00, 0x0C, 0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9
        }
        ,
        {
            0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8, 0xA3, 0x29, 0x83, 0xDA
        }
        ,
        {
            0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA, 0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0xA8, 0x8A
        }
        ,
        {
            0x9A, 0xF5, 0x20, 0xAA, 0xDA, 0xDF, 0xD8, 0xA8, 0x40, 0xAA, 0xD0, 0xDA, 0xDE, 0xD8, 0xA8, 0x60
        }
        ,
        {
            0xAA, 0xDA, 0xD0, 0xDF, 0xD8, 0xF1, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97
        }
        ,
        {
            0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40, 0xB8, 0xB0, 0xB4, 0xA8, 0x8C, 0x9C, 0xF0, 0x04
        }
        ,
        {
            0x28, 0x51, 0x79, 0x1D, 0x30, 0x14, 0x38, 0xB2, 0x82, 0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78
        }
        ,
        {
            0x78, 0x9B, 0xF1, 0x1A, 0xB0, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79
        }
        ,
        {
            0x8A, 0x24, 0x70, 0x59, 0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68
        }
        ,
        {
            0x8A, 0x64, 0x48, 0x31, 0x8B, 0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71, 0x58, 0x44, 0x68
        }
    }
    ,
    {
        {
            0x11, 0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04
        }
        ,
        {
            0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66
        }
        ,
        {
            0xF0, 0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31
        }
        ,
        {
            0xA9, 0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60
        }
        ,
        {
            0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76
        }
        ,
        {
            0x7E, 0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A, 0x6E, 0x8A, 0x56
        }
        ,
        {
            0x8A, 0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E, 0x9D, 0xB8, 0xAD
        }
        ,
        {
            0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0x81, 0x91
        }
        ,
        {
            0xAC, 0x38, 0xAD, 0x3A, 0xB5, 0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8
        }
        ,
        {
            0x6D, 0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51, 0xD9, 0x04, 0xAE
        }
        ,
        {
            0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD, 0x00, 0x8D, 0xAE, 0x19, 0x81, 0xAD, 0xD9
        }
        ,
        {
            0x01, 0xD8, 0xF2, 0xAE, 0xDA, 0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9, 0xAD, 0xAD, 0xAD
        }
        ,
        {
            0xAD, 0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76, 0xF3, 0xAC, 0x2E
        }
        ,
        {
            0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28, 0x28, 0x28, 0x9C, 0xAC, 0x30, 0x18, 0xA8
        }
        ,
        {
            0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28, 0x34, 0x3C, 0x9C, 0x24, 0xF2, 0xB0, 0x89
        }
        ,
        {
            0xAC, 0x91, 0x2C, 0x4C, 0x6C, 0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9, 0xD8, 0xD8, 0x79
        }
    }
    ,
    {
        {
            0xD9, 0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D, 0xD9, 0x28, 0xD8
        }
        ,
        {
            0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x83, 0x93, 0x35, 0x3D, 0x80, 0x25, 0xDA
        }
        ,
        {
            0xD8, 0xD8, 0x85, 0x69, 0xDA, 0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34, 0x3C, 0xF3, 0xAB
        }
        ,
        {
            0x8B, 0xF8, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xFA, 0xB0, 0x87, 0x9C, 0xB9, 0xA3
        }
        ,
        {
            0xDD, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x95, 0xF1, 0xA3, 0xA3, 0xA3, 0x9D, 0xF1, 0xA3, 0xA3, 0xA3
        }
        ,
        {
            0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3
        }
        ,
        {
            0xA3, 0xB2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0xA3, 0xA3, 0xA3
        }
        ,
        {
            0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3, 0xF3, 0x9B, 0xA3, 0xA3, 0xDC
        }
        ,
        {
            0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xD8, 0xD8, 0xFF
        }
    }
};
