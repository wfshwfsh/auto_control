#ifndef BNO055_UART_H
#define BNO055_UART_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>
#include <string.h>


/* BNO055 Address A and B */
#define BNO055_ADDRESS_A (0x28)
#define BNO055_ADDRESS_B (0x29)

/* BNO055 ID */
#define BNO055_ID (0xA0)

/* Offsets registers */
#define NUM_BNO055_OFFSET_REGISTERS (22)

/* Maximum number of attempts to resend a cmd*/
#define MAX_CMD_SEND_ATTEMPTS 10

/* A structure to represent data vector */
typedef struct
{
    union
    {
        // Array of length 3
        float v[3];
        // Structure with x, y, and z components
        struct
        {
            float x;
            float y;
            float z;
        };
        // Structure with heading, pitch, and roll orientation
        struct
        {
            // TODO: These will need to be rearranged
            float heading;
            float roll;
            float pitch;
        };
    };
} bno055_vector_t;

/* A structure to represent revisions */
typedef struct
{
    uint8_t accel_rev;
    uint8_t mag_rev;
    uint8_t gyro_rev;
    uint16_t sw_rev;
    uint8_t bl_rev;
} bno055_rev_info_t;

/* Calibration offsets */
typedef struct
{
    uint16_t accel_offset_x;
    uint16_t accel_offset_y;
    uint16_t accel_offset_z;

    uint16_t mag_offset_x;
    uint16_t mag_offset_y;
    uint16_t mag_offset_z;

    uint16_t gyro_offset_x;
    uint16_t gyro_offset_y;
    uint16_t gyro_offset_z;

    uint16_t accel_radius;

    uint16_t mag_radius;
} bno055_offsets_t;

/* Operation mode setting */
typedef enum
{
    OPERATION_MODE_CONFIG = 0x00,
    OPERATION_MODE_ACCONLY = 0x01,
    OPERATION_MODE_MAGONLY = 0x02,
    OPERATION_MODE_GYRONLY = 0x03,
    OPERATION_MODE_ACCMAG = 0x04,
    OPERATION_MODE_ACCGYRO = 0x05,
    OPERATION_MODE_MAGGYRO = 0x06,
    OPERATION_MODE_AMG = 0x07,
    OPERATION_MODE_IMUPLUS = 0x08,
    OPERATION_MODE_COMPASS = 0x09,
    OPERATION_MODE_M4G = 0x0A,
    OPERATION_MODE_NDOF_FMC_OFF = 0x0B,
    OPERATION_MODE_NDOF = 0x0C
} bno055_opmode_t;


enum Ascale {  // ACC Full Scale
	AFS_2G = 0,
	AFS_4G,
	AFS_8G,
	AFS_18G
};
enum Abw { // ACC Bandwidth
	ABW_7_81Hz = 0,
	ABW_15_63Hz,
	ABW_31_25Hz,
	ABW_62_5Hz,
	ABW_125Hz,    
	ABW_250Hz,
	ABW_500Hz,     
	ABW_1000Hz,    //0x07
};

enum APwrMode { // ACC Pwr Mode //POWER_MODE_NORMAL???
	NormalA = 0,  
	SuspendA,
	LowPower1A,
	StandbyA,        
	LowPower2A,
	DeepSuspendA
};

enum Gscale {  // gyro full scale
	GFS_2000DPS = 0,
	GFS_1000DPS,
	GFS_500DPS,
	GFS_250DPS,
	GFS_125DPS    // 0x04
};
enum GPwrMode { // GYR Pwr Mode
	NormalG = 0,
	FastPowerUpG,
	DeepSuspendedG,
	SuspendG,
	AdvancedPowerSaveG
};
enum Gbw { // gyro bandwidth
	GBW_523Hz = 0,
	GBW_230Hz,
	GBW_116Hz,
	GBW_47Hz,
	GBW_23Hz,
	GBW_12Hz,
	GBW_64Hz,
	GBW_32Hz
};

//??
enum OPRMode {  // BNO-55 operation modes
	CONFIGMODE = 0x00,
	// Sensor Mode
	ACCONLY,
	MAGONLY,
	GYROONLY,
	ACCMAG,
	ACCGYRO,
	MAGGYRO,
	AMG,            // 0x07
	// Fusion Mode
	IMU,
	COMPASS,
	M4G,
	NDOF_FMC_OFF,
	NDOF            // 0x0C OPERATION_MODE_NDOF
};

enum PWRMode {
	Normalpwr = 0,   
	Lowpower,       
	Suspendpwr       
};
enum Modr {         // magnetometer output data rate  
	MODR_2Hz = 0,     
	MODR_6Hz,
	MODR_8Hz,
	MODR_10Hz,  
	MODR_15Hz,
	MODR_20Hz,
	MODR_25Hz, 
	MODR_30Hz 
};
enum MOpMode { // MAG Op Mode
	LowPower = 0,
	Regular,
	EnhancedRegular,
	HighAccuracy
};
enum MPwrMode { // MAG power mode
	Normal = 0,   
	Sleep,     
	Suspend,
	ForceMode  
};


/* BNO055 power settings */
typedef enum
{
    POWER_MODE_NORMAL = 0x00,
    POWER_MODE_LOWPOWER = 0x01,
    POWER_MODE_SUSPEND = 0x02
} bno055_powermode_t;

/* Remap settings */
typedef enum
{
    REMAP_CONFIG_P0 = 0x21,
    REMAP_CONFIG_P1 = 0x24, // default
    REMAP_CONFIG_P2 = 0x24,
    REMAP_CONFIG_P3 = 0x21,
    REMAP_CONFIG_P4 = 0x24,
    REMAP_CONFIG_P5 = 0x21,
    REMAP_CONFIG_P6 = 0x21,
    REMAP_CONFIG_P7 = 0x24
} bno055_axis_remap_config_t;

/* Remap Signs */
typedef enum
{
    REMAP_SIGN_P0 = 0x04,
    REMAP_SIGN_P1 = 0x00, // default
    REMAP_SIGN_P2 = 0x06,
    REMAP_SIGN_P3 = 0x02,
    REMAP_SIGN_P4 = 0x03,
    REMAP_SIGN_P5 = 0x01,
    REMAP_SIGN_P6 = 0x07,
    REMAP_SIGN_P7 = 0x05
} bno055_axis_remap_sign_t;

/* BNO055 Registers */
typedef enum
{
	/* PAGE0 REGISTER DEFINITION START*/
    BNO055_CHIP_ID_ADDR = 0x00,
    BNO055_ACCEL_REV_ID_ADDR = 0x01,
    BNO055_MAG_REV_ID_ADDR = 0x02,
    BNO055_GYRO_REV_ID_ADDR = 0x03,
    BNO055_SW_REV_ID_LSB_ADDR = 0x04,
    BNO055_SW_REV_ID_MSB_ADDR = 0x05,
    BNO055_BL_REV_ID_ADDR = 0X06,
	
	
    /* Page id register definition */
    BNO055_PAGE_ID_ADDR = 0x07,

    /* Accel data register */
    BNO055_ACCEL_DATA_X_LSB_ADDR = 0x08,
    BNO055_ACCEL_DATA_X_MSB_ADDR = 0x09,
    BNO055_ACCEL_DATA_Y_LSB_ADDR = 0x0A,
    BNO055_ACCEL_DATA_Y_MSB_ADDR = 0x0B,
    BNO055_ACCEL_DATA_Z_LSB_ADDR = 0x0C,
    BNO055_ACCEL_DATA_Z_MSB_ADDR = 0x0D,

    /* Mag data register */
    BNO055_MAG_DATA_X_LSB_ADDR = 0x0E,
    BNO055_MAG_DATA_X_MSB_ADDR = 0x0F,
    BNO055_MAG_DATA_Y_LSB_ADDR = 0x10,
    BNO055_MAG_DATA_Y_MSB_ADDR = 0x11,
    BNO055_MAG_DATA_Z_LSB_ADDR = 0x12,
    BNO055_MAG_DATA_Z_MSB_ADDR = 0x13,

    /* Gyro data registers */
    BNO055_GYRO_DATA_X_LSB_ADDR = 0x14,
    BNO055_GYRO_DATA_X_MSB_ADDR = 0x15,
    BNO055_GYRO_DATA_Y_LSB_ADDR = 0x16,
    BNO055_GYRO_DATA_Y_MSB_ADDR = 0x17,
    BNO055_GYRO_DATA_Z_LSB_ADDR = 0x18,
    BNO055_GYRO_DATA_Z_MSB_ADDR = 0x19,

    /* Euler data registers */
    BNO055_EULER_H_LSB_ADDR = 0x1A,
    BNO055_EULER_H_MSB_ADDR = 0x1B,
    BNO055_EULER_R_LSB_ADDR = 0x1C,
    BNO055_EULER_R_MSB_ADDR = 0x1D,
    BNO055_EULER_P_LSB_ADDR = 0x1E,
    BNO055_EULER_P_MSB_ADDR = 0x1F,

    /* Quaternion data registers */
    BNO055_QUATERNION_DATA_W_LSB_ADDR = 0x20,
    BNO055_QUATERNION_DATA_W_MSB_ADDR = 0x21,
    BNO055_QUATERNION_DATA_X_LSB_ADDR = 0x22,
    BNO055_QUATERNION_DATA_X_MSB_ADDR = 0x23,
    BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0x24,
    BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0x25,
    BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0x26,
    BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0x27,

    /* Linear acceleration data registers */
    BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0x28,
    BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR = 0x29,
    BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0x2A,
    BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0x2B,
    BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0x2C,
    BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0x2D,

    /* Gravity data registers */
    BNO055_GRAVITY_DATA_X_LSB_ADDR = 0x2E,
    BNO055_GRAVITY_DATA_X_MSB_ADDR = 0x2F,
    BNO055_GRAVITY_DATA_Y_LSB_ADDR = 0x30,
    BNO055_GRAVITY_DATA_Y_MSB_ADDR = 0x31,
    BNO055_GRAVITY_DATA_Z_LSB_ADDR = 0x32,
    BNO055_GRAVITY_DATA_Z_MSB_ADDR = 0x33,

    /* Temperature data register */
    BNO055_TEMP_ADDR = 0x34,

    /* Status registers */
    BNO055_CALIB_STAT_ADDR = 0x35,
    BNO055_SELFTEST_RESULT_ADDR = 0x36,
    BNO055_INTR_STAT_ADDR = 0x37,

    BNO055_SYS_CLK_STAT_ADDR = 0x38,
    BNO055_SYS_STAT_ADDR = 0x39,
    BNO055_SYS_ERR_ADDR = 0x3A,

    /* Unit selection register */
    BNO055_UNIT_SEL_ADDR = 0x3B,

    /* Mode registers */
    BNO055_OPR_MODE_ADDR = 0x3D,
    BNO055_PWR_MODE_ADDR = 0x3E,

    BNO055_SYS_TRIGGER_ADDR = 0x3F,
    BNO055_TEMP_SOURCE_ADDR = 0x40,

    /* Axis remap registers */
    BNO055_AXIS_MAP_CONFIG_ADDR = 0x41,
    BNO055_AXIS_MAP_SIGN_ADDR = 0x42,

    /* SIC registers */
    BNO055_SIC_MATRIX_0_LSB_ADDR = 0x43,
    BNO055_SIC_MATRIX_0_MSB_ADDR = 0x44,
    BNO055_SIC_MATRIX_1_LSB_ADDR = 0x45,
    BNO055_SIC_MATRIX_1_MSB_ADDR = 0x46,
    BNO055_SIC_MATRIX_2_LSB_ADDR = 0x47,
    BNO055_SIC_MATRIX_2_MSB_ADDR = 0x48,
    BNO055_SIC_MATRIX_3_LSB_ADDR = 0x49,
    BNO055_SIC_MATRIX_3_MSB_ADDR = 0x4A,
    BNO055_SIC_MATRIX_4_LSB_ADDR = 0x4B,
    BNO055_SIC_MATRIX_4_MSB_ADDR = 0x4C,
    BNO055_SIC_MATRIX_5_LSB_ADDR = 0x4D,
    BNO055_SIC_MATRIX_5_MSB_ADDR = 0x4E,
    BNO055_SIC_MATRIX_6_LSB_ADDR = 0x4F,
    BNO055_SIC_MATRIX_6_MSB_ADDR = 0x50,
    BNO055_SIC_MATRIX_7_LSB_ADDR = 0x51,
    BNO055_SIC_MATRIX_7_MSB_ADDR = 0x52,
    BNO055_SIC_MATRIX_8_LSB_ADDR = 0x53,
    BNO055_SIC_MATRIX_8_MSB_ADDR = 0x54,

    /* Accelerometer Offset registers */
    ACCEL_OFFSET_X_LSB_ADDR = 0x55,
    ACCEL_OFFSET_X_MSB_ADDR = 0x56,
    ACCEL_OFFSET_Y_LSB_ADDR = 0x57,
    ACCEL_OFFSET_Y_MSB_ADDR = 0x58,
    ACCEL_OFFSET_Z_LSB_ADDR = 0x59,
    ACCEL_OFFSET_Z_MSB_ADDR = 0x5A,

    /* Magnetometer Offset registers */
    MAG_OFFSET_X_LSB_ADDR = 0x5B,
    MAG_OFFSET_X_MSB_ADDR = 0x5C,
    MAG_OFFSET_Y_LSB_ADDR = 0x5D,
    MAG_OFFSET_Y_MSB_ADDR = 0x5E,
    MAG_OFFSET_Z_LSB_ADDR = 0x5F,
    MAG_OFFSET_Z_MSB_ADDR = 0x60,

    /* Gyroscope Offset register s*/
    GYRO_OFFSET_X_LSB_ADDR = 0x61,
    GYRO_OFFSET_X_MSB_ADDR = 0x62,
    GYRO_OFFSET_Y_LSB_ADDR = 0x63,
    GYRO_OFFSET_Y_MSB_ADDR = 0x64,
    GYRO_OFFSET_Z_LSB_ADDR = 0x65,
    GYRO_OFFSET_Z_MSB_ADDR = 0x66,

    /* Radius registers */
    ACCEL_RADIUS_LSB_ADDR = 0x67,
    ACCEL_RADIUS_MSB_ADDR = 0x68,
    MAG_RADIUS_LSB_ADDR = 0x69,
    MAG_RADIUS_MSB_ADDR = 0x6A
} bno055_page0_register_t;

typedef enum
{
    /* PAGE1 REGISTER DEFINITION START*/
	BNO055_PAGE_ID          = 0x07,
	BNO055_ACC_CONFIG       = 0x08,
	BNO055_MAG_CONFIG       = 0x09,
	BNO055_GYRO_CONFIG_0    = 0x0A,
	BNO055_GYRO_CONFIG_1    = 0x0B,
	BNO055_ACC_SLEEP_CONFIG = 0x0C,
	BNO055_GYR_SLEEP_CONFIG = 0x0D,
	BNO055_INT_MSK          = 0x0F,
	BNO055_INT_EN           = 0x10,
	BNO055_ACC_AM_THRES     = 0x11,
	BNO055_ACC_INT_SETTINGS = 0x12,
	BNO055_ACC_HG_DURATION  = 0x13,
	BNO055_ACC_HG_THRESH    = 0x14,
	BNO055_ACC_NM_THRESH    = 0x15,
	BNO055_ACC_NM_SET       = 0x16,
	BNO055_GYR_INT_SETTINGS = 0x17,
	BNO055_GYR_HR_X_SET     = 0x18,
	BNO055_GYR_DUR_X        = 0x19,
	BNO055_GYR_HR_Y_SET     = 0x1A,
	BNO055_GYR_DUR_Y        = 0x1B,
	BNO055_GYR_HR_Z_SET     = 0x1C,
	BNO055_GYR_DUR_Z        = 0x1D,
	BNO055_GYR_AM_THRESH    = 0x1E,
	BNO055_GYR_AM_SET       = 0x1F,
	
} bno055_page1_register_t;
#endif


