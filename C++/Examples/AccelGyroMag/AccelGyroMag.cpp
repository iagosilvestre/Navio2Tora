/*
Provided to you by Emlid Ltd (c) 2015.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Read accelerometer, gyroscope and magnetometer values from
inertial measurement unit: MPU9250 or LSM9DS1 over SPI on Raspberry Pi + Navio.

Navio's onboard sensors are connected to the SPI bus on Raspberry Pi
and can be read through /dev/spidev0.1 (MPU9250), /dev/spidev0.3 (acc/gyro LSM9DS1)
and /dev/spidev0.2 (mag LSM9DS1).

To run this example navigate to the directory containing it and run following commands:
make
./AccelGyroMag -i [sensor name]
Sensors names: mpu is MPU9250, lsm is LSM9DS1.
For print help:
./AccelGyroMag -h
*/
#include <cstdio>
#include <stdarg.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include "Util.h"


#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <memory>

#define G_SI 9.80665
#define PI   3.14159
#define DEVICE_ACC_GYRO     "/dev/spidev0.3"
#define DEVICE_MAGNETOMETER "/dev/spidev0.2"

#define READ_FLAG     0x80
#define MULTIPLE_READ 0x40
#define SCRIPT_PATH "../../../check_apm.sh"

// MPU9250 registers
#define MPUREG_XG_OFFS_TC          0x00
#define MPUREG_YG_OFFS_TC          0x01
#define MPUREG_ZG_OFFS_TC          0x02
#define MPUREG_X_FINE_GAIN         0x03
#define MPUREG_Y_FINE_GAIN         0x04
#define MPUREG_Z_FINE_GAIN         0x05
#define MPUREG_XA_OFFS_H           0x06
#define MPUREG_XA_OFFS_L           0x07
#define MPUREG_YA_OFFS_H           0x08
#define MPUREG_YA_OFFS_L           0x09
#define MPUREG_ZA_OFFS_H           0x0A
#define MPUREG_ZA_OFFS_L           0x0B
#define MPUREG_PRODUCT_ID          0x0C
#define MPUREG_SELF_TEST_X         0x0D
#define MPUREG_SELF_TEST_Y         0x0E
#define MPUREG_SELF_TEST_Z         0x0F
#define MPUREG_SELF_TEST_A         0x10
#define MPUREG_XG_OFFS_USRH        0x13
#define MPUREG_XG_OFFS_USRL        0x14
#define MPUREG_YG_OFFS_USRH        0x15
#define MPUREG_YG_OFFS_USRL        0x16
#define MPUREG_ZG_OFFS_USRH        0x17
#define MPUREG_ZG_OFFS_USRL        0x18
#define MPUREG_SMPLRT_DIV          0x19
#define MPUREG_CONFIG              0x1A
#define MPUREG_GYRO_CONFIG         0x1B
#define MPUREG_ACCEL_CONFIG        0x1C
#define MPUREG_ACCEL_CONFIG_2      0x1D
#define MPUREG_LP_ACCEL_ODR        0x1E
#define MPUREG_MOT_THR             0x1F
#define MPUREG_FIFO_EN             0x23
#define MPUREG_I2C_MST_CTRL        0x24
#define MPUREG_I2C_SLV0_ADDR       0x25
#define MPUREG_I2C_SLV0_REG        0x26
#define MPUREG_I2C_SLV0_CTRL       0x27
#define MPUREG_I2C_SLV1_ADDR       0x28
#define MPUREG_I2C_SLV1_REG        0x29
#define MPUREG_I2C_SLV1_CTRL       0x2A
#define MPUREG_I2C_SLV2_ADDR       0x2B
#define MPUREG_I2C_SLV2_REG        0x2C
#define MPUREG_I2C_SLV2_CTRL       0x2D
#define MPUREG_I2C_SLV3_ADDR       0x2E
#define MPUREG_I2C_SLV3_REG        0x2F
#define MPUREG_I2C_SLV3_CTRL       0x30
#define MPUREG_I2C_SLV4_ADDR       0x31
#define MPUREG_I2C_SLV4_REG        0x32
#define MPUREG_I2C_SLV4_DO         0x33
#define MPUREG_I2C_SLV4_CTRL       0x34
#define MPUREG_I2C_SLV4_DI         0x35
#define MPUREG_I2C_MST_STATUS      0x36
#define MPUREG_INT_PIN_CFG         0x37
#define MPUREG_INT_ENABLE          0x38
#define MPUREG_ACCEL_XOUT_H        0x3B
#define MPUREG_ACCEL_XOUT_L        0x3C
#define MPUREG_ACCEL_YOUT_H        0x3D
#define MPUREG_ACCEL_YOUT_L        0x3E
#define MPUREG_ACCEL_ZOUT_H        0x3F
#define MPUREG_ACCEL_ZOUT_L        0x40
#define MPUREG_TEMP_OUT_H          0x41
#define MPUREG_TEMP_OUT_L          0x42
#define MPUREG_GYRO_XOUT_H         0x43
#define MPUREG_GYRO_XOUT_L         0x44
#define MPUREG_GYRO_YOUT_H         0x45
#define MPUREG_GYRO_YOUT_L         0x46
#define MPUREG_GYRO_ZOUT_H         0x47
#define MPUREG_GYRO_ZOUT_L         0x48
#define MPUREG_EXT_SENS_DATA_00    0x49
#define MPUREG_EXT_SENS_DATA_01    0x4A
#define MPUREG_EXT_SENS_DATA_02    0x4B
#define MPUREG_EXT_SENS_DATA_03    0x4C
#define MPUREG_EXT_SENS_DATA_04    0x4D
#define MPUREG_EXT_SENS_DATA_05    0x4E
#define MPUREG_EXT_SENS_DATA_06    0x4F
#define MPUREG_EXT_SENS_DATA_07    0x50
#define MPUREG_EXT_SENS_DATA_08    0x51
#define MPUREG_EXT_SENS_DATA_09    0x52
#define MPUREG_EXT_SENS_DATA_10    0x53
#define MPUREG_EXT_SENS_DATA_11    0x54
#define MPUREG_EXT_SENS_DATA_12    0x55
#define MPUREG_EXT_SENS_DATA_13    0x56
#define MPUREG_EXT_SENS_DATA_14    0x57
#define MPUREG_EXT_SENS_DATA_15    0x58
#define MPUREG_EXT_SENS_DATA_16    0x59
#define MPUREG_EXT_SENS_DATA_17    0x5A
#define MPUREG_EXT_SENS_DATA_18    0x5B
#define MPUREG_EXT_SENS_DATA_19    0x5C
#define MPUREG_EXT_SENS_DATA_20    0x5D
#define MPUREG_EXT_SENS_DATA_21    0x5E
#define MPUREG_EXT_SENS_DATA_22    0x5F
#define MPUREG_EXT_SENS_DATA_23    0x60
#define MPUREG_I2C_SLV0_DO         0x63
#define MPUREG_I2C_SLV1_DO         0x64
#define MPUREG_I2C_SLV2_DO         0x65
#define MPUREG_I2C_SLV3_DO         0x66
#define MPUREG_I2C_MST_DELAY_CTRL  0x67
#define MPUREG_SIGNAL_PATH_RESET   0x68
#define MPUREG_MOT_DETECT_CTRL     0x69
#define MPUREG_USER_CTRL           0x6A
#define MPUREG_PWR_MGMT_1          0x6B
#define MPUREG_PWR_MGMT_2          0x6C
#define MPUREG_BANK_SEL            0x6D
#define MPUREG_MEM_START_ADDR      0x6E
#define MPUREG_MEM_R_W             0x6F
#define MPUREG_DMP_CFG_1           0x70
#define MPUREG_DMP_CFG_2           0x71
#define MPUREG_FIFO_COUNTH         0x72
#define MPUREG_FIFO_COUNTL         0x73
#define MPUREG_FIFO_R_W            0x74
#define MPUREG_WHOAMI              0x75
#define MPUREG_XA_OFFSET_H         0x77
#define MPUREG_XA_OFFSET_L         0x78
#define MPUREG_YA_OFFSET_H         0x7A
#define MPUREG_YA_OFFSET_L         0x7B
#define MPUREG_ZA_OFFSET_H         0x7D
#define MPUREG_ZA_OFFSET_L         0x7E

/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */

#define AK8963_I2C_ADDR             0x0c  // should return 0x18
#define AK8963_Device_ID            0x48

// Read-only Reg
#define AK8963_WIA                  0x00
#define AK8963_INFO                 0x01
#define AK8963_ST1                  0x02
#define AK8963_HXL                  0x03
#define AK8963_HXH                  0x04
#define AK8963_HYL                  0x05
#define AK8963_HYH                  0x06
#define AK8963_HZL                  0x07
#define AK8963_HZH                  0x08
#define AK8963_ST2                  0x09

// Write/Read Reg
#define AK8963_CNTL1                0x0A
#define AK8963_CNTL2                0x0B
#define AK8963_ASTC                 0x0C
#define AK8963_TS1                  0x0D
#define AK8963_TS2                  0x0E
#define AK8963_I2CDIS               0x0F

// Read-only Reg ( ROM )
#define AK8963_ASAX                 0x10
#define AK8963_ASAY                 0x11
#define AK8963_ASAZ                 0x12

// Configuration bits MPU9250
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10

#define READ_FLAG                   0x80

/* ---- Sensitivity --------------------------------------------------------- */

#define MPU9250A_2g       ((float)0.000061035156f) // 0.000061035156 g/LSB
#define MPU9250A_4g       ((float)0.000122070312f) // 0.000122070312 g/LSB
#define MPU9250A_8g       ((float)0.000244140625f) // 0.000244140625 g/LSB
#define MPU9250A_16g      ((float)0.000488281250f) // 0.000488281250 g/LSB

#define MPU9250G_250dps   ((float)0.007633587786f) // 0.007633587786 dps/LSB
#define MPU9250G_500dps   ((float)0.015267175572f) // 0.015267175572 dps/LSB
#define MPU9250G_1000dps  ((float)0.030487804878f) // 0.030487804878 dps/LSB
#define MPU9250G_2000dps  ((float)0.060975609756f) // 0.060975609756 dps/LSB

#define MPU9250M_4800uT   ((float)0.6f)            // 0.6 uT/LSB

#define MPU9250T_85degC   ((float)0.002995177763f) // 0.002995177763 degC/LSB

#define Magnetometer_Sensitivity_Scale_Factor ((float)0.15f)
// who am I values
#define WHO_AM_I_ACC_GYRO           0x68
#define WHO_AM_I_MAG                0x3D

// Accelerometer and Gyroscope registers
#define LSM9DS1XG_ACT_THS           0x04
#define LSM9DS1XG_ACT_DUR           0x05
#define LSM9DS1XG_INT_GEN_CFG_XL    0x06
#define LSM9DS1XG_INT_GEN_THS_X_XL  0x07
#define LSM9DS1XG_INT_GEN_THS_Y_XL  0x08
#define LSM9DS1XG_INT_GEN_THS_Z_XL  0x09
#define LSM9DS1XG_INT_GEN_DUR_XL    0x0A
#define LSM9DS1XG_REFERENCE_G       0x0B
#define LSM9DS1XG_INT1_CTRL         0x0C
#define LSM9DS1XG_INT2_CTRL         0x0D
#define LSM9DS1XG_WHO_AM_I          0x0F  // should return 0x68
#define LSM9DS1XG_CTRL_REG1_G       0x10
#define LSM9DS1XG_CTRL_REG2_G       0x11
#define LSM9DS1XG_CTRL_REG3_G       0x12
#define LSM9DS1XG_ORIENT_CFG_G      0x13
#define LSM9DS1XG_INT_GEN_SRC_G     0x14
#define LSM9DS1XG_OUT_TEMP_L        0x15
#define LSM9DS1XG_OUT_TEMP_H        0x16
#define LSM9DS1XG_STATUS_REG        0x17
#define LSM9DS1XG_OUT_X_L_G         0x18
#define LSM9DS1XG_OUT_X_H_G         0x19
#define LSM9DS1XG_OUT_Y_L_G         0x1A
#define LSM9DS1XG_OUT_Y_H_G         0x1B
#define LSM9DS1XG_OUT_Z_L_G         0x1C
#define LSM9DS1XG_OUT_Z_H_G         0x1D
#define LSM9DS1XG_CTRL_REG4         0x1E
#define LSM9DS1XG_CTRL_REG5_XL      0x1F
#define LSM9DS1XG_CTRL_REG6_XL      0x20
#define LSM9DS1XG_CTRL_REG7_XL      0x21
#define LSM9DS1XG_CTRL_REG8         0x22
#define LSM9DS1XG_CTRL_REG9         0x23
#define LSM9DS1XG_CTRL_REG10        0x24
#define LSM9DS1XG_INT_GEN_SRC_XL    0x26
#define LSM9DS1XG_OUT_X_L_XL        0x28
#define LSM9DS1XG_OUT_X_H_XL        0x29
#define LSM9DS1XG_OUT_Y_L_XL        0x2A
#define LSM9DS1XG_OUT_Y_H_XL        0x2B
#define LSM9DS1XG_OUT_Z_L_XL        0x2C
#define LSM9DS1XG_OUT_Z_H_XL        0x2D
#define LSM9DS1XG_FIFO_CTRL         0x2E
#define LSM9DS1XG_FIFO_SRC          0x2F
#define LSM9DS1XG_INT_GEN_CFG_G     0x30
#define LSM9DS1XG_INT_GEN_THS_XH_G  0x31
#define LSM9DS1XG_INT_GEN_THS_XL_G  0x32
#define LSM9DS1XG_INT_GEN_THS_YH_G  0x33
#define LSM9DS1XG_INT_GEN_THS_YL_G  0x34
#define LSM9DS1XG_INT_GEN_THS_ZH_G  0x35
#define LSM9DS1XG_INT_GEN_THS_ZL_G  0x36
#define LSM9DS1XG_INT_GEN_DUR_G     0x37

// Magnetometer registers
#define LSM9DS1M_OFFSET_X_REG_L_M   0x05
#define LSM9DS1M_OFFSET_X_REG_H_M   0x06
#define LSM9DS1M_OFFSET_Y_REG_L_M   0x07
#define LSM9DS1M_OFFSET_Y_REG_H_M   0x08
#define LSM9DS1M_OFFSET_Z_REG_L_M   0x09
#define LSM9DS1M_OFFSET_Z_REG_H_M   0x0A
#define LSM9DS1M_WHO_AM_I           0x0F  // should return 0x3D
#define LSM9DS1M_CTRL_REG1_M        0x20
#define LSM9DS1M_CTRL_REG2_M        0x21
#define LSM9DS1M_CTRL_REG3_M        0x22
#define LSM9DS1M_CTRL_REG4_M        0x23
#define LSM9DS1M_CTRL_REG5_M        0x24
#define LSM9DS1M_STATUS_REG_M       0x27
#define LSM9DS1M_OUT_X_L_M          0x28
#define LSM9DS1M_OUT_X_H_M          0x29
#define LSM9DS1M_OUT_Y_L_M          0x2A
#define LSM9DS1M_OUT_Y_H_M          0x2B
#define LSM9DS1M_OUT_Z_L_M          0x2C
#define LSM9DS1M_OUT_Z_H_M          0x2D
#define LSM9DS1M_INT_CFG_M          0x30
#define LSM9DS1M_INT_SRC_M          0x31
#define LSM9DS1M_INT_THS_L_M        0x32
#define LSM9DS1M_INT_THS_H_M        0x33

// Configuration bits Accelerometer and Gyroscope
#define BITS_XEN_G                  0x08
#define BITS_YEN_G                  0x10
#define BITS_ZEN_G                  0x20
#define BITS_XEN_XL                 0x08
#define BITS_YEN_XL                 0x10
#define BITS_ZEN_XL                 0x20
#define BITS_ODR_G_14900mHZ         0x20
#define BITS_ODR_G_59500mHZ         0x40
#define BITS_ODR_G_119HZ            0x60
#define BITS_ODR_G_238HZ            0x80
#define BITS_ODR_G_476HZ            0xA0
#define BITS_ODR_G_952HZ            0xC0
#define BITS_ODR_XL_10HZ            0x20
#define BITS_ODR_XL_50HZ            0x40
#define BITS_ODR_XL_119HZ           0x60
#define BITS_ODR_XL_238HZ           0x80
#define BITS_ODR_XL_476HZ           0xA0
#define BITS_ODR_XL_952HZ           0xC0
#define BITS_FS_G_MASK              0xE3
#define BITS_FS_G_245DPS            0x00
#define BITS_FS_G_500DPS            0x08
#define BITS_FS_G_2000DPS           0x18
#define BITS_FS_XL_MASK             0xE7
#define BITS_FS_XL_2G               0x00
#define BITS_FS_XL_4G               0x10
#define BITS_FS_XL_8G               0x18
#define BITS_FS_XL_16G              0x08

// Configuration bits Magnetometer
#define BITS_TEMP_COMP              0x80
#define BITS_OM_LOW                 0x00
#define BITS_OM_MEDIUM              0x20
#define BITS_OM_HIGH                0x40
#define BITS_OM_ULTRA_HIGH          0x60
#define BITS_ODR_M_625mHZ           0x00
#define BITS_ODR_M_1250mHZ          0x04
#define BITS_ODR_M_250mHZ           0x08
#define BITS_ODR_M_5HZ              0x0C
#define BITS_ODR_M_10HZ             0x10
#define BITS_ODR_M_20HZ             0x14
#define BITS_ODR_M_40HZ             0x18
#define BITS_ODR_M_80HZ             0x1C
#define BITS_FS_M_MASK              0x0C
#define BITS_FS_M_4Gs               0x00
#define BITS_FS_M_8Gs               0x20
#define BITS_FS_M_12Gs              0x40
#define BITS_FS_M_16Gs              0x60
#define BITS_MD_CONTINUOUS          0x00
#define BITS_MD_SINGLE              0x01
#define BITS_MD_POWERDOWN           0x02
#define BITS_OMZ_LOW                0x00
#define BITS_OMZ_MEDIUM             0x04
#define BITS_OMZ_HIGH               0x08
#define BITS_OMZ_ULTRA_HIGH         0x0C
#define ARRAY_SIZE(a) sizeof(a) / sizeof(a[0])
#define NAVIO2 3
#define NAVIO 1


std::unique_ptr <InertialSensor> get_inertial_sensor( std::string sensor_name)
{
    if (sensor_name == "mpu") {
        printf("Selected: MPU9250\n");
        auto ptr = std::unique_ptr <InertialSensor>{ new MPU9250() };
        return ptr;
    }
    else if (sensor_name == "lsm") {
        printf("Selected: LSM9DS1\n");
        auto ptr = std::unique_ptr <InertialSensor>{ new LSM9DS1() };
        return ptr;
    }
    else {
        return NULL;
    }
}

void print_help()
{
    printf("Possible parameters:\nSensor selection: -i [sensor name]\n");
    printf("Sensors names: mpu is MPU9250, lsm is LSM9DS1\nFor help: -h\n");
}

std::string get_sensor_name(int argc, char *argv[])
{
    if (get_navio_version() == NAVIO2) {

        if (argc < 2) {
            printf("Enter parameter\n");
            print_help();
            return std::string();
        }

        // prevent the error message
        opterr = 0;
        int parameter;

        while ((parameter = getopt(argc, argv, "i:h")) != -1) {
            switch (parameter) {
            case 'i': if (!strcmp(optarg,"mpu") ) return "mpu";
                            else return "lsm";
            case 'h': print_help(); return "-1";
            case '?': printf("Wrong parameter.\n");
                      print_help();
                      return std::string();
            }
        }

    } else { //sensor on NAVIO+

        return "mpu";
    }

}
//=============================================================================
int main(int argc, char *argv[])
{

    if (check_apm()) {
        return 1;
    }

    auto sensor_name = get_sensor_name(argc, argv);
    if (sensor_name.empty())
        return EXIT_FAILURE;

    auto sensor = get_inertial_sensor(sensor_name);

    if (!sensor) {
        printf("Wrong sensor name. Select: mpu or lsm\n");
        return EXIT_FAILURE;
    }

    if (!sensor->probe()) {
        printf("Sensor not enabled\n");
        return EXIT_FAILURE;
    }
    sensor->initialize();

    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
//-------------------------------------------------------------------------

    while(1) {
        sensor->update();
        sensor->read_accelerometer(&ax, &ay, &az);
        sensor->read_gyroscope(&gx, &gy, &gz);
        sensor->read_magnetometer(&mx, &my, &mz);
        printf("Acc: %+7.3f %+7.3f %+7.3f  ", ax, ay, az);
        printf("Gyr: %+8.3f %+8.3f %+8.3f  ", gx, gy, gz);
        printf("Mag: %+7.3f %+7.3f %+7.3f\n", mx, my, mz);

       usleep(500000);
    }
}


MPU9250::MPU9250()
{
}

/*-----------------------------------------------------------------------------------------------
                                    REGISTER READ & WRITE
usage: use these methods to read and write MPU9250 registers over SPI
-----------------------------------------------------------------------------------------------*/

unsigned int MPU9250::WriteReg(uint8_t WriteAddr, uint8_t WriteData)
{
    unsigned char tx[2] = {WriteAddr, WriteData};
    unsigned char rx[2] = {0};

    SPIdev::transfer("/dev/spidev0.1", tx, rx, 2);

    return rx[1];
}

//-----------------------------------------------------------------------------------------------

unsigned int MPU9250::ReadReg(uint8_t ReadAddr)
{
    return WriteReg(ReadAddr | READ_FLAG, 0x00);
}

//-----------------------------------------------------------------------------------------------

void MPU9250::ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes)
{
    unsigned int  i = 0;

    unsigned char tx[255] = {0};
    unsigned char rx[255] = {0};

    tx[0] = ReadAddr | READ_FLAG;

    SPIdev::transfer("/dev/spidev0.1", tx, rx, Bytes + 1);

    for(i=0; i<Bytes; i++)
        ReadBuf[i] = rx[i + 1];

    usleep(50);
}

/*-----------------------------------------------------------------------------------------------
                                TEST CONNECTION
usage: call this function to know if SPI and MPU9250 are working correctly.
returns true if mpu9250 answers
-----------------------------------------------------------------------------------------------*/

bool MPU9250::probe()
{
    uint8_t responseXG, responseM;

    responseXG = ReadReg(MPUREG_WHOAMI | READ_FLAG);

    WriteReg(MPUREG_USER_CTRL, 0x20);  // I2C Master mode
    WriteReg(MPUREG_I2C_MST_CTRL, 0x0D); // I2C configuration multi-master  IIC 400KHz
    WriteReg(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_WIA); //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer
    usleep(10000);
    responseM = ReadReg(MPUREG_EXT_SENS_DATA_00);

    if (responseXG == 0x71 && responseM == 0x48)
        return true;
    else
        return false;
}

/*-----------------------------------------------------------------------------------------------
                                    INITIALIZATION
usage: call this function at startup for initialize settings of sensor
low pass filter suitable values are:
BITS_DLPF_CFG_256HZ_NOLPF2
BITS_DLPF_CFG_188HZ
BITS_DLPF_CFG_98HZ
BITS_DLPF_CFG_42HZ
BITS_DLPF_CFG_20HZ
BITS_DLPF_CFG_10HZ
BITS_DLPF_CFG_5HZ
BITS_DLPF_CFG_2100HZ_NOLPF
returns 1 if an error occurred
-----------------------------------------------------------------------------------------------*/

#define MPU_InitRegNum 16

bool MPU9250::initialize()
{
    uint8_t i = 0;
    uint8_t MPU_Init_Data[MPU_InitRegNum][2] = {
        //{0x80, MPUREG_PWR_MGMT_1},     // Reset Device - Disabled because it seems to corrupt initialisation of AK8963
        {0x01, MPUREG_PWR_MGMT_1},     // Clock Source
        {0x00, MPUREG_PWR_MGMT_2},     // Enable Acc & Gyro
        {0x00, MPUREG_CONFIG},         // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
        {0x18, MPUREG_GYRO_CONFIG},    // +-2000dps
        {3<<3, MPUREG_ACCEL_CONFIG},   // +-16G
        {0x08, MPUREG_ACCEL_CONFIG_2}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
        {0x30, MPUREG_INT_PIN_CFG},    //
        //{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
        //{0x20, MPUREG_USER_CTRL},      // Enable AUX
        {0x20, MPUREG_USER_CTRL},       // I2C Master mode
        {0x0D, MPUREG_I2C_MST_CTRL}, //  I2C configuration multi-master  IIC 400KHz

        {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  //Set the I2C slave addres of AK8963 and set for write.
        //{0x09, MPUREG_I2C_SLV4_CTRL},
        //{0x81, MPUREG_I2C_MST_DELAY_CTRL}, //Enable I2C delay

        {AK8963_CNTL2, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
        {0x01, MPUREG_I2C_SLV0_DO}, // Reset AK8963
        {0x81, MPUREG_I2C_SLV0_CTRL},  //Enable I2C and set 1 byte

        {AK8963_CNTL1, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
        {0x12, MPUREG_I2C_SLV0_DO}, // Register value to continuous measurement in 16bit
        {0x81, MPUREG_I2C_SLV0_CTRL}  //Enable I2C and set 1 byte

    };

    set_acc_scale(BITS_FS_16G);
    set_gyro_scale(BITS_FS_2000DPS);

    for(i=0; i<MPU_InitRegNum; i++) {
        WriteReg(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
        usleep(100000);  //I2C must slow down the write speed, otherwise it won't work
    }

    calib_mag();
    return 0;
}
/*-----------------------------------------------------------------------------------------------
                                ACCELEROMETER SCALE
usage: call this function at startup, after initialization, to set the right range for the
accelerometers. Suitable ranges are:
BITS_FS_2G
BITS_FS_4G
BITS_FS_8G
BITS_FS_16G
returns the range set (2,4,8 or 16)
-----------------------------------------------------------------------------------------------*/

unsigned int MPU9250::set_acc_scale(int scale)
{
    unsigned int temp_scale;
    WriteReg(MPUREG_ACCEL_CONFIG, scale);

    switch (scale) {
    case BITS_FS_2G:
        acc_divider = 16384;
        break;
    case BITS_FS_4G:
        acc_divider = 8192;
        break;
    case BITS_FS_8G:
        acc_divider = 4096;
        break;
    case BITS_FS_16G:
        acc_divider = 2048;
        break;
    }
    temp_scale=WriteReg(MPUREG_ACCEL_CONFIG | READ_FLAG, 0x00);

    switch (temp_scale) {
    case BITS_FS_2G:
        temp_scale = 2;
        break;
    case BITS_FS_4G:
        temp_scale = 4;
        break;
    case BITS_FS_8G:
        temp_scale = 8;
        break;
    case BITS_FS_16G:
        temp_scale = 16;
        break;
    }
    return temp_scale;
}


/*-----------------------------------------------------------------------------------------------
                                GYROSCOPE SCALE
usage: call this function at startup, after initialization, to set the right range for the
gyroscopes. Suitable ranges are:
BITS_FS_250DPS
BITS_FS_500DPS
BITS_FS_1000DPS
BITS_FS_2000DPS
returns the range set (250,500,1000 or 2000)
-----------------------------------------------------------------------------------------------*/

unsigned int MPU9250::set_gyro_scale(int scale)
{
    unsigned int temp_scale;
    WriteReg(MPUREG_GYRO_CONFIG, scale);
    switch (scale){
    case BITS_FS_250DPS:
        gyro_divider = 131;
        break;
    case BITS_FS_500DPS:
        gyro_divider = 65.5;
        break;
    case BITS_FS_1000DPS:
        gyro_divider = 32.8;
        break;
    case BITS_FS_2000DPS:
        gyro_divider = 16.4;
        break;
    }

    temp_scale=WriteReg(MPUREG_GYRO_CONFIG | READ_FLAG, 0x00);
    switch (temp_scale){
    case BITS_FS_250DPS:
        temp_scale = 250;
        break;
    case BITS_FS_500DPS:
        temp_scale = 500;
        break;
    case BITS_FS_1000DPS:
        temp_scale = 1000;
        break;
    case BITS_FS_2000DPS:
        temp_scale = 2000;
        break;
    }
    return temp_scale;
}

/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER CALIBRATION
usage: call this function to read accelerometer data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
returns Factory Trim value
-----------------------------------------------------------------------------------------------*/

void MPU9250::calib_acc()
{
    uint8_t response[4];
    int temp_scale;
    // read current acc scale
    temp_scale = WriteReg(MPUREG_ACCEL_CONFIG | READ_FLAG, 0x00);
    set_acc_scale(BITS_FS_8G);
    //ENABLE SELF TEST need modify
    //temp_scale=WriteReg(MPUREG_ACCEL_CONFIG, 0x80>>axis);

    ReadRegs(MPUREG_SELF_TEST_X,response,4);
    calib_data[0] = ((response[0]&11100000) >> 3) | ((response[3]&00110000) >> 4);
    calib_data[1] = ((response[1]&11100000) >> 3) | ((response[3]&00001100) >> 2);
    calib_data[2] = ((response[2]&11100000) >> 3) | ((response[3]&00000011));

    set_acc_scale(temp_scale);
}

//-----------------------------------------------------------------------------------------------

void MPU9250::calib_mag()
{
    uint8_t response[3];
    float data;
    int i;

    WriteReg(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_ASAX); //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x83); //Read 3 bytes from the magnetometer

    //WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);    //Enable I2C and set bytes
    usleep(10000);
    //response[0]=WriteReg(MPUREG_EXT_SENS_DATA_01 | READ_FLAG, 0x00);    //Read I2C
    ReadRegs(MPUREG_EXT_SENS_DATA_00, response, 3);

    //response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);    //Read I2C
    for(i=0; i<3; i++) {
        data = response[i];
        magnetometer_ASA[i] = ((data - 128) / 256 + 1) * Magnetometer_Sensitivity_Scale_Factor;
    }
}


//-----------------------------------------------------------------------------------------------

void MPU9250::update()
{
    uint8_t response[21];
    int16_t bit_data[3];
    int i;

    //Send I2C command at first
    WriteReg(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 7 bytes from the magnetometer
    //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.

    ReadRegs(MPUREG_ACCEL_XOUT_H, response, 21);

    //Get accelerometer value
    for(i=0; i<3; i++) {
        bit_data[i] = ((int16_t)response[i*2] << 8) | response[i*2+1];
    }
    _ax = G_SI * bit_data[0] / acc_divider;
    _ay = G_SI * bit_data[1] / acc_divider;
    _az = G_SI * bit_data[2] / acc_divider;

    //Get temperature
    bit_data[0] = ((int16_t)response[i*2] << 8) | response[i*2+1];
    temperature = ((bit_data[0] - 21) / 333.87) + 21;

    //Get gyroscope value
    for(i=4; i<7; i++) {
        bit_data[i-4] = ((int16_t)response[i*2] << 8) | response[i*2+1];
    }
    _gx = (PI / 180) * bit_data[0] / gyro_divider;
    _gy = (PI / 180) * bit_data[1] / gyro_divider;
    _gz = (PI / 180) * bit_data[2] / gyro_divider;

    //Get Magnetometer value
    for(i=7; i<10; i++) {
        bit_data[i-7] = ((int16_t)response[i*2+1] << 8) | response[i*2];
    }
    _mx = bit_data[0] * magnetometer_ASA[0];
    _my = bit_data[1] * magnetometer_ASA[1];
    _mz = bit_data[2] * magnetometer_ASA[2];
}


LSM9DS1::LSM9DS1()
{
}

/*-----------------------------------------------------------------------------------------------
                                    REGISTER READ & WRITE
usage: use these methods to read and write LSM9DS1 registers over SPI
-----------------------------------------------------------------------------------------------*/

unsigned int LSM9DS1::WriteReg(const char *dev, uint8_t WriteAddr, uint8_t WriteData )
{
    unsigned char tx[2] = {WriteAddr, WriteData};
    unsigned char rx[2] = {0};
    SPIdev::transfer(dev, tx, rx, 2);
    return rx[1];
}

unsigned int  LSM9DS1::ReadReg(const char *dev, uint8_t ReadAddr)
{
    return WriteReg(dev, ReadAddr | READ_FLAG, 0x00);
}

void LSM9DS1::ReadRegs(const char *dev, uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes )
{
    unsigned char tx[255] = {0};
    unsigned char rx[255] = {0};

    tx[0] = ReadAddr | READ_FLAG;
    if (!strcmp(dev, DEVICE_MAGNETOMETER)) tx[0] |= MULTIPLE_READ;

    SPIdev::transfer(dev, tx, rx, Bytes + 1);

    for (uint i = 0; i < Bytes; i++)
        ReadBuf[i] = rx[i + 1];

    usleep(50);
}

/*-----------------------------------------------------------------------------------------------
                                TEST CONNECTION
usage: call this function to know if SPI and LSM9DS1 are working correctly.
returns true if Accel/Gyro and Magnetometer answers
-----------------------------------------------------------------------------------------------*/

bool LSM9DS1::probe()
{
    uint8_t responseXG,responseM;
    responseXG = ReadReg(DEVICE_ACC_GYRO, LSM9DS1XG_WHO_AM_I);
    responseM = ReadReg(DEVICE_MAGNETOMETER, LSM9DS1M_WHO_AM_I);
    if (responseXG == WHO_AM_I_ACC_GYRO && responseM == WHO_AM_I_MAG)
        return true;
    else
        return false;
}

bool LSM9DS1::initialize()
{
    //--------Accelerometer and Gyroscope---------
    // enable the 3-axes of the gyroscope
    WriteReg(DEVICE_ACC_GYRO, LSM9DS1XG_CTRL_REG4, BITS_XEN_G |
                                                BITS_YEN_G |
                                                BITS_ZEN_G);
    // configure the gyroscope
    WriteReg(DEVICE_ACC_GYRO, LSM9DS1XG_CTRL_REG1_G, BITS_ODR_G_952HZ |
                                                  BITS_FS_G_2000DPS);
    usleep(200);

    // enable the three axes of the accelerometer
    WriteReg(DEVICE_ACC_GYRO, LSM9DS1XG_CTRL_REG5_XL, BITS_XEN_XL |
                                                   BITS_YEN_XL |
                                                   BITS_ZEN_XL);
    // configure the accelerometer-specify bandwidth selection with Abw
    WriteReg(DEVICE_ACC_GYRO, LSM9DS1XG_CTRL_REG6_XL, BITS_ODR_XL_952HZ |
                                                   BITS_FS_XL_16G);
    usleep(200);

    //------------Magnetometer----------------
    WriteReg(DEVICE_MAGNETOMETER, LSM9DS1M_CTRL_REG1_M, BITS_TEMP_COMP |
                                            BITS_OM_HIGH |
                                            BITS_ODR_M_80HZ);
    WriteReg(DEVICE_MAGNETOMETER, LSM9DS1M_CTRL_REG2_M, BITS_FS_M_16Gs);
    // continuous conversion mode
    WriteReg(DEVICE_MAGNETOMETER, LSM9DS1M_CTRL_REG3_M, BITS_MD_CONTINUOUS);
    WriteReg(DEVICE_MAGNETOMETER, LSM9DS1M_CTRL_REG4_M, BITS_OMZ_HIGH);
    WriteReg(DEVICE_MAGNETOMETER, LSM9DS1M_CTRL_REG5_M, 0x00 );
    usleep(200);

    set_gyro_scale(BITS_FS_G_2000DPS);
    set_acc_scale(BITS_FS_XL_16G);
    set_mag_scale(BITS_FS_M_16Gs);
    return true;
}

void LSM9DS1::update()
{
    uint8_t response[6];
    int16_t bit_data[3];

    // Read temperature
    ReadRegs(DEVICE_ACC_GYRO, LSM9DS1XG_OUT_TEMP_L, &response[0], 2);
    temperature = (float)(((int16_t)response[1] << 8) | response[0]) / 256. + 25.;

    // Read accelerometer
    ReadRegs(DEVICE_ACC_GYRO, LSM9DS1XG_OUT_X_L_XL, &response[0], 6);
    for (int i=0; i<3; i++) {
        bit_data[i] = ((int16_t)response[2*i+1] << 8) | response[2*i] ;
    }
    _ax = G_SI * ((float)bit_data[0] * acc_scale);
    _ay = G_SI * ((float)bit_data[1] * acc_scale);
    _az = G_SI * ((float)bit_data[2] * acc_scale);

    // Read gyroscope
    ReadRegs(DEVICE_ACC_GYRO, LSM9DS1XG_OUT_X_L_G, &response[0], 6);
    for (int i=0; i<3; i++) {
        bit_data[i] = ((int16_t)response[2*i+1] << 8) | response[2*i] ;
    }
    _gx = (PI / 180) * ((float)bit_data[0] * gyro_scale);
    _gy = (PI / 180) * ((float)bit_data[1] * gyro_scale);
    _gz = (PI / 180) * ((float)bit_data[2] * gyro_scale);

    // Read magnetometer
    ReadRegs(DEVICE_MAGNETOMETER, LSM9DS1M_OUT_X_L_M, &response[0], 6);
    for (int i=0; i<3; i++) {
        bit_data[i] = ((int16_t)response[2*i+1] << 8) | response[2*i] ;
    }
    _mx = 100.0 * ((float)bit_data[0] * mag_scale);
    _my = 100.0 * ((float)bit_data[1] * mag_scale);
    _mz = 100.0 * ((float)bit_data[2] * mag_scale);

    // Change rotation of LSM9DS1 like in MPU-9250
    rotate();
}

void LSM9DS1::rotate()
{
    float replacement_acc, replacement_gyro;

    replacement_acc = _ax;
    _ax = -_ay;
    _ay = -replacement_acc;

    replacement_gyro = _gx;
    _gx = -_gy;
    _gy = -replacement_gyro;

    _my = -_my;
    _mz = -_mz;
}

void LSM9DS1::set_gyro_scale(int scale)
{
    uint8_t reg;
    reg = BITS_FS_G_MASK & ReadReg(DEVICE_ACC_GYRO, LSM9DS1XG_CTRL_REG1_G);
    WriteReg(DEVICE_ACC_GYRO, LSM9DS1XG_CTRL_REG1_G,reg | scale);
    switch (scale) {
    case BITS_FS_G_245DPS:
        gyro_scale = 0.00875;
        break;
    case BITS_FS_G_500DPS:
        gyro_scale = 0.0175;
        break;
    case BITS_FS_G_2000DPS:
        gyro_scale = 0.07;
        break;
    }
}

void LSM9DS1::set_acc_scale(int scale)
{
    uint8_t reg;
    reg = BITS_FS_XL_MASK & ReadReg(DEVICE_ACC_GYRO, LSM9DS1XG_CTRL_REG6_XL);
    WriteReg(DEVICE_ACC_GYRO, LSM9DS1XG_CTRL_REG6_XL, reg | scale);
    switch (scale) {
    case BITS_FS_XL_2G:
        acc_scale = 0.000061;
        break;
    case BITS_FS_XL_4G:
        acc_scale = 0.000122;
        break;
    case BITS_FS_XL_8G:
        acc_scale = 0.000244;
        break;
    case BITS_FS_XL_16G:
        acc_scale = 0.000732;
        break;
    }
}

void LSM9DS1::set_mag_scale(int scale)
{
    uint8_t reg;
    reg = BITS_FS_M_MASK & ReadReg(DEVICE_MAGNETOMETER, LSM9DS1M_CTRL_REG2_M);
    WriteReg(DEVICE_MAGNETOMETER, LSM9DS1M_CTRL_REG2_M, reg | scale);
    switch (scale) {
    case BITS_FS_M_4Gs:
        mag_scale = 0.00014;
        break;
    case BITS_FS_M_8Gs:
        mag_scale = 0.00029;
        break;
    case BITS_FS_M_12Gs:
        mag_scale = 0.00043;
        break;
    case BITS_FS_M_16Gs:
        mag_scale = 0.00058;
        break;
    }
}

int write_file(const char *path, const char *fmt, ...)
{
    errno = 0;

    int fd = ::open(path, O_WRONLY | O_CLOEXEC);
    if (fd == -1) {
        return -errno;
    }

    va_list args;
    va_start(args, fmt);

    int ret = ::vdprintf(fd, fmt, args);
    int errno_bkp = errno;
    ::close(fd);

    va_end(args);

    if (ret < 1) {
        return -errno_bkp;
    }

    return ret;
}

int read_file(const char *path, const char *fmt, ...)
{
    errno = 0;

    FILE *file = ::fopen(path, "re");
    if (!file)
        return -errno;

    va_list args;
    va_start(args, fmt);

    int ret = ::vfscanf(file, fmt, args);
    int errno_bkp = errno;
    ::fclose(file);

    va_end(args);

    if (ret < 1)
        return -errno_bkp;

    return ret;
}

bool check_apm()
{
    int ret =  system("ps -AT | grep -c ap-timer > /dev/null");

    if (WEXITSTATUS(ret) <= 0) {
        fprintf(stderr, "APM is running. Can't launch the example\n");
        return true;
    }

    return false;
}

int get_navio_version()
{
    int version;
    read_file("/sys/firmware/devicetree/base/hat/product_id", "%x",&version);
    return version;
}
