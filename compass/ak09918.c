#include "ak09918.h"
#include <stdio.h>
#include "uart_debug.h" 
#include "FreeRTOS.h"
#include "i2cm_support.h"
#include <math.h>
//#include "common.h"

const char* ak09918_err_string(int err);
int ak09918_get_mode(AK09918_dev_t* dev);
static int ak09918_set_mode(AK09918_dev_t *dev, AK09918_mode_type_t mode);
int ak09918_reset(AK09918_dev_t* dev);
int ak09918_is_ready(AK09918_dev_t* dev);
int ak09918_is_skip(AK09918_dev_t* dev);
int ak09918_read(AK09918_dev_t* dev);
int ak09918_read_raw(AK09918_dev_t* dev, int32_t *rx, int32_t *ry, int32_t *rz);
int ak09918_self_test(AK09918_dev_t* dev);
int ak09918_init(AK09918_dev_t *dev);
int ak09918_latchdata(AK09918_dev_t *dev);
uint16_t ak09918_getdirection(axis_dblock_t *GDB, AK09918_dev_t *CDB);

static const char* ak09918_err_strings[] = {
	"AK09918_ERR_OK: OK",
	"AK09918_ERR_DOR: Data skipped(read too slowly)",
	"AK09918_ERR_NOT_RDY: Not ready(check too quickly)",
	"AK09918_ERR_TIMEOUT: Timeout",
	"AK09918_ERR_SELFTEST: Self test failed",
	"AK09918_ERR_OVERFLOW: Sensor overflow",
	"AK09918_ERR_WR_FAIL: Fail to write",
	"AK09918_ERR_RD_FAIL: Fail to read",
	"Unknown Error",
};
#if 0
void* ak09918_alloc(void) {
    return malloc(sizeof(ak09918_t));
}

int ak09918_free(rpi_ak09918_t* dev) {
    free(dev);
    return 0;
}
#endif
const char* ak09918_err_string(int err)
{
    const int size = sizeof ak09918_err_strings /
                      sizeof ak09918_err_strings[0];
    err = (err < 0)? -err: err;
    if (err >= size) {
        err = size - 1;
    }
    return ak09918_err_strings[err];
}

int ak09918_get_mode(AK09918_dev_t* dev)
{
    return dev->mode;
}

int ak09918_reset(AK09918_dev_t* dev)
{
    int ret;
    uint8_t buf;
    buf = AK09918_SRST_BIT;
    ret = I2C_1_WriteBytes(dev->addr, AK09918_CNTL3, 1, &buf);
    if (ret < 0) {
        return AK09918_ERR_WRITE_FAILED;
    }
    return AK09918_ERR_OK;
}
    
int ak09918_is_ready(AK09918_dev_t* dev)
{
    uint8_t buf;
    uint8_t ret;
    ret = I2C_1_ReadBytes(dev->addr, AK09918_ST1, 1, &buf);
    if (ret != 0) {
        DebugPrintf("Error: AK09918_ERR_READ_FAILED = %02x\n", buf);
        return AK09918_ERR_READ_FAILED;
    }
    if (buf & AK09918_DRDY_BIT){
        return AK09918_ERR_OK;
    }
    //DebugPrintf("Info: AK09918_ERR_OK = %02x\n", buf);
    return AK09918_ERR_NOT_RDY;
}

int ak09918_is_skip(AK09918_dev_t* dev)
{
    uint8_t buf;
    uint8_t ret;
    ret = I2C_1_ReadBytes(dev->addr, AK09918_ST1, 1, &buf);
    if (ret != 0) {
        return AK09918_ERR_READ_FAILED;
    }
    if (buf & AK09918_DOR_BIT){
        DebugPrintf("Error: AK09918_ERR_DOR = %02x\n", buf);
        return AK09918_ERR_DOR;
    }
    return AK09918_ERR_OK;
}

int ak09918_read(AK09918_dev_t* dev)
{
	#define COEF    0.15f
    
	int32_t ret;
	int32_t tx, ty, tz;
    
	ret = ak09918_read_raw(dev, &tx, &ty, &tz);
    
    /* X, Y axis is upside down on the board.*/
	dev->value.axis[0] = -tx * COEF;
	dev->value.axis[1] = -ty * COEF;
	dev->value.axis[2] = tz * COEF;
	return ret;
}

int ak09918_read_raw(AK09918_dev_t* dev, int32_t *rx, int32_t *ry, int32_t *rz)
{
	uint8_t buf[8];
    int count = 0;
    
    memset(buf,0,sizeof(buf));
	if (dev->mode == AK09918_NORMAL) {
		for (;;) {
			I2C_1_ReadBytes(dev->addr, AK09918_CNTL2, 1, &buf[0]);
			if (buf[0] == 0) {
				break;
			}
			if (count++ >= 15) {
                DebugPrintf("Error: AK09918_ERR_TIMEOUT = %d\n", count);
				return AK09918_ERR_TIMEOUT;
			}
            vTaskDelay(1);
		}
	}
#if 0
	rt = rpi_i2c_read(dev->addr, AK09918_HXL, buf, sizeof buf);
	if (rt < 0) {
		return AK09918_ERR_READ_FAILED;
	}
#else
    memset(buf, 0, sizeof(buf));
    I2C_1_ReadBytes(dev->addr, AK09918_HXL, sizeof(buf), buf);
#endif
	if (buf[7] & AK09918_HOFL_BIT) {
        DebugPrintf("Error: AK09918_ERR_OVERFLOW = %d\n", buf[7]);
		return AK09918_ERR_OVERFLOW;
	}
	*rx = *(int16_t*)&buf[0];
	*ry = *(int16_t*)&buf[2];
	*rz = *(int16_t*)&buf[4];
	return AK09918_ERR_OK;
}

// 1. Set Power-down mode. (MODE[4:0] bits = â€?0000â€?
// 2. Set Self-test mode.  (MODE[4:0] bits = â€?0000â€?
// 3. Check Data Ready or not by polling DRDY bit of ST1 register.
// 4. When Data Ready, proceed to the next step.
//    Read measurement data. (HXL to HZH)
int ak09918_self_test(AK09918_dev_t* dev)
{
	uint8_t buf[8];
	int32_t x, y, z;
	int ret;
    AK09918_mode_type_t l_mode;
    
	l_mode = dev->mode;

	/* skip the buffer data or else self testing will failed */
	ak09918_read_raw(dev, &x, &y, &z);

	ret = ak09918_set_mode(dev, AK09918_POWER_DOWN);
	Cy_SysLib_Delay(10);
	ret = ak09918_set_mode(dev, AK09918_SELF_TEST);

	for (;;) {
		if ((ret = ak09918_is_ready(dev)) == AK09918_ERR_OK) {
			break;
		}
		if (ret == AK09918_ERR_READ_FAILED) {
            DebugPrintf("Error: AK09918_ERR_READ_FAILED = %d\n", ret);
			ak09918_set_mode(dev, l_mode);
			return ret;
		}
		Cy_SysLib_Delay(1);
	}
#if 0
	ret = I2C_1_ReadBytes(dev->addr, AK09918_HXL, buf, sizeof(buf));
	if (ret < 0) {
		ak09918_set_mode(dev, l_mode);
		return AK09918_ERR_READ_FAILED;
	}
#else
    I2C_1_ReadBytes(dev->addr, AK09918_HXL, sizeof(buf), buf);
#endif
	x = *(int16_t*)&buf[0];
	y = *(int16_t*)&buf[2];
	z = *(int16_t*)&buf[4];

	if (( -200 <= x && x <= 200 ) && ( -200 <= y && y <= 200 ) && (-1000 <= z && z <= -150)
	) {
		ret = AK09918_ERR_OK;
	} else {
		ret = AK09918_ERR_SELFTEST_FAILED;
	}

	ak09918_set_mode(dev, l_mode);
	return ret;
}


static int ak09918_set_mode(AK09918_dev_t *dev, AK09918_mode_type_t mode)
{
    uint8_t buf;
    buf = mode;
    I2C_1_WriteBytes(dev->addr, AK09918_CNTL2, 1, &buf);
#if 0
    int ret;
	if (ret) {
        DebugPrintf("Error: AK09918_ERR_WRITE_FAILED(%d).\n", ret);
		return AK09918_ERR_WRITE_FAILED;
	}
#endif
	return AK09918_ERR_OK;
}

int ak09918_init(AK09918_dev_t *dev)
{
    uint8_t buf;
    int32_t ret;

#if 0
    memset(dev, 0, sizeof(AK09918_dev_t));
#endif

	dev->addr = AK09918_I2C_ADDR;
	dev->mode = AK09918_NORMAL;
    
    /* Read company's ID. */
	I2C_1_ReadBytes(dev->addr, AK09918_WIA1, 1, &buf);
    //DebugPrintf("Info: %02x\n", buf);
#if 1
    if(buf != 0x48){
        DebugPrintf("Error: Read Company's id failed(%08x).\n", buf);
        return AK09918_ERR_READ_FAILED;
    }
#endif
	ret = ak09918_set_mode(dev, AK09918_NORMAL);
    if(ret != AK09918_ERR_OK){
        return AK09918_ERR_WRITE_FAILED;
    }
    /* Read chip's ID. */
	I2C_1_ReadBytes(dev->addr, AK09918_WIA2, 1, &buf);
    //DebugPrintf("Info: %02x\n", buf);
#if 1
    if(buf != 0x0C){
        DebugPrintf("Error: Read chip's id failed(%08x).\n", buf);
        return AK09918_ERR_READ_FAILED;
    }
#endif
    /* Self test. */
    ret = ak09918_self_test(dev);
    printf("Info: Self test result - %s\n", ak09918_err_string(ret));
	return AK09918_ERR_OK;
}

#if 0
int ak09918_enable(unsigned char){

}
#endif

int ak09918_power_down(AK09918_dev_t *dev)
{
    int32_t ret;

	ret = ak09918_set_mode(dev, AK09918_POWER_DOWN);
    if(ret != AK09918_ERR_OK){
        return AK09918_ERR_WRITE_FAILED;
    }
    return AK09918_ERR_OK;
}

int ak09918_get_data(AK09918_dev_t *dev)
{
    int32 ret;
    unsigned char retry = 0;
    static uint32_t delay = 1000;

	ak09918_set_mode(dev, AK09918_POWER_DOWN);
    ak09918_set_mode(dev, dev->mode);
    vTaskDelay(1);
    /* Check if data is ready. */
#if 0
    ret = ak09918_is_ready(dev);
    if (ret != AK09918_ERR_OK) {
    	delay++;
    	DebugPrintf("Error: %s usleep = %d us(TAG)\n", ak09918_err_string(ret), (int)delay);
        //vTaskDelay(delay);
        return AK09918_ERR_NOT_RDY;
    }
#else
    while(ret != AK09918_ERR_OK){
        ret = ak09918_is_ready(dev);
        if(retry++ >= 16){
            vTaskDelay(1);
            goto  err1;
        }
    }
#endif
    /* Check if data is skip. */
    ret = ak09918_is_skip(dev);
    if (ret == AK09918_ERR_DOR) {
    	if(delay > 0) delay--;
    	DebugPrintf("Error: %s usleep = %d us(CONT)\n", ak09918_err_string(ret), (int)delay);
    }
    
    /* Acquire data from FIFO. */
    ret = ak09918_read(dev);
    if (ret != AK09918_ERR_OK) {
    	DebugPrintf("Error: %s\n", ak09918_err_string(ret));
    	//vTaskDelay(100);
        return AK09918_ERR_NOT_RDY;
    }
    
#if 1
    /* Print magnetic density in uT. */
    DebugPrintf("%7.2lf\t%7.2lf\t%7.2lf\n", dev->value.axis[0], dev->value.axis[1], dev->value.axis[2]);
#endif

    return 0;
err1:
    return AK09918_ERR_NOT_RDY;
}

/* This is a test bench. */
#define GDB_DN 0
uint16_t ak09918_get_direction(axis_dblock_t *GDB, AK09918_dev_t *CDB)
{
    double roll, pitch, azimuth;
    double dir_x, dir_y;
    uint16_t true_azimuth;
    //uint8_t x,y;
    /* Calculate pitch and roll, in the range (-pi,pi) 
     * 1.pitch = atan2((double)-acc_x, sqrt((long)acc_z*(long)acc_z + (long)acc_y*(long)acc_y));
     * 2.roll = atan2((double)acc_y, sqrt((long)acc_z*(long)acc_z  + (long)acc_x*(long)acc_x));
     */
#if 0
    DebugPrintf("Infor(M^2/S): %7.2lf\t%7.2lf\t%7.2lf\n", GDB->sample_block[0].axia_data[GDB_DN].axis[0], \
        GDB->sample_block[0].axia_data[GDB_DN].axis[1], GDB->sample_block[0].axia_data[GDB_DN].axis[2]);
#endif

#if 0
    /* Print magnetic density in uT. */
    DebugPrintf("Info: ACC:%7.2lf\t%7.2lf\t%7.2lf\n", GDB->sample_block[0].axia_data[GDB_DN].axis[0], \
        GDB->sample_block[0].axia_data[GDB_DN].axis[1], GDB->sample_block[0].axia_data[GDB_DN].axis[2]);

    DebugPrintf("Info: MAG:%7.2lf\t%7.2lf\t%7.2lf\n", CDB->x, CDB->y, CDB->z);

    pitch = atan2((double)-GDB->sample_block[0].axia_data[GDB_DN].axis[0], \
        sqrt(pow((long)GDB->sample_block[0].axia_data[GDB_DN].axis[2], 2) \
        + pow((long)GDB->sample_block[0].axia_data[GDB_DN].axis[1], 2)));
    
    roll = atan2((double)GDB->sample_block[0].axia_data[GDB_DN].axis[1], \
        sqrt(pow((long)GDB->sample_block[0].axia_data[GDB_DN].axis[2], 2) \
        + pow((long)GDB->sample_block[0].axia_data[GDB_DN].axis[0], 2)));
    
    /* Calculate Azimuth (This is a simple test. Washington Ruan @May.31th,2019):
     * Magnetic horizontal components, after compensating for Roll(r) and Pitch(p) are:
     * X_h = X*cos(p) + Y*sin(r)*sin(p) + Z*cos(r)*sin(p)
     * Y_h = Y*cos(r) - Z*sin(r)
     * Azimuth = arcTan(Y_h/X_h)
     * 1.dir_x = (double)mag_x*cos(pitch) + (double)mag_y*sin(roll)*sin(pitch) + (double)mag_z*cos(roll)*sin(pitch);
     * 2.dir_y = (double)mag_y*cos(roll) - (double)mag_z*sin(roll);
     */
    dir_x = (double)CDB->x*cos(pitch) + (double)CDB->y*sin(roll)*sin(pitch) + (double)CDB->z*cos(roll)*sin(pitch);
    dir_y = (double)CDB->y*cos(roll) - (double)CDB->z*sin(roll);
#else   /* For BW02 demo board. */
#if 0   /* BW02's huge demo board. */
    /* Print magnetic density in uT. */
    DebugPrintf("Info: ACC:%7.2lf\t%7.2lf\t%7.2lf\n", GDB->sample_block[0].axia_data[GDB_DN].axis[0], \
        GDB->sample_block[0].axia_data[GDB_DN].axis[1], GDB->sample_block[0].axia_data[GDB_DN].axis[2]);

    DebugPrintf("Info: MAG:%7.2lf\t%7.2lf\t%7.2lf\n", CDB->x, CDB->y, CDB->z);

    pitch = atan2((double)GDB->sample_block[0].axia_data[GDB_DN].axis[0], \
        sqrt(pow((long)GDB->sample_block[0].axia_data[GDB_DN].axis[2], 2) \
        + pow((long)(-GDB->sample_block[0].axia_data[GDB_DN].axis[1]), 2)));

    roll = atan2((double)(-GDB->sample_block[0].axia_data[GDB_DN].axis[1]), \
        sqrt(pow((long)GDB->sample_block[0].axia_data[GDB_DN].axis[2], 2) \
        + pow((long)(-GDB->sample_block[0].axia_data[GDB_DN].axis[0]), 2)));

    /* Calculate Azimuth (This is a simple test. Washington Ruan @May.31th,2019):
     * Magnetic horizontal components, after compensating for Roll(r) and Pitch(p) are:
     * X_h = X*cos(p) + Y*sin(r)*sin(p) + Z*cos(r)*sin(p)
     * Y_h = Y*cos(r) - Z*sin(r)
     * Azimuth = arcTan(Y_h/X_h)
     * 1.dir_x = (double)mag_x*cos(pitch) + (double)mag_y*sin(roll)*sin(pitch) + (double)mag_z*cos(roll)*sin(pitch);
     * 2.dir_y = (double)mag_y*cos(roll) - (double)mag_z*sin(roll);
     */
    dir_x = (double)CDB->x*cos(pitch) + (double)CDB->y*sin(roll)*sin(pitch) + (double)CDB->z*cos(roll)*sin(pitch);
    dir_y = (double)CDB->y*cos(roll) - (double)CDB->z*sin(roll);
#else   /* BW02 */
    /* Print magnetic density in uT. */
#if 1
    DebugPrintf("Info: ACC:%7.2lf\t%7.2lf\t%7.2lf\n", GDB->sample_block[0].axia_data[GDB_DN].axis[0], \
        GDB->sample_block[0].axia_data[GDB_DN].axis[1], GDB->sample_block[0].axia_data[GDB_DN].axis[2]);

    DebugPrintf("Info: MAG:%7.2lf\t%7.2lf\t%7.2lf\n", CDB->value.axis[0] , CDB->value.axis[1], CDB->value.axis[2]);
#endif
    pitch = atan2((double)-GDB->sample_block[0].axia_data[GDB_DN].axis[0], \
        sqrt(pow((long)GDB->sample_block[0].axia_data[GDB_DN].axis[2], 2) \
        + pow((long)GDB->sample_block[0].axia_data[GDB_DN].axis[1], 2)));

    roll = atan2((double)GDB->sample_block[0].axia_data[GDB_DN].axis[1], \
        sqrt(pow((long)GDB->sample_block[0].axia_data[GDB_DN].axis[2], 2) \
        + pow((long)GDB->sample_block[0].axia_data[GDB_DN].axis[0], 2)));

    /* Calculate Azimuth (This is a simple test. Washington Ruan @May.31th,2019):
     * Magnetic horizontal components, after compensating for Roll(r) and Pitch(p) are:
     * X_h = X*cos(p) + Y*sin(r)*sin(p) + Z*cos(r)*sin(p)
     * Y_h = Y*cos(r) - Z*sin(r)
     * Azimuth = arcTan(Y_h/X_h)
     * 1.dir_x = (double)mag_x*cos(pitch) + (double)mag_y*sin(roll)*sin(pitch) + (double)mag_z*cos(roll)*sin(pitch);
     * 2.dir_y = (double)mag_y*cos(roll) - (double)mag_z*sin(roll);
     */
    dir_x = (double)(CDB->value.axis[0])*cos(pitch) + (double)(CDB->value.axis[1])*sin(roll)*sin(pitch) + (double)CDB->value.axis[2]*cos(roll)*sin(pitch);
    dir_y = (double)(CDB->value.axis[1])*cos(roll) - (double)CDB->value.axis[2]*sin(roll);
#endif
#endif
    azimuth = abs((atan2(dir_y, dir_x)/3.1415976)*360);
    true_azimuth = azimuth - (-4.5);
#if 0    
    /* Convert Azimuth in the range (0, 2pi) */
    if(azimuth < 0) {
        azimuth = 2*PI + azimuth;
    }

    true_azimuth = azimuth * (180 / PI) - LocalDeclination;
    if(true_azimuth >= 360){
        true_azimuth = true_azimuth - 360;
    }
#endif
#if 0
    x = 32 + 24 * sin(azimuth);
    y = 32 - 24 * cos(azimuth);
    DebugPrintf("Infor: Direction(%d) = %7.2lf, x = %d, y = %d\n", true_azimuth, azimuth, x, y);
#endif
    return true_azimuth;
}
/* End of this file. */
