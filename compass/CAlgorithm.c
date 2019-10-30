/**
 * @File    CAlgorithm.c
 * @Date    Sunday, Sep. 22th, 2019 at 10:42:05 AM BJT
 * @Author  Washington Ruan
 * @Email   washingtonxr@live.com
 *
 * This file contains the implementation of magnetic sensor device for compass
 * module.
 *
 * @bug No known bugs.
 **/

#include "CAlgorithm.h"
#include <string.h>
#include <math.h>

static Calgorithm_db_t caldb;
static unsigned char mdata_normalization(mag4com_info_t *data);

/**
 * Implement the algorithm.
 * Input: Gravity value and magnetic value in each axis.
 * Return: True azimuth.
 */
unsigned char mdata_detect_direction(mag4com_info_t *data)
{
    unsigned char i;
    unsigned char ret;
    
    /* Roll angle. */
    float roll;
    
    /* Pitch angle. */
    float pitch;
    
    /* The horizontal components. */
    float magvector_x, magvector_y;
    
    /* Magnetic north synthesis direction angle. */
    float azimuth;
    
    /* Check magnetometer data in the range of caldb. Or need to run declination process again. */
    for(i = 0; i < AXIS_NUM; i++){
        
        /* Check top range. */
        if(data->c_axis[i] >= caldb.max.axis[i] * MAG_RATION_TOP){
#ifdef CA_DEBUG
            printf("MAG_ORANG(MAX): %4.4f\t%4.4f\n", data->c_axis[i], caldb.max.axis[i]);
#endif
            return MAG_ORANG;
        }
        
        /* Check bottom range. */
        if(data->c_axis[i] <= caldb.min.axis[i] * MAG_RATION_BOTTOM){
#ifdef CA_DEBUG
            printf("MAG_ORANG(MIN): %4.4f\t%4.4f\n", data->c_axis[i], caldb.min.axis[i]);
#endif
            return MAG_ORANG;
        }
    }
    
    ret = mdata_normalization(data);
    if(ret != MAG_OK){
        return MAG_ERROR;
    }
    
#ifdef CA_DEBUG
    printf("Cdata after normalization:%4.4f\t%4.4f\t%4.4f\n", data->c_axis[0], data->c_axis[1], data->c_axis[2]);
#endif

    /* Caculate pitch angle. */
    pitch = atan2(-data->g_axis[0], sqrt(pow(data->g_axis[1], 2) + pow(data->g_axis[2], 2)));
    
    /* Calculate roll angle. */
    roll = atan2(data->g_axis[1], sqrt(pow(data->g_axis[0], 2) + pow(data->g_axis[2], 2)));
    
    /* Calculate the component of the geomagnetic in horizontal plane(X-axis)*/
    magvector_x = -data->c_axis[0]*cos(pitch) - data->c_axis[1]*sin(roll)*sin(pitch)\
                    + data->c_axis[2]*cos(roll)*sin(pitch);

    /* Calculate the component of the geomagnetic in horizontal plane(Y-axis)*/
    magvector_y = -data->c_axis[1]*cos(roll) - data->c_axis[2]*sin(roll);

#ifdef CA_DEBUG
    printf("pitch = %4.4f\n", pitch);
    printf("roll = %4.4f\n", roll);

    printf("magvector_x = %4.4f\n", magvector_x);
    printf("magvector_y = %4.4f\n", magvector_y);
#endif

    /* Calculate azimuth. */
    azimuth = (-atan2(magvector_y, magvector_x)/PI)*180 + 180;

#ifdef CA_DEBUG
    printf("azimuth = %4.4f\n", azimuth);
#endif

    /* Calculate true azimuth. */
    data->true_azimuth = azimuth - MagDecl8Fuzhou;

#ifdef CA_DEBUG
    printf("true_azimuth = %4.4f\n", data->true_azimuth);
#endif

    return MAG_OK;
}

/**
 * Normalize the data.
 * Return: Value.
 */
static unsigned char mdata_normalization(mag4com_info_t *data)
{
    unsigned char i;
    
    for(i = 0; i < AXIS_NUM; i++){
        
        /* Remove the bias in Y asis after normalization. */
        data->c_axis[i] = (data->c_axis[i] - caldb.min.axis[i])/caldb.detal[i];
        
        /* Normalized and symmetric with x asis. */
        data->c_axis[i]  = data->c_axis[i]*2 - 1;
    }
    return MAG_OK;
}

/**
 * Get the range for declination.
 * Return: True of false.
 */
unsigned char mdata_declination(AK09918_dev_t *dev, Compass_delay_func c_delay)
{
    unsigned int cnt;
    unsigned char i;

    /* Initilize the value. */
    for(i = 0; i < AXIS_NUM; i++){
        caldb.max.axis[i] = -5000.0f;
        caldb.min.axis[i] = 5000.0f;
        printf("Max:%f\tMin:%f\n", caldb.max.axis[i], caldb.min.axis[i]);
    }
    
#if 1
    printf("Compass declinating start: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
#endif

    for(cnt = 0; cnt < MAG_INT_TIME; cnt++){

        /* Get raw data form hardware. */
        ak09918_get_data(dev);

        /* Latch maximun and minimum data. */
        for(i = 0; i < AXIS_NUM; i++){
            caldb.max.axis[i] = MAX(caldb.max.axis[i], dev->value.axis[i]);
            caldb.min.axis[i] = MIN(caldb.min.axis[i], dev->value.axis[i]);
        }

        /* Delay for 50mm. */
        c_delay(50);
    }

#if 1
    printf("Done.\n");
#endif

    for(i = 0; i < AXIS_NUM; i++){
        /* Calculate detal. */
        caldb.detal[i] = caldb.max.axis[i] - caldb.min.axis[i];
    }

#ifdef CA_DEBUG

    /* Just for debugging. */
    printf("============================================\n");
    
    printf("|Axis\tMax\tMin\tDetal\t|\n");
    for(i = 0; i < AXIS_NUM; i++){
        printf("--------------------------------------------\n");
        printf("|%d\t%4.4f\t%4.4f\t%4.4f\t|\n", i, caldb.max.axis[i], caldb.min.axis[i], caldb.detal[i]);
    }
    
    printf("============================================\n");

#endif
    
    return MAG_OK;
}

unsigned char mdata_init(void)
{
    unsigned char i;

    memset(&caldb, 0, sizeof(caldb));

    /* Initilize the value. */
    for(i = 0; i < AXIS_NUM; i++){
        caldb.max.axis[i] = -4000;
        caldb.min.axis[i] = 4000;
    }
    return MAG_OK;
}

void mdata_version(void)
{
    printf("CAlgorithm Version:%f\n", CAVERSION);
}

/* End of this file. */
