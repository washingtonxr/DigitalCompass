/**
 * @File    CAlgorithm.h
 * @Date    Sunday, Sep. 22th, 2019 at 10:42:05 AM BJT
 * @Author  Washington Ruan
 * @Email   washingtonxr@live.com
 *
 * This file contains the implementation of magnetic sensor device for compass
 * module.
 *
 * @bug No known bugs.
 **/

#ifndef CALGORITHM_H
#define CALGORITHM_H

#include "CAlgorithmCom.h"
#if 0
/* Sign for debugging. */
#define CA_DEBUG            1
#endif
/* Parameter for algorithm. */
#define CAVERSION           0.10    /* Main algorithm version. */
#define PI                  3.141592653589793638f

#define MAG_RATION_TOP      1.1f
#define MAG_RATION_BOTTOM   1.1f

#define ABS(a)              (0 - (a)) > 0 ? (-(a)) : (a)
#define MAX(a,b)            ((a) > (b) ? (a) : (b)) 
#define MIN(a,b)            ((a) < (b) ? (a) : (b))

/* Structure of algorithm. */
typedef struct{
    float axis[AXIS_NUM];
}c_axis_t;

typedef struct{
    c_axis_t max;
    c_axis_t min;
    float detal[AXIS_NUM];
}Calgorithm_db_t;

typedef void (* Compass_delay_func)(unsigned int mmdelay);

unsigned char mdata_init(void);
unsigned char mdata_detect_direction(mag4com_info_t *data);
unsigned char mdata_declination(AK09918_dev_t      *dev, Compass_delay_func c_delay);

#endif
/* End of this file. */
