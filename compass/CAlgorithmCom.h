/**
 * @File    CAlgorithmCom.h
 * @Date    Sunday, Sep. 22th, 2019 at 10:42:05 AM BJT
 * @Author  Washington Ruan
 * @Email   washingtonxr@live.com
 *
 * This file contains the implementation of magnetic sensor device for compass
 * module.
 *
 * @bug No known bugs.
 **/

#ifndef CALGORITHMCOM_H
#define CALGORITHMCOM_H

#include "ak09918.h"
#if 0
#include "Peripherals.h"
#endif

#if 1
#define MAG_ORANG           2       /* Current data out of range and need to declination. */
#define MAG_ERROR           1       /* Back error. */
#define MAG_OK              0       /* That is right. */
#else
typedef enum{
    MAG_OK = 0,                     /* That is right. */
    MAG_ERROR,                      /* Back error. */
    MAG_ORANG                       /* Current data out of range and need to declination. */
}mag4retinfo_t;

#endif
#define MAG_INT_TIME        120     /* The number of sample points required for initialization. */
#define AXIS_NUM            3       /* Number of Sensor Axis Number. */

#define MagDecl8Fuzhou      -4.5f   /* True north degree(Check from NASA).*/

#ifndef NOEXPORTLIB
#include "uart_debug.h"
#endif

/* The sign of whether an initial is required? */
typedef enum{
    UNLOCKED = 0,
    LOCKED
}mag4com_state_t;

/* Magnetometer' informantion 4 compass. */
typedef struct {
    /* Current galivity value in each axis. */
    float g_axis[AXIS_NUM];           /* x,y,z axis' RAW data. */
    /* Current magnetic value in each axis. */
    float c_axis[AXIS_NUM];
    /* True azimuth in your current location. */
    float true_azimuth;
    /* Magnetometer state. */
    mag4com_state_t data_state;
    /* Calibnate state. */
    mag4com_state_t calib_status;

}mag4com_info_t;

#endif
/* End of this file. */
