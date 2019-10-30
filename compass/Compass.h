/**
 * @File    Compass.h
 * @Date    Sunday, Sep. 22th, 2019 at 14:29:58 PM BJT
 * @Author  Washington Ruan
 * @Email   washingtonxr@live.com
 *
 * This file contains the implementation of compass
 * module.(Application layer)
 *
 * @bug No known bugs.
 **/
 
#ifndef COMPASS_H
#define COMPASS_H

#include "CAlgorithmCom.h"
#include "ak09918.h"

typedef struct{
    float mag_decl;             /* Magnetic declination. */
    mag4com_state_t state;      /* State of compass. */
    AK09918_dev_t data;         /* Database of device. */
}Compass_dev_t;

typedef struct{
    /* True azimuth in your current location. */
    float true_azimuth;
    /* Magnetometer state. */
    mag4com_state_t state;
}Compass_info_t;

extern unsigned char Compass_ReadEn;
/**
 * This is a function used to register resources, driver and algorithm.
 * Notice: 
 *          1, Don't forget to initilize the hardware before using this part!!!
 *          2, You need to take care about the true local magnetic declination. 
 *             I recommend querying or import the interface ahead of using. You
 *             need to do some algorithm actually(NASA's magnatic declination is good).
 *
 * Author: Washington Ruan
 * Return: True or false.
 * Reversion: 1.0 of course(Spe. 22th, 2019).
 * Bug list: Nothing.
 */
unsigned char Compass_Init(void);

/**
 * This is a function used to unregister and release resources.
 * Return: true or false.
 */
unsigned char Compass_Deinit(void);

/**
 * This is a function used to put compass into sleep mode.
 * Return: true or false.
 */
unsigned char Compass_Sleep(void);

/**
 * This is a function used to Calibnation process.
 * Return: true or false.
 */
unsigned char Compass_Calibnation(AK09918_dev_t *data);

/**
 * This is a function used to be get the true azimuth in north.
 * Input: Glavity data and magnetometer data.
 * Return: Azimuth in double.
 */
float Compass_Direction(axis_du_t *GDB, Compass_info_t *CDB);

/**
 * This is a function to Compass_LatchData.
 * Input: Glavity data.
 * Return: Compass_Start status: Correct:0, error:1.
 */
unsigned char Compass_LatchData(axis_du_t *GDB);

/**
 * This is a function to enable compass module.
 * Input: Void.
 * Return: Compass_Start status: Correct:0, error:1.
 */
unsigned char Compass_Start(void);

/**
 * This is a function to disable compass module.
 * Input: Void.
 * Return: Compass_Start status: Correct:0, error:1.
 */
unsigned char Compass_Stop(void);

/**
 * This is a function to take compass azimuth angle.
 * Input: Void.
 * Return: Azimuth angle.
 */
float Compass_Getdata(void);


#endif
/* End of this file. */
