/**
 * @File    Compass.c
 * @Date    Sunday, Sep. 22th, 2019 at 14:29:56 PM BJT
 * @Author  Washington Ruan
 * @Email   washingtonxr@live.com
 *
 * This file contains the implementation of compass
 * module.(Application layer)
 *
 * @bug No known bugs.
 **/

#include "Compass.h"
#include <stdio.h>
#include "uart_debug.h" 
#include "FreeRTOS.h"
#include "i2cm_support.h"
#include <math.h>
#include "CAlgorithm.h"
#include "GAlgorithm.h"

static Compass_dev_t *Compass_dev;
static mag4com_info_t *Compass_info;
unsigned char Compass_ReadEn;
static Compass_info_t Compass_Data;

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
unsigned char Compass_Init(void)
{
    unsigned char ret;
    
    dbg_info("Start initializing the compass module.\n");
    /* You may need to initilize the hardware. */
    /* TBD. */
    
    /* Declare and allocate a space for device. */
    Compass_dev = (Compass_dev_t *)pvPortMalloc(sizeof(Compass_dev_t));
    if(Compass_dev == NULL){
        dbg_err("Memory allocate failed - Compass_dev.\n");
    }
    
    memset(Compass_dev, 0, sizeof(Compass_dev_t));

    /* Allocate memory for Compass_info. */
    Compass_info = (mag4com_info_t *)pvPortMalloc(sizeof(mag4com_info_t));
    if(Compass_info == NULL){
        dbg_err("Memory allocate failed - Compass_dev.\n");
    }
    
    memset(Compass_info, 0, sizeof(mag4com_info_t));
    
    /* Initialize driver. */
    /* Initilize database and implement selftest. */
    ret = ak09918_init(&Compass_dev->data);
    if(ret != AK09918_ERR_OK){
        dbg_err("Initialize ak09918 failed(%d).\n", ret);
        return MAG_ERROR;
    }

    dbg_info("Initialize the calgorithm.\n");
    
    /* Initialize the algorithm. */
    ret = mdata_init();
    if(ret != MAG_OK){
        dbg_err("mdata_init failed.\n");
    }
    
    /* Start first magnetic calibnation. */
    TickType_t Compass_Tickstart, Compass_Tickend;
    Compass_Tickstart = xTaskGetTickCount();
    
    ret = Compass_Calibnation(&Compass_dev->data);
    if(ret != MAG_OK){
        dbg_err("Compass_Calibnation failed.\n");
    }
    
    Compass_Tickend = xTaskGetTickCount();
    dbg_info("Compass_Calibnation use time = %d\n", Compass_Tickend - Compass_Tickstart);

    Compass_Sleep();
    
    return MAG_OK;
}

/**
 * This is a function used to unregister and release resources.
 * Return: true or false.
 */
unsigned char Compass_Deinit(void)
{
    vPortFree(Compass_dev);  
    vPortFree(Compass_info); 
    
    dbg_info("Compass resources are released.\n");
    
    return MAG_OK;
}

/**
 * This is a function used to put compass into sleep mode.
 * Return: true or false.
 */
unsigned char Compass_Sleep(void)
{
    dbg_info("Push compass to sleep.\n");
    
    ak09918_power_down(&Compass_dev->data);
    return MAG_OK;
}

/**
 * This is a function used to Calibnation process.
 * Return: true or false.
 */
unsigned char Compass_Calibnation(AK09918_dev_t *data)
{
    unsigned char ret;
    
    /* Executive algorithm. */
    ret = mdata_declination(data, vTaskDelay);
    if(ret != 0){
        return MAG_ERROR;
    }
    
    return MAG_OK;
}

/**
 * This is a function used to be get the true azimuth in north.
 * Input: Glavity data and magnetometer data.
 * Return: Azimuth in double.
 */
#define GDB_ST_NUM 0    /* Drawn a sample from GDB. */
float Compass_Direction(axis_du_t *GDB, Compass_info_t *CDB)
{
    unsigned char ret;

#if 0
    dbg_info("Info: ACC:%7.4f\t%7.4f\t%7.4f\n", GDB->axia_data[GDB_ST_NUM].axis[0], \
            GDB->axia_data[GDB_ST_NUM].axis[1], GDB->axia_data[GDB_ST_NUM].axis[2]);
#endif

    /* Set ODR. */
    Compass_dev->data.mode = AK09918_NORMAL;
    
    /* Get raw data form hardware. */
    ret = ak09918_get_data(&Compass_dev->data);
    if(ret != 0){
        dbg_err("ak09918_get_data failed.\n");
    }

#if 0
    dbg_info("Info: MAG:%7.4f\t%7.4f\t%7.4f\n", -Compass_dev->data.value.axis[0], \
                    -Compass_dev->data.value.axis[1], Compass_dev->data.value.axis[2]);
#endif

#if 0
    memcpy(Compass_info->c_axis, Compass_dev->data.value.axis, AXIS_NUM);
    memcpy(Compass_info->g_axis, GDB->axia_data[GDB_ST_NUM].axis, AXIS_NUM);

    dbg_info("Data for detect direction.-------------------------------------\n");
#endif

    for(unsigned char i = 0; i < AXIS_NUM; i++){
        Compass_info->c_axis[i] = Compass_dev->data.value.axis[i];
        Compass_info->g_axis[i] = GDB->axia_data[GDB_ST_NUM].axis[i];
    }
    
#if 0
    dbg_info("Info: ACC:%7.4f\t%7.4f\t%7.4f\n", Compass_info->g_axis[0], Compass_info->g_axis[1], Compass_info->g_axis[2]);
    dbg_info("Info: MAG:%7.4f\t%7.4f\t%7.4f\n", Compass_info->c_axis[0], Compass_info->c_axis[1], Compass_info->c_axis[2]);
#endif

    ret = mdata_detect_direction(Compass_info);
    if(ret != MAG_OK){
        if(ret == MAG_ORANG){
            dbg_err("Need to Calibnation(%d).\n", ret);
            Compass_info->state = UNLOCKED;
        }else{
            dbg_err("mdata_detect_direction failed(%d).\n", ret);
            Compass_info->state = UNLOCKED;
        }
    }
    
    Compass_info->state = LOCKED;

    CDB->state = Compass_info->state;
    CDB->true_azimuth = Compass_info->true_azimuth;
        
    return MAG_OK;
}

/**
 * This is a function to Compass_LatchData.
 * Input: Glavity data.
 * Return: Compass_Start status: Correct:0, error:1.
 */
unsigned char Compass_LatchData(axis_du_t *GDB)
{
    unsigned char ret;
    /* Get the angle from compass. */
    ret = Compass_Direction(GDB, &Compass_Data);
    if(ret != MAG_OK){
        dbg_err("Compass_Direction latch failed.\n");
    }

    if(Compass_Data.state == LOCKED){
        dbg_info("Info: True_azimuth(%d) = %4.4f\n", Compass_Data.state , Compass_Data.true_azimuth);
    }

    return MAG_OK;
}

/**
 * This is a function to enable compass module.
 * Input: Void.
 * Return: Compass_Start status: Correct:0, error:1.
 */
unsigned char Compass_Start(void)
{
    Compass_ReadEn = 1;
    return 0;
}

/**
 * This is a function to disable compass module.
 * Input: Void.
 * Return: Compass_Start status: Correct:0, error:1.
 */
unsigned char Compass_Stop(void)
{
    if(Compass_ReadEn){
        Compass_ReadEn = 0;
    }
    return 0;
}

/**
 * This is a function to take compass azimuth angle.
 * Input: Void.
 * Return: Azimuth angle.
 */
float Compass_Getdata(void)
{
    return Compass_Data.true_azimuth;
}

/* End of this file. */
