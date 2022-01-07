/**
 * @file tuya_mpu_daq.h
 * @author lifan
 * @brief MPU6050 DAQ (Data Acquisition) module header file
 * @version 1.0.0
 * @date 2022-01-07
 *
 * @copyright Copyright (c) tuya.inc 2022
 *
 */

#ifndef __TUYA_MPU_DAQ_H__
#define __TUYA_MPU_DAQ_H__

#include "tuya_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************
************************micro define************************
***********************************************************/

/***********************************************************
***********************typedef define***********************
***********************************************************/

/***********************************************************
***********************variable define**********************
***********************************************************/

/***********************************************************
***********************function define**********************
***********************************************************/
/**
 * @brief MPU6050 DAQ module init
 * @param[in] none
 * @return none
 */
VOID_T tuya_mpu_daq_init(VOID_T);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __TUYA_MPU_DAQ_H__ */
