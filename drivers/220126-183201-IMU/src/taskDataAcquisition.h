/** ***************************************************************************
 * @file   taskDataAcquisition.h
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * sensor data acquisition task runs at 100Hz, gets the data for each sensor
 * and applies available calibration
 ******************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/

#ifndef _TASK_DATA_ACQUISITION_H_
#define _TASK_DATA_ACQUISITION_H_

uint32_t imu_count; // Number of IMU measurements
uint32_t cam_count; // Number of camera triggers sent
uint32_t cam_stamp; // Last stamp of camera trigger signal
BOOL trigger_flag;  // Flag to indicate whether the camera was triggered!
uint8_t camera_frequency; // Flag to indicate if the cameras are ready to be triggered
uint8_t divider;
BOOL resetSystem(uint8_t freq); // Reset the above parameters

#include "stdint.h"
#include "GlobalConstants.h"
extern void TaskDataAcquisition(void const *argument);
extern void PrepareToNewDacqTick();
extern void TaskDataAcquisition_Init(void);
extern void GetSensorsData(void);
extern void EnterMainAlgLoop(void);
extern void DataAquisitionStart(void);
extern BOOL isOneHundredHertzFlag(void);


#endif