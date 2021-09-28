#ifndef _dynamixelFunc_h_
#define _dynamixelFunc_h_

#include <iostream>
#include "ros/ros.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

extern DynamixelWorkbench dxlWb;

bool dynamixelInit(uint8_t *ids, uint8_t *numberOfId);
bool dxlTorqueOn(bool on, uint8_t *ids, uint8_t *numberOfId);
bool dxlBulkRead(uint8_t *ids, uint16_t number, uint16_t *addr, uint16_t *length, int32_t *rawData);

#endif
