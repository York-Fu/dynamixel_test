#include "dynamixelFunc.h"

DynamixelWorkbench dxlWb;

bool dynamixelHandlersInit(uint8_t *ids)
{
  bool result = false;
  const char *log = NULL;

  result = dxlWb.addSyncWriteHandler(ids[0], "Goal_Position", &log); // 至少存在一个舵机
  if (result == false)
  {
    ROS_ERROR("addSyncWriteHandler pos failed, %s", log);
    return false;
  }

  result = dxlWb.addSyncWriteHandler(ids[0], "Moving_Speed", &log);
  if (result == false)
  {
    ROS_ERROR("addSyncWriteHandler vel failed, %s", log);
    return false;
  }

  result = dxlWb.addSyncWriteHandler(ids[0], "Punch", &log);
  if (result == false)
  {
    ROS_ERROR("addSyncWriteHandler torque failed, %s", log);
    return false;
  }

  result = dxlWb.addSyncWriteHandler(ids[0], "Lock", &log);
  if (result == false)
  {
    ROS_ERROR("addSyncWriteHandler mode failed, %s", log);
    return false;
  }

  result = dxlWb.initBulkRead(&log);
  if (result == false)
  {
    ROS_ERROR("initBulkRead failed, %s", log);
    return false;
  }

  return true;
}

bool dynamixelInit(uint8_t *ids, uint8_t *numberOfId)
{
  const char *portName = "/dev/ttyUSB0";
  int baudrate = 1000000;
  const char *log = NULL;
  bool result = false;
  result = dxlWb.init(portName, baudrate, &log); // initWorkbench
  if (result == false)
  {
    ROS_ERROR("open port failed, log: %s", log);
    return false;
  }
  else
    ROS_INFO("open port succeed, baudrate: %d", baudrate);

  ROS_INFO("scan...");
  dxlWb.scan(ids, numberOfId, 22, &log);
  if (*numberOfId > 0)
  {
    ROS_INFO("number of servo devices: %d", *numberOfId);
    std::cout << "device id:" << std::endl;
    for (uint8_t i = 0; i < *numberOfId; i++)
    {
      std::cout << (int)ids[i] << "  ";
    }
    std::cout << std::endl;

    result = dynamixelHandlersInit(ids);
    if (result == false)
    {
      ROS_ERROR("initDxlHandlers failed!");
      return false;
    }
  }
  else
  {
    ROS_ERROR("no dynamixel device found!");
    return false;
  }

  return true;
}

bool dxlTorqueOn(bool on, uint8_t *ids, uint8_t *numberOfId)
{
  bool result = false;
  const char *log = NULL;
  std::cout << "set torque\n";
  for (uint8_t i = 0; i < *numberOfId; i++)
  {
    if (on)
      result = dxlWb.torqueOn(ids[i], &log);
    else
      result = dxlWb.torqueOff(ids[i], &log);
    if (result == false)
    {
      ROS_WARN("in %s function, torqueOn %d failed, %s", __func__, ids[i], log);
      return false;
    }
  }
  return true;
}

bool dxlBulkRead(uint8_t *ids, uint16_t number, uint16_t *addr, uint16_t *length, int32_t *rawData)
{
  bool result = false;
  const char *log = NULL;
  dxlWb.clearBulkReadParam();
  for (uint8_t i = 0; i < number; i++)
  {
    result = dxlWb.addBulkReadParam(ids[i], addr[i], length[i], &log);
    if (result == false)
    {
      ROS_ERROR("addBulkReadParam failed, %s", log);
      // ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
      return false;
    }
  }
  result = dxlWb.bulkRead(&log);
  if (result == false)
  {
    ROS_ERROR("bulkRead failed, %s", log);
    // ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
    return false;
  }
  result = dxlWb.getRawBulkReadData(ids, number, addr, length, rawData, &log);
  if (result == false)
  {
    ROS_ERROR("getRawBulkReadData failed, %s", log);
    // ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
    return false;
  }
  return true;
}
