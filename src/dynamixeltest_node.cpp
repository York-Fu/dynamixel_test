
#include "loopTime.h"
#include "sys/time.h"
#include "dynamixelFunc.h"
#include <std_msgs/Float64MultiArray.h>

#define TO_INT16(a, b) ((int16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define GYRO_COEFFICIENT ((1000.0 / 32768) * (M_PI / 180.0))
#define ACC_COEFFICIENT (8.0 * 9.8 / 32768)
#define MAG_COEFFICIENT (1.0)

#define SERVO_COEFF (12.8f) // 4096 / 320.0
#define VALUE_TO_DEGREE(v) ((v - 2048) / SERVO_COEFF)
#define VALUE_TO_DPS(v) (500.0 / 32768 * v)
#define VALUE_TO_MA(v) (5000.0 / 32768 * v)
#define DEGREE_TO_VALUE(v) (v * SERVO_COEFF + 2048)
#define DPS_TO_VALUE(v) (32768 / 500.0 * v)
#define MA_TO_VALUE(v) (32768 / 5000.0 * v)

typedef struct
{
  double_t x;
  double_t y;
  double_t z;
} Axis_t;

typedef struct
{
  double_t roll;
  double_t pitch;
  double_t yaw;
} EulerAngle_t;

typedef struct
{
  Axis_t angularVel;
  Axis_t linearAcc;
  Axis_t magnetic;
  EulerAngle_t eulerAngle;
} ImuParam_t;

typedef struct
{
  Axis_t force;
  Axis_t moment;
} ForceParam_t;

typedef struct
{
  double_t position;
  double_t velocity;
  double_t current;
} JointParam_t;

uint8_t dxlIds[255] = {0};
uint8_t numberOfId = 0;

uint8_t jointIds[22] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
ForceParam_t forceData[2];
JointParam_t jointData[22];
JointParam_t jointCmd[22];
ImuParam_t imuData;
double_t distance = 0.0;

ros::Publisher imuPub;
ros::Publisher distancePub;
ros::Publisher jointPosPub;
ros::Publisher jointVelPub;
ros::Publisher jointCurretPub;
ros::Publisher jointCmdPosPub;
ros::Publisher jointCmdVelPub;
ros::Publisher jointCmdCurrentPub;

void imuConvertRawData(int32_t *rawData, ImuParam_t &data)
{
  uint16_t index = 0;
  data.angularVel.x = GYRO_COEFFICIENT * TO_INT16(rawData[index++], rawData[index++]);
  data.angularVel.y = GYRO_COEFFICIENT * TO_INT16(rawData[index++], rawData[index++]);
  data.angularVel.z = GYRO_COEFFICIENT * TO_INT16(rawData[index++], rawData[index++]);
  data.linearAcc.x = ACC_COEFFICIENT * TO_INT16(rawData[index++], rawData[index++]);
  data.linearAcc.y = ACC_COEFFICIENT * TO_INT16(rawData[index++], rawData[index++]);
  data.linearAcc.z = ACC_COEFFICIENT * TO_INT16(rawData[index++], rawData[index++]);
  index += 1;
  data.magnetic.x = MAG_COEFFICIENT * TO_INT16(rawData[index++], rawData[index++]);
  data.magnetic.y = MAG_COEFFICIENT * TO_INT16(rawData[index++], rawData[index++]);
  data.magnetic.z = MAG_COEFFICIENT * TO_INT16(rawData[index++], rawData[index++]);
}

bool readData()
{
  bool result = false;
  uint8_t boardNum = 1;
  uint8_t jointNum = 0;
  uint8_t fsrNum = 0;
  uint8_t ids[] = {
      200,
      // 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
      // 13, 14, 15, 16, 17, 18, 19, 20,
      // 21, 22,
      // 111, 112,
  };
  uint16_t addr[] = {
      24,
      // 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36,
      // 36, 36, 36, 36, 36, 36, 36, 36,
      // 36, 36,
      // 90, 90,
  };
  uint16_t length[] = {
      56,
      // 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
      // 6, 6, 6, 6, 6, 6, 6, 6,
      // 2, 2,
      // 4, 4,
  };
  int32_t rData[1024] = {0};
  int32_t tempData[256] = {0};

  result = dxlBulkRead(ids, jointNum + fsrNum + boardNum, addr, length, rData);
  if (result == false)
  {
    return false;
  }
  if (boardNum > 0)
  {
    imuConvertRawData(&rData[38 - 24], imuData);
    memcpy(tempData, &rData[59 - 24], 2);
    distance = (((uint32_t)tempData[0] << 8) & 0xff00) | ((uint32_t)tempData[1] & 0xff);
  }
  if (jointNum > 0)
  {
    memcpy(tempData, &rData[boardNum * 56], jointNum * 6);
    for (uint8_t i = 0; i < jointNum; i++)
    {
      jointData[ids[i] - 1].position = VALUE_TO_DEGREE(DXL_MAKEWORD(tempData[6 * i + 0], tempData[6 * i + 1]));
      jointData[ids[i] - 1].velocity = VALUE_TO_DPS((int16_t)((tempData[6 * i + 2] & 0xff) | (uint16_t)(tempData[6 * i + 3] << 8)));
      jointData[ids[i] - 1].current = VALUE_TO_MA((int16_t)((tempData[6 * i + 4] & 0xff) | (uint16_t)(tempData[6 * i + 5] << 8)));
    }
  }
  if (fsrNum > 0)
  {
    memcpy(tempData, &rData[boardNum * 56 + jointNum * 6], fsrNum * 4);
  }
  return true;
}

bool getJointData(uint8_t *ids, uint8_t number, JointParam_t *data)
{
  uint16_t addr[number] = {0};
  uint16_t length[number] = {0};
  int32_t rData[1024] = {0};
  bool result = false;
  for (uint8_t i = 0; i < number; i++)
  {
    addr[i] = 36;
    length[i] = 6;
  }
  result = dxlBulkRead(ids, number, addr, length, rData);
  if (result == false)
  {
    return false;
  }
  for (uint8_t i = 0; i < number; i++)
  {
    data[i].position = VALUE_TO_DEGREE(DXL_MAKEWORD(rData[6 * i + 0], rData[6 * i + 1]));
    data[i].velocity = VALUE_TO_DPS((int16_t)((rData[6 * i + 2] & 0xff) | (uint16_t)(rData[6 * i + 3] << 8)));
    data[i].current = VALUE_TO_MA((int16_t)((rData[6 * i + 4] & 0xff) | (uint16_t)(rData[6 * i + 5] << 8)));
  }
  return true;
}

bool setJointPosition(uint8_t *ids, uint8_t number, JointParam_t *param)
{
  bool result = false;
  const char *log = NULL;
  int32_t wData[1024] = {0};
  for (uint8_t i = 0; i < number; i++)
  {
    wData[i] = 1;
  }
  result = dxlWb.syncWrite(3, ids, number, wData, 1, &log);
  if (result == false)
  {
    ROS_ERROR("set joint velocity failed, %s", log);
    // ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
    return false;
  }
  for (uint8_t i = 0; i < number; i++)
  {
    wData[i] = DEGREE_TO_VALUE(param[i].position);
  }
  result = dxlWb.syncWrite(0, ids, number, wData, 1, &log);
  if (result == false)
  {
    ROS_ERROR("set joint postion failed, %s", log);
    // ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
    return false;
  }
  return true;
}

bool setJointVelocity(uint8_t *ids, uint8_t number, JointParam_t *param)
{
  bool result = false;
  const char *log = NULL;
  int32_t wData[1024] = {0};
  for (uint8_t i = 0; i < number; i++)
  {
    wData[i] = 2;
  }
  result = dxlWb.syncWrite(3, ids, number, wData, 1, &log);
  if (result == false)
  {
    ROS_ERROR("set joint velocity failed, %s", log);
    // ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
    return false;
  }
  for (uint8_t i = 0; i < number; i++)
  {
    wData[i] = DPS_TO_VALUE(param[i].velocity);
  }
  result = dxlWb.syncWrite(1, ids, number, wData, 1, &log);
  if (result == false)
  {
    ROS_ERROR("set joint velocity failed, %s", log);
    // ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
    return false;
  }
  return true;
}

bool setJointTorque(uint8_t *ids, uint8_t number, JointParam_t *param)
{
  bool result = false;
  const char *log = NULL;
  int32_t wData[1024] = {0};
  for (uint8_t i = 0; i < number; i++)
  {
    wData[i] = 3;
  }
  result = dxlWb.syncWrite(3, ids, number, wData, 1, &log);
  if (result == false)
  {
    ROS_ERROR("set joint velocity failed, %s", log);
    // ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
    return false;
  }
  for (uint8_t i = 0; i < number; i++)
  {
    wData[i] = MA_TO_VALUE(param[i].current);
  }
  result = dxlWb.syncWrite(2, ids, number, wData, 1, &log);
  if (result == false)
  {
    ROS_ERROR("set joint current failed, %s", log);
    // ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
    return false;
  }
  return true;
}

void topicInit()
{
  ros::NodeHandle nh;
  imuPub = nh.advertise<std_msgs::Float64MultiArray>("/sensor/imu", 10);
  distancePub = nh.advertise<std_msgs::Float64MultiArray>("/sensor/distance", 10);
  jointPosPub = nh.advertise<std_msgs::Float64MultiArray>("/sensor/joint/position", 10);
  jointVelPub = nh.advertise<std_msgs::Float64MultiArray>("/sensor/joint/velocity", 10);
  jointCurretPub = nh.advertise<std_msgs::Float64MultiArray>("/sensor/joint/current", 10);
  jointCmdPosPub = nh.advertise<std_msgs::Float64MultiArray>("/command/joint/position", 10);
  jointCmdVelPub = nh.advertise<std_msgs::Float64MultiArray>("/command/joint/velocity", 10);
  jointCmdCurrentPub = nh.advertise<std_msgs::Float64MultiArray>("/command/joint/current", 10);
}

void topicPub()
{
  std_msgs::Float64MultiArray f64ArrayMsg;
  f64ArrayMsg.data.resize(9);
  f64ArrayMsg.data[0] = imuData.angularVel.x * (180.0 / M_PI);
  f64ArrayMsg.data[1] = imuData.angularVel.y * (180.0 / M_PI);
  f64ArrayMsg.data[2] = imuData.angularVel.z * (180.0 / M_PI);
  f64ArrayMsg.data[3] = imuData.linearAcc.x;
  f64ArrayMsg.data[4] = imuData.linearAcc.y;
  f64ArrayMsg.data[5] = imuData.linearAcc.z;
  f64ArrayMsg.data[6] = imuData.magnetic.x;
  f64ArrayMsg.data[7] = imuData.magnetic.y;
  f64ArrayMsg.data[8] = imuData.magnetic.z;
  imuPub.publish(f64ArrayMsg);

  f64ArrayMsg.data.resize(1);
  f64ArrayMsg.data[0] = distance;
  distancePub.publish(f64ArrayMsg);

  f64ArrayMsg.data.resize(22);
  for (uint16_t i = 0; i < 22; i++)
    f64ArrayMsg.data[i] = jointData[i].position;
  jointPosPub.publish(f64ArrayMsg);

  f64ArrayMsg.data.resize(22);
  for (uint16_t i = 0; i < 22; i++)
    f64ArrayMsg.data[i] = jointData[i].velocity;
  jointVelPub.publish(f64ArrayMsg);

  f64ArrayMsg.data.resize(22);
  for (uint16_t i = 0; i < 22; i++)
    f64ArrayMsg.data[i] = jointData[i].current;
  jointCurretPub.publish(f64ArrayMsg);

  f64ArrayMsg.data.resize(22);
  for (uint16_t i = 0; i < 22; i++)
    f64ArrayMsg.data[i] = jointCmd[i].position;
  jointCmdPosPub.publish(f64ArrayMsg);

  f64ArrayMsg.data.resize(22);
  for (uint16_t i = 0; i < 22; i++)
    f64ArrayMsg.data[i] = jointCmd[i].velocity;
  jointCmdVelPub.publish(f64ArrayMsg);

  f64ArrayMsg.data.resize(22);
  for (uint16_t i = 0; i < 22; i++)
    f64ArrayMsg.data[i] = jointCmd[i].current;
  jointCmdCurrentPub.publish(f64ArrayMsg);
}

void bulkreadTest()
{
  bool result = false;
  uint32_t count = 0, errCount = 0;
  struct timeval t0, t1;
  double t = 0;

  ros::Rate loopRate(200);
  TimeoutCheck timeCheck(0.01);
  gettimeofday(&t0, NULL);
  while (ros::ok())
  {
    result = readData();
    if (result == false)
    {
      errCount++;
      // getchar();
      readData();
      // getchar();
    }
    else
    {
      count++;
    }
    topicPub();
    gettimeofday(&t1, NULL);
    t = (t1.tv_sec + t1.tv_usec * 1e-6) - (t0.tv_sec + t0.tv_usec * 1e-6);
    std::cout << "status: " << count << ", " << errCount << ", " << t << "\r";

    loopRate.sleep();
    // timeCheck.check("loop");
  }
  std::cout << "\nstatus: " << count << ", " << errCount << ", " << t << "\n";
}

double_t get_sin_wave(double_t A, double_t T, double_t b, double_t dt)
{
  static double_t time = 0, out = 0;
  out = (A * sin((2 * M_PI / T) * time) + b);
  time += dt;
  if (time >= T) // prevention overflow
  {
    time = 0;
  }
  return out;
}

double_t get_square_wave(double_t A, double_t T, double_t b, double_t dt)
{
  static double_t time = 0, out = 0;
  if (time < (T / 2.0))
  {
    out = A + b;
  }
  else
  {
    out = -A + b;
  }
  time += dt;
  if (time >= T)
  {
    time = 0;
  }
  return out;
}

double_t get_triangular_wave(double_t A, double_t T, double_t b, double_t dt)
{
  static double_t time = 0, out = 0;
  if (time < (T / 2.0))
  {
    out = (A / (T / 2.0)) * time + b;
  }
  else
  {
    out = (A / (T / 2.0)) * (T - time) + b;
  }
  time += dt;
  if (time >= T)
  {
    time = 0;
  }
  return out;
}

void jointMove(uint8_t id, double_t goal_pos, double_t vel)
{
  double_t dt = 0.05; // s
  double_t A = 0, T = 0;
  uint32_t count = 0, total_cnt = 0;
  struct timespec next_time;

  getJointData(&jointIds[id - 1], 1, &jointData[id - 1]);
  A = goal_pos - jointData[id - 1].position;
  T = fabs(A) / vel;
  total_cnt = (uint32_t)(T / dt);
  printf("start:%f stop:%f T:%f \n", jointData[id - 1].position, goal_pos, T);
  if (fabs(A) < 1e-4)
    return;

  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (1)
  {
    count++;
    jointCmd[id - 1].position = A * sin((2 * M_PI / (4 * T)) * (count * dt)) + jointData[id - 1].position;
    setJointPosition(&jointIds[id - 1], 1, &jointCmd[id - 1]);
    if (count >= total_cnt)
    {
      break;
    }

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}

void jointTest()
{
  double_t dt = 0.005;
  ros::Rate loopRate(1.0 / dt);
  // TimeoutCheck timeCheck(dt);
  dxlTorqueOn(true, dxlIds, &numberOfId);
  jointMove(1, 0, 20);
  while (ros::ok())
  {
    getJointData(jointIds, 1, jointData);

    // jointCmd[0].current = get_square_wave(100, 1, 0, dt);
    // setJointTorque(jointIds, 1, jointCmd);
    // jointCmd[0].velocity = get_square_wave(40, 1, 0, dt);
    // setJointVelocity(jointIds, 1, jointCmd);
    jointCmd[0].position = get_sin_wave(5, 1, 0, dt);
    setJointPosition(jointIds, 1, jointCmd);

    topicPub();
    loopRate.sleep();
    // timeCheck.check("loop");
  }
}

int main(int argc, char **argv)
{
  bool result = false;

  ros::init(argc, argv, "dynamixel_test");
  ros::NodeHandle nh;
  std::cout << "dynamixel init...\n";
  result = dynamixelInit(dxlIds, &numberOfId);
  if (result == false)
  {
    std::cout << "exit!\n";
    return 0;
  }
  std::cout << "dynamixel init complate.\n";
  topicInit();

  // bulkreadTest();
  jointTest();

  return 0;
}