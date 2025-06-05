/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include "Dynamixel2Arduino.h"

namespace DYNAMIXEL{

const uint16_t model_number_table[] PROGMEM = {
    AX12A, AX12W, AX18A,
    
    RX10, RX24F, RX28, RX64,
    
    DX113, DX116, DX117,
    
    EX106,

    MX12W,  MX28,   MX64,    MX106,
    MX28_2, MX64_2, MX106_2,
    
    XL320,
    XL330_M288,
    XL330_M077,
    XC330_M181,
    XC330_M288,    
    XC330_T181,
    XC330_T288,    
    XL430_W250,
    XXL430_W250,
    XC430_W150,  XC430_W240,
    XXC430_W250,
    XM430_W210,  XM430_W350,
    XM540_W150,  XM540_W270, 
    XH430_V210,  XH430_V350, XH430_W210, XH430_W350,
    XH540_V150,  XH540_V270, XH540_W150, XH540_W270,
    XD430_T210,  XD430_T350,
    XD540_T150,  XD540_T270,
    XW430_T200,  XW430_T333,
    XW540_T140,  XW540_T260,

    PRO_L42_10_S300_R,   
    PRO_L54_30_S400_R,   PRO_L54_30_S500_R,   PRO_L54_50_S290_R,   PRO_L54_50_S500_R,
    PRO_M42_10_S260_R,   PRO_M42_10_S260_RA,
    PRO_M54_40_S250_R,   PRO_M54_40_S250_RA,  PRO_M54_60_S250_R,   PRO_M54_60_S250_RA,
    PRO_H42_20_S300_R,   PRO_H42_20_S300_RA,
    PRO_H54_100_S500_R,  PRO_H54_100_S500_RA, PRO_H54_200_S500_R,  PRO_H54_200_S500_RA,

    PRO_M42P_010_S260_R, 
    PRO_M54P_040_S250_R, PRO_M54P_060_S250_R,
    PRO_H42P_020_S300_R, 
    PRO_H54P_100_S500_R, PRO_H54P_200_S500_R,

    YM070_210_M001_RH, YM070_210_B001_RH, YM070_210_R051_RH, YM070_210_R099_RH, YM070_210_A051_RH, YM070_210_A099_RH,
    YM080_230_M001_RH, YM080_230_B001_RH, YM080_230_R051_RH, YM080_230_R099_RH, YM080_230_A051_RH, YM080_230_A099_RH
};

const uint8_t model_number_table_count = sizeof(model_number_table)/sizeof(model_number_table[0]);

enum Functions{
  SET_ID,
  SET_BAUD_RATE,

  SET_PROTOCOL,
 
  SET_POSITION,
  GET_POSITION,

  SET_VELOCITY,
  GET_VELOCITY,

  SET_PWM,
  GET_PWM,

  SET_CURRENT,
  GET_CURRENT,

  LAST_DUMMY_FUNC = 0xFF
};

} //namespace DYNAMIXEL

using namespace DYNAMIXEL;

typedef struct ModelDependencyFuncItemAndRangeInfo{
  uint8_t func_idx; //enum Functions
  uint8_t item_idx; //enum ControlTableItem
  uint8_t unit_type; //enum ParamUnit
  int32_t min_value;
  int32_t max_value;
  float unit_value;
} ModelDependencyFuncItemAndRangeInfo_t;

typedef struct ItemAndRangeInfo{
  uint8_t item_idx; //enum ControlTableItem
  uint8_t unit_type; //enum ParamUnit
  int32_t min_value;
  int32_t max_value;
  float unit_value;
} ItemAndRangeInfo_t;

static ItemAndRangeInfo_t getModelDependencyFuncInfo(uint16_t model_num, uint8_t func_num);
static float f_map(float x, float in_min, float in_max, float out_min, float out_max);
static bool checkAndconvertWriteData(float in_data, int32_t &out_data, uint8_t unit, ItemAndRangeInfo_t &item_info);
static bool checkAndconvertReadData(int32_t in_data, float &out_data, uint8_t unit, ItemAndRangeInfo_t &item_info);


Dynamixel2Arduino::Dynamixel2Arduino(uint16_t packet_buf_size)
//: Master(2.0, packet_buf_size), model_number_idx_last_index_(0)
{
  memset(&model_number_idx_, 0xff, sizeof(model_number_idx_));
}

Dynamixel2Arduino::Dynamixel2Arduino(HardwareSerial& port, int dir_pin, uint16_t packet_buf_size)
//: Master(2.0, packet_buf_size), model_number_idx_last_index_(0)
{
  //p_dxl_port_ = new SerialPortHandler(port, dir_pin);
  //setPort(//p_dxl_port_);
  memset(&model_number_idx_, 0xff, sizeof(model_number_idx_));
}

// Refer to http://emanual.robotis.com/#protocol
bool Dynamixel2Arduino::setPortProtocolVersion(float version)
{
    uint8_t version_idx;

    if (version == 2.0)
    {
        version_idx = 2;
    }
    else if (version == 1.0)
    {
        version_idx = 1;
    }
    else
    {
        //last_lib_err_ = DXL_LIB_ERROR_INVAILD_PROTOCOL_VERSION;
        return false;
    }

    return true;
}

/* For Master configuration */
void Dynamixel2Arduino::begin(unsigned long baud)
{
  //p_dxl_port_ = (SerialPortHandler*)getPort();

  //if(//p_dxl_port_ == nullptr){
    //setLastLibErrCode(D2A_LIB_ERROR_NULLPTR_PORT_HANDLER);
    return;
  //}

  //p_dxl_port_->begin(baud);
}

unsigned long Dynamixel2Arduino::getPortBaud()
{
  //p_dxl_port_ = (SerialPortHandler*)getPort();

  //if(//p_dxl_port_ == nullptr){
    //setLastLibErrCode(D2A_LIB_ERROR_NULLPTR_PORT_HANDLER);
    return 0;
  //}

  //return //p_dxl_port_->getBaud();
}

bool Dynamixel2Arduino::scan()
{
  bool ret = true;

  ret = ping();

  return ret;
}

bool Dynamixel2Arduino::ping(uint8_t id)
{
  bool ret = false;
 
  //if (id != DXL_BROADCAST_ID){
  //  InfoFromPing_t recv_info;
  //  if(Master::ping(id, &recv_info, 1, 10) > 0){
  //    if(recv_info.id == id){
  //      if(getPortProtocolVersion() == 1.0){
  //        recv_info.model_number = getModelNumber(id);
  //      }
  //      ret = setModelNumber(id, recv_info.model_number);
  //    }
  //  }
  //}else{
  //  uint8_t recv_ids[254];
  //  uint8_t recv_cnt;

  //  //recv_cnt = Master::ping(DXL_BROADCAST_ID, recv_ids, sizeof(recv_ids), 3*253);

  //  if(recv_cnt > 0){
  //    for (uint8_t i=0; i<recv_cnt; i++){
  //      (void)setModelNumber(recv_ids[i], getModelNumber(id));
  //    }
  //    ret = true;
  //  }
  //}

  return ret;  
}

bool 
Dynamixel2Arduino::setModelNumber(uint8_t id, uint16_t model_number)
{
  bool ret = false;

  if(id <= 253){
    model_number_idx_[id] = getModelNumberIndex(model_number);
    ret = (model_number_idx_[id] != 0xFF) ? true:false;
    if(ret == false){
      //setLastLibErrCode(D2A_LIB_ERROR_UNKNOWN_MODEL_NUMBER);
    }
  }else{
    //setLastLibErrCode(DXL_LIB_ERROR_INVAILD_ID);
  }

  return ret;
}

uint16_t Dynamixel2Arduino::getModelNumber(uint8_t id)
{
  uint16_t model_num = 0xFFFF;

  //(void) read(id, COMMON_MODEL_NUMBER_ADDR, COMMON_MODEL_NUMBER_ADDR_LENGTH,
  // (uint8_t*)&model_num, sizeof(model_num), 20);

  return model_num;
}

bool Dynamixel2Arduino::setID(uint8_t id, uint8_t new_id)
{
  return writeControlTableItem(ControlTableItem::ID, id, new_id);
}

bool Dynamixel2Arduino::setProtocol(uint8_t id, float version)
{
  uint8_t ver_idx;

  if(version == 1.0){
    ver_idx = 1;
  }else if(version == 2.0){
    ver_idx = 2;
  }else{
    ////setLastLibErrCode(DXL_LIB_ERROR_INVAILD_PROTOCOL_VERSION);
    return false;
  }

  return writeControlTableItem(ControlTableItem::PROTOCOL_VERSION, id, ver_idx);
}

//TODO: Simplify the code by grouping model numbers.
bool Dynamixel2Arduino::setBaudrate(uint8_t id, uint32_t baudrate)
{
  uint16_t model_num = getModelNumberFromTable(id);
  uint8_t baud_idx = 0;

  switch(model_num)
  {
    case AX12A:
    case AX12W:
    case AX18A:
    case DX113:
    case DX116:
    case DX117:
    case RX10:
    case RX24F:
    case RX28:
    case RX64:
    case EX106:    
    case MX12W:
    case MX28:
    case MX64:
    case MX106:
      // baud_idx = round(2000000.0/(float)baudrate) - 1;
      // if(baud_idx > 254)
      //   return false;
      switch(baudrate)
      {
        case 9600:
          baud_idx = 207;
          break;
        case 57600:
          baud_idx = 34;
          break;
        case 115200:
          baud_idx = 16;
          break;
        case 1000000:
          baud_idx = 1;
          break;
        default:
          return false;                    
      }        
      break;

    case XL320:
      switch(baudrate)
      {
        case 9600:
          baud_idx = 0;
          break;
        case 57600:
          baud_idx = 1;
          break;
        case 115200:
          baud_idx = 2;
          break;
        case 1000000:
          baud_idx = 3;
          break;
        default:
          return false;                    
      }    
      break;
    case XC330_M288:
    case XC330_M181:
    case XC330_T288:
    case XC330_T181:
    case XL330_M288:
    case XL330_M077:
      switch(baudrate)
      {
        case 9600:
          baud_idx = 0;
          break;
        case 57600:
          baud_idx = 1;
          break;
        case 115200:
          baud_idx = 2;
          break;
        case 1000000:
          baud_idx = 3;
          break;
        case 2000000:
          baud_idx = 4;
          break;
        case 3000000:
          baud_idx = 5;
          break;
        case 4000000:
          baud_idx = 6;
          break;
        default:
          return false;                    
      }    
      break;

    case MX28_2:
    case MX64_2:
    case MX106_2:
    case XC430_W150:
    case XC430_W240:
    case XXC430_W250:
    case XL430_W250:
    case XXL430_W250:
    case XM430_W210:
    case XM430_W350:
    case XH430_V210:
    case XH430_V350:
    case XH430_W210:
    case XH430_W350:
    case XD430_T210:
    case XD430_T350:
    case XM540_W150:
    case XM540_W270:
    case XH540_W150:
    case XH540_W270:
    case XH540_V150:
    case XH540_V270:
    case XD540_T150:
    case XD540_T270:
    case XW430_T200:
    case XW430_T333:
    case XW540_T140:
    case XW540_T260:    
      switch(baudrate)
      {
        case 9600:
          baud_idx = 0;
          break;
        case 57600:
          baud_idx = 1;
          break;
        case 115200:
          baud_idx = 2;
          break;
        case 1000000:
          baud_idx = 3;
          break;
        case 2000000:
          baud_idx = 4;
          break;
        case 3000000:
          baud_idx = 5;
          break;
        case 4000000:
          baud_idx = 6;
          break;
        case 4500000:
          baud_idx = 7;
          break;    
        default:
          return false;          
      }
      break;

    // case PRO_L42_10_S300_R:
    // case PRO_L54_30_S400_R:
    // case PRO_L54_30_S500_R:
    // case PRO_L54_50_S290_R:
    // case PRO_L54_50_S500_R:
    case PRO_M42_10_S260_R:
    case PRO_M54_40_S250_R:
    case PRO_M54_60_S250_R:
    case PRO_H42_20_S300_R:
    case PRO_H54_100_S500_R:
    case PRO_H54_200_S500_R:
      switch(baudrate)
      {
        case 9600:
          baud_idx = 0;
          break;
        case 57600:
          baud_idx = 1;
          break;
        case 115200:
          baud_idx = 2;
          break;
        case 1000000:
          baud_idx = 3;
          break;
        case 2000000:
          baud_idx = 4;
          break;
        case 3000000:
          baud_idx = 5;
          break;
        case 4000000:
          baud_idx = 6;
          break;
        case 4500000:
          baud_idx = 7;
          break;
        case 10500000:
          baud_idx = 8;
          break;
        default:
          return false;          
      }
      break;

    case PRO_M42_10_S260_RA:
    case PRO_M54_40_S250_RA:
    case PRO_M54_60_S250_RA:
    case PRO_H42_20_S300_RA:
    case PRO_H54_100_S500_RA:
    case PRO_H54_200_S500_RA:
    case PRO_H42P_020_S300_R:
    case PRO_H54P_100_S500_R:
    case PRO_H54P_200_S500_R:
    case PRO_M42P_010_S260_R:
    case PRO_M54P_040_S250_R:
    case PRO_M54P_060_S250_R:
      switch(baudrate)
      {
        case 9600:
          baud_idx = 0;
          break;
        case 57600:
          baud_idx = 1;
          break;
        case 115200:
          baud_idx = 2;
          break;
        case 1000000:
          baud_idx = 3;
          break;
        case 2000000:
          baud_idx = 4;
          break;
        case 3000000:
          baud_idx = 5;
          break;
        case 4000000:
          baud_idx = 6;
          break;
        case 4500000:
          baud_idx = 7;
          break;
        case 6000000:
          baud_idx = 8;
          break;          
        case 10500000:
          baud_idx = 9;
          break;          
        default:
          return false;          
      }                
      break;

    case YM070_210_M001_RH:
    case YM070_210_B001_RH:
    case YM070_210_R051_RH:
    case YM070_210_R099_RH:
    case YM070_210_A051_RH:
    case YM070_210_A099_RH:
    case YM080_230_M001_RH:
    case YM080_230_B001_RH:
    case YM080_230_R051_RH:
    case YM080_230_R099_RH:
    case YM080_230_A051_RH:
    case YM080_230_A099_RH:
      switch(baudrate)
      {
        case 9600:
          baud_idx = 0;
          break;
        case 57600:
          baud_idx = 1;
          break;
        case 115200:
          baud_idx = 2;
          break;
        case 1000000:
          baud_idx = 3;
          break;
        case 2000000:
          baud_idx = 4;
          break;
        case 3000000:
          baud_idx = 5;
          break;
        case 4000000:
          baud_idx = 6;
          break;
        case 4500000:
          baud_idx = 7;
          break;
        case 6000000:
          baud_idx = 8;
          break;          
        case 10500000:
          baud_idx = 9;
          break;          
        default:
          return false;          
      }                
      break;

    default:
      return false;
      break;
  }

  return writeControlTableItem(ControlTableItem::BAUD_RATE, id, baud_idx);
}


/* Commands for Slave */
bool Dynamixel2Arduino::torqueOn(uint8_t id)
{
  return setTorqueEnable(id, true);
}

bool Dynamixel2Arduino::torqueOff(uint8_t id)
{
  return setTorqueEnable(id, false);
}

bool Dynamixel2Arduino::setTorqueEnable(uint8_t id, bool enable)
{
  return writeControlTableItem(ControlTableItem::TORQUE_ENABLE, id, enable);
}

bool Dynamixel2Arduino::ledOn(uint8_t id)
{
  return setLedState(id, true);
}

bool Dynamixel2Arduino::ledOff(uint8_t id)
{
  return setLedState(id, false);
}

bool Dynamixel2Arduino::setLedState(uint8_t id, bool state)
{
  bool ret = false;
  uint16_t model_num = getModelNumberFromTable(id);

  switch(model_num)
  {
    // case PRO_L42_10_S300_R:
    // case PRO_L54_30_S400_R:
    // case PRO_L54_30_S500_R:
    // case PRO_L54_50_S290_R:
    // case PRO_L54_50_S500_R:
    case PRO_M42_10_S260_R:
    case PRO_M54_40_S250_R:
    case PRO_M54_60_S250_R:
    case PRO_H42_20_S300_R:
    case PRO_H54_100_S500_R:
    case PRO_H54_200_S500_R:
    case PRO_M42_10_S260_RA:
    case PRO_M54_40_S250_RA:
    case PRO_M54_60_S250_RA:
    case PRO_H42_20_S300_RA:
    case PRO_H54_100_S500_RA:
    case PRO_H54_200_S500_RA:
    case PRO_H42P_020_S300_R:
    case PRO_H54P_100_S500_R:
    case PRO_H54P_200_S500_R:
    case PRO_M42P_010_S260_R:
    case PRO_M54P_040_S250_R:
    case PRO_M54P_060_S250_R:
          if (state == false) {
              writeControlTableItem(ControlTableItem::LED_GREEN, id, state);
              writeControlTableItem(ControlTableItem::LED_BLUE, id, state);
          }
      ret = writeControlTableItem(ControlTableItem::LED_RED, id, state);
      break;

    default:
      ret = writeControlTableItem(ControlTableItem::LED, id, state);
      break;
  }

  return ret;
}


//TODO: Simplify the code by grouping model numbers.
bool Dynamixel2Arduino::setOperatingMode(uint8_t id, uint8_t mode)
{
  bool ret = false;
  uint16_t model_num = getModelNumberFromTable(id);

  switch(model_num)
  {
    case AX12A:
    case AX12W:
    case AX18A:
    case DX113:
    case DX116:
    case DX117:
    case RX10:
    case RX24F:
    case RX28:
    case RX64:
      if(mode == OP_POSITION){
        if(writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
          ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 1023);
      }else if(mode == OP_VELOCITY){
        if(writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
          ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 0);
      }
      break;

    case EX106:
      if(mode == OP_POSITION){
        if(writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
          ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 4095);
      }else if(mode == OP_VELOCITY){
        if(writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
          ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 0);
      }
      break;

    case XL320:
      if(mode == OP_POSITION){
        ret = writeControlTableItem(ControlTableItem::CONTROL_MODE, id, 2);
      }else if(mode == OP_VELOCITY){
        ret = writeControlTableItem(ControlTableItem::CONTROL_MODE, id, 1);
      }
      break;
    
    case MX12W:
    case MX28:
      if(mode == OP_POSITION){
        if(writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
          ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 4095);
      }else if(mode == OP_VELOCITY){
        if(writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
          ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 0);
      }else if(mode == OP_EXTENDED_POSITION){
        if(writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 4095))
          ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 4095);
      }
      break;   

    case MX64:
    case MX106:
      if(mode == OP_POSITION){
        if(writeControlTableItem(ControlTableItem::TORQUE_CTRL_MODE_ENABLE, id, 0)
          || writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
          ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 4095);
      }else if(mode == OP_VELOCITY){
        if(writeControlTableItem(ControlTableItem::TORQUE_CTRL_MODE_ENABLE, id, 0)
          || writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
          ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 0);
      }else if(mode == OP_EXTENDED_POSITION){
        if(writeControlTableItem(ControlTableItem::TORQUE_CTRL_MODE_ENABLE, id, 0)
          || writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 4095))
          ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 4095);
      }else if(mode == OP_CURRENT){
        ret = writeControlTableItem(ControlTableItem::TORQUE_CTRL_MODE_ENABLE, id, 1);
      }
      break;

    case MX28_2:
    case XC430_W150:
    case XC430_W240:
    case XXC430_W250:
    case XL430_W250:
    case XXL430_W250:
      if(mode == OP_POSITION){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 3);
      }else if(mode == OP_VELOCITY){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 1);
      }else if(mode == OP_EXTENDED_POSITION){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 4);
      }else if(mode == OP_PWM){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 16);
      }
      break;

    case MX64_2:
    case MX106_2:
    case XL330_M288:
    case XL330_M077:
    case XC330_M288:
    case XC330_M181:    
    case XC330_T181:
    case XC330_T288:    
    case XM430_W210:
    case XM430_W350:
    case XH430_V210:
    case XH430_V350:
    case XH430_W210:
    case XH430_W350:
    case XD430_T210:
    case XD430_T350:
    case XM540_W150:
    case XM540_W270:
    case XH540_W150:
    case XH540_W270:
    case XH540_V150:
    case XH540_V270:
    case XD540_T150:
    case XD540_T270:
    case XW430_T200:
    case XW430_T333:
    case XW540_T140:
    case XW540_T260:    
      if(mode == OP_POSITION){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 3);
      }else if(mode == OP_VELOCITY){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 1);
      }else if(mode == OP_EXTENDED_POSITION){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 4);
      }else if(mode == OP_CURRENT){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 0);
      }else if(mode == OP_PWM){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 16);
      }else if(mode == OP_CURRENT_BASED_POSITION){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 5);
      }
      break;            

    // case PRO_L42_10_S300_R:
    // case PRO_L54_30_S400_R:
    // case PRO_L54_30_S500_R:
    // case PRO_L54_50_S290_R:
    // case PRO_L54_50_S500_R:
    case PRO_M42_10_S260_R:
    case PRO_M54_40_S250_R:
    case PRO_M54_60_S250_R:
    case PRO_H42_20_S300_R:
    case PRO_H54_100_S500_R:
    case PRO_H54_200_S500_R:
      if(mode == OP_POSITION){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 3);
      }else if(mode == OP_VELOCITY){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 1);
      }else if(mode == OP_EXTENDED_POSITION){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 4);
      }else if(mode == OP_CURRENT){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 0);
      }
      break;

    case PRO_M42_10_S260_RA:
    case PRO_M54_40_S250_RA:
    case PRO_M54_60_S250_RA:
    case PRO_H42_20_S300_RA:
    case PRO_H54_100_S500_RA:
    case PRO_H54_200_S500_RA:
    case PRO_H42P_020_S300_R:
    case PRO_H54P_100_S500_R:
    case PRO_H54P_200_S500_R:
    case PRO_M42P_010_S260_R:
    case PRO_M54P_040_S250_R:
    case PRO_M54P_060_S250_R:
      if(mode == OP_POSITION){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 3);
      }else if(mode == OP_VELOCITY){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 1);
      }else if(mode == OP_EXTENDED_POSITION){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 4);
      }else if(mode == OP_CURRENT){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 0);
      }else if(mode == OP_PWM){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 16);
      }
      break;

    case YM070_210_M001_RH:
    case YM070_210_B001_RH:
    case YM070_210_R051_RH:
    case YM070_210_R099_RH:
    case YM070_210_A051_RH:
    case YM070_210_A099_RH:
    case YM080_230_M001_RH:
    case YM080_230_B001_RH:
    case YM080_230_R051_RH:
    case YM080_230_R099_RH:
    case YM080_230_A051_RH:
    case YM080_230_A099_RH:
      if(mode == OP_POSITION){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 3);
      }else if(mode == OP_VELOCITY){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 1);
      }else if(mode == OP_CURRENT){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 0);
      }
      break;

    default:
      break;
  }

  return ret;
}

bool Dynamixel2Arduino::setGoalPosition(uint8_t id, float value, uint8_t unit)
{
  if(unit != UNIT_RAW && unit != UNIT_DEGREE)
    return false;

  return true;//return writeForRangeDependencyFunc(SET_POSITION, id, value, unit);
}

float Dynamixel2Arduino::getPresentPosition(uint8_t id, uint8_t unit)
{
  if(unit != UNIT_RAW && unit != UNIT_DEGREE)
    return 0.0;

  return true; // return readForRangeDependencyFunc(GET_POSITION, id, unit);
}

bool Dynamixel2Arduino::setGoalVelocity(uint8_t id, float value, uint8_t unit)
{
  if(unit != UNIT_RAW && unit != UNIT_PERCENT && unit != UNIT_RPM)
    return false;

  return true; // return writeForRangeDependencyFunc(SET_VELOCITY, id, value, unit);
}

float Dynamixel2Arduino::getPresentVelocity(uint8_t id, uint8_t unit)
{
  if(unit != UNIT_RAW && unit != UNIT_PERCENT && unit != UNIT_RPM)
    return 0.0;

  return true; // return readForRangeDependencyFunc(GET_VELOCITY, id, unit);
}

bool Dynamixel2Arduino::setGoalPWM(uint8_t id, float value, uint8_t unit)
{
  if(unit != UNIT_RAW && unit != UNIT_PERCENT)
    return false;

  return true; // return writeForRangeDependencyFunc(SET_PWM, id, value, unit);
}

float Dynamixel2Arduino::getPresentPWM(uint8_t id, uint8_t unit)
{
  if(unit != UNIT_RAW && unit != UNIT_PERCENT)
    return 0.0;

  return true; // return readForRangeDependencyFunc(GET_PWM, id, unit);
}

bool Dynamixel2Arduino::setGoalCurrent(uint8_t id, float value, uint8_t unit)
{
  if(unit != UNIT_RAW && unit != UNIT_PERCENT && unit != UNIT_MILLI_AMPERE)
    return false;

  return true; // return writeForRangeDependencyFunc(SET_CURRENT, id, value, unit);
}

float Dynamixel2Arduino::getPresentCurrent(uint8_t id, uint8_t unit)
{
  if(unit != UNIT_RAW && unit != UNIT_PERCENT && unit != UNIT_MILLI_AMPERE)
    return 0.0;

  return true; // return readForRangeDependencyFunc(GET_CURRENT, id, unit);
}

bool Dynamixel2Arduino::getTorqueEnableStat(uint8_t id)
{
  bool ret = false;

  if(readControlTableItem(ControlTableItem::TORQUE_ENABLE, id) == DXL_TORQUE_ON){
    ret = true;
  }else{
    ret = false;
  }

  return ret;
}

int32_t Dynamixel2Arduino::readControlTableItem(uint8_t item_idx, uint8_t id, uint32_t timeout)
{
  int32_t ret = 0;
  uint16_t model_num = getModelNumberFromTable(id);

  // To use the command function without ping() or model addition.
  if(model_num == UNREGISTERED_MODEL){
    if(setModelNumber(id, getModelNumber(id)) == true){
      model_num = getModelNumberFromTable(id);
    }
  }

  if(model_num != UNREGISTERED_MODEL){
    ret = readControlTableItem(model_num, item_idx, id, timeout);
  }else{
    //setLastLibErrCode(D2A_LIB_ERROR_UNKNOWN_MODEL_NUMBER);
  }

  return ret;
}

bool Dynamixel2Arduino::writeControlTableItem(uint8_t item_idx, uint8_t id, int32_t data, uint32_t timeout)
{
  bool ret = false;
  uint16_t model_num = getModelNumberFromTable(id);
  
  // To use the command function without ping() or model addition.
  if(model_num == UNREGISTERED_MODEL){
    if(setModelNumber(id, getModelNumber(id)) == true){
      model_num = getModelNumberFromTable(id);
    }
  }

  if(model_num != UNREGISTERED_MODEL){
    ret = writeControlTableItem(model_num, item_idx, id, data, timeout);
  }else{
    //setLastLibErrCode(D2A_LIB_ERROR_UNKNOWN_MODEL_NUMBER);
  }

  return ret;
}




/* Private Member Function */

int32_t Dynamixel2Arduino::readControlTableItem(uint16_t model_num, uint8_t item_idx, uint8_t id, uint32_t timeout)
{
  int32_t recv_len, ret = 0;
  ControlTableItemInfo_t item_info;

  //p_dxl_port_ = (SerialPortHandler*)getPort();
  //if(//p_dxl_port_ == nullptr){
    //setLastLibErrCode(D2A_LIB_ERROR_NULLPTR_PORT_HANDLER);
    return 0;
  //}

  //item_info = getControlTableItemInfo(model_num, item_idx);

  //if(item_info.addr_length > 0)
  //{
  //  recv_len = read(id, item_info.addr, item_info.addr_length, (uint8_t*)&ret, sizeof(ret), timeout);

  //  if(recv_len == 1){
  //    int8_t t_data = (int8_t)ret;
  //    ret = (int32_t)t_data;
  //  }else if(recv_len == 2){
  //    int16_t t_data = (int16_t)ret;
  //    ret = (int32_t)t_data;
  //  }
  //}

  //return ret;
}

bool Dynamixel2Arduino::writeControlTableItem(uint16_t model_num, uint8_t item_idx, uint8_t id, int32_t data, uint32_t timeout)
{
  bool ret = false;
  ControlTableItemInfo_t item_info;
  //
  ////p_dxl_port_ = (SerialPortHandler*)getPort();
  //if(//p_dxl_port_ == nullptr){
  //  //setLastLibErrCode(D2A_LIB_ERROR_NULLPTR_PORT_HANDLER);
  //  return false;
  //}

  //item_info = getControlTableItemInfo(model_num, item_idx);
  //if(item_info.addr_length > 0){
  //  ret = write(id, item_info.addr, (uint8_t*)&data, item_info.addr_length, timeout);
  //}

  return ret;
}

uint8_t Dynamixel2Arduino::getModelNumberIndex(uint16_t model_num)
{
  uint8_t i, ret = 0xFF;

  //// quick shortcut
  //if(model_num == model_number_idx_[model_number_idx_last_index_]){
  //  ret = model_number_idx_last_index_;
  //}else{
  //  for(i=0; i<model_number_table_count; i++)
  //  {
  //    if(model_num == pgm_read_word(&model_number_table[i])){
  //      model_number_idx_last_index_ = i;
  //      ret = i;
  //      break;
  //    }
  //  }
  //}

  return ret;
}


uint16_t Dynamixel2Arduino::getModelNumberFromTable(uint8_t id)
{
  uint8_t idx;
  uint16_t model_num=0;

  //if(id > 254){
  //  //setLastLibErrCode(DXL_LIB_ERROR_INVAILD_ID);
  //  return UNREGISTERED_MODEL;
  //}

  //idx = model_number_idx_[id];
  //model_num = (idx < model_number_table_count) ? pgm_read_word(&model_number_table[idx]) : UNREGISTERED_MODEL;

  return model_num;
}