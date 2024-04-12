#ifndef QUTEE_QUTEE_HPP_
#define QUTEE_QUTEE_HPP_

#define NN_INPUT_SIZE 18 // 6+12
#define NN_OUTPUT_SIZE 12
#define NB_STEPS floor(CONFIG_EPISODE_DURATION * CONFIG_CONTROL_FREQUENCY)
#define PERIOD floor(1000000 / CONFIG_CONTROL_FREQUENCY)

#undef HIGH // undef macro from adruino lib
#undef LOW // undef macro from arduino lib
#include "NNQuteeController.hpp"
#undef HIGH // undef macro from adruino lib
#undef LOW // undef macro from arduino lib

#include "nvs_flash.h"
#include "nvs.h"
#define STORAGE_NAMESPACE "storage"

#include "arduino.h"
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include "Adafruit_MAX1704X.h"

#include "Dynamixel2Arduino.h"
#include "utility/port_handler.h"
#include "QuteeDxlPortHandler.hpp"
//#include "QuteeController.hpp"

//This namespace is required to use Control table item names
using namespace ControlTableItem;
 

class Qutee
{
public:
  typedef NNQuteeController<NN_INPUT_SIZE, CONFIG_NB_HIDDEN_LAYERS, CONFIG_NB_NEURONS_PER_LAYER,NN_OUTPUT_SIZE> Policy_t;
  typedef Eigen::TensorFixedSize< float , Eigen::Sizes<NN_INPUT_SIZE, 1> > State_t;
  typedef Eigen::TensorFixedSize< float , Eigen::Sizes<NN_OUTPUT_SIZE, 1> > Actions_t;
  typedef std::vector<State_t,Eigen::aligned_allocator<State_t> > List_State_t;
  typedef std::vector<Actions_t,Eigen::aligned_allocator<Actions_t> > List_Actions_t;
  
  const float DEG_2_RAD = 0.01745329251f; //trig functions require radians, BNO055 outputs degrees
  
  Qutee():
    _states(NB_STEPS),
    _actions(NB_STEPS),
    _dxl_port(UART_NUM_1),
    _tft(TFT_CS, TFT_DC, TFT_RST),
    _bno(55, 0x28)
  {        
    
  }
  void run_episode();
  void init();
  void scan();
  void control_step(State_t& state_to_fill, Actions_t& actions_to_fill);   
  void tft_update_data_screen(const State_t& state, const Actions_t& actions, float freq);
  Policy_t& get_policy(){return _policy;}
  const List_Actions_t& get_list_actions(){return _actions;}
  const List_State_t& get_list_states(){return _states;}
  void calibration();
  void go_to_neutral_pose();
  float battery_voltage();
  
  void checkup();
  void menu();
private:
  void send_actions(const Actions_t& actions);
  void get_motor_positions(State_t& state_ref, size_t offset);
  void get_state(State_t& state_ref);
  void init_motors();
  void set_PID_gains();
  void init_imu();
  void init_tft();
  void displaySensorStatus(void);
  void tft_init_data_screen();
  void tft_load_screen();
  void tft_init_checkup_screen();
  void tft_update_checkup_screen(const float* err);
  void init_NVS();
  void manage_imu_calibration_nvs(uint8_t* calibration_data, size_t size, bool write);
  void load_imu_calibration();
  void print_state();
  // ----- Attributes ---- //
  List_State_t _states;
  List_Actions_t _actions;
  
  Policy_t _policy;
  State_t _state;
  
  static const int DXL_IDs[];
  static const uint8_t DXL_ID_CNT = 12;
  QuteeDxlPortHandler _dxl_port;
  Dynamixel2Arduino _dxl;
  Adafruit_ST7789 _tft;
  Adafruit_BNO055 _bno;
  Adafruit_MAX17048 _maxlipo;
  
  
  
  // Starting address of the Data to read; Present Position = 132
  static const uint16_t SR_START_ADDR = 132;
  // Length of the Data to read; Length of Position data of X series is 4 byte
  static const uint16_t SR_ADDR_LEN = 4;
  // Starting address of the Data to write; Goal Position = 116
  static const uint16_t SW_START_ADDR = 116;
  // Length of the Data to write; Length of Position data of X series is 4 byte
  static const uint16_t SW_ADDR_LEN = 4;
  typedef struct sr_data{
    int32_t present_position;
  } __attribute__((packed)) sr_data_t;
  typedef struct sw_data{
    int32_t goal_position;
  } __attribute__((packed)) sw_data_t;
  sr_data_t _sr_data[DXL_ID_CNT];
  DYNAMIXEL::InfoSyncReadInst_t _sr_infos;
  DYNAMIXEL::XELInfoSyncRead_t _info_xels_sr[DXL_ID_CNT];
  sw_data_t _sw_data[DXL_ID_CNT];
  DYNAMIXEL::InfoSyncWriteInst_t _sw_infos;
  DYNAMIXEL::XELInfoSyncWrite_t _info_xels_sw[DXL_ID_CNT];
  static const uint16_t _user_pkt_buf_cap = 128;
  uint8_t _user_pkt_buf[_user_pkt_buf_cap];
};


#endif
