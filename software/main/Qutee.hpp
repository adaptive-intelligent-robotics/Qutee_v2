#ifndef QUTEE_QUTEE_HPP_
#define QUTEE_QUTEE_HPP_

#define NN_INPUT_SIZE 6+12
#define NN_OUTPUT_SIZE 12
#undef HIGH // undef macro from adruino lib
#undef LOW // undef macro from arduino lib
#include "NNQuteeController.hpp"
#undef HIGH // undef macro from adruino lib
#undef LOW // undef macro from arduino lib

#include "arduino.h"
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789


#include "Dynamixel2Arduino.h"
#include "utility/port_handler.h"
#include "QuteeDxlPortHandler.hpp"
//#include "QuteeController.hpp"

//This namespace is required to use Control table item names
using namespace ControlTableItem;
 /*

#include "arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <esp_log.h>

double xPos = 0, yPos = 0, headingVel = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 500; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample

//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
*/

/*void setup(void)
{
    Wire.setPins(GPIO_NUM_3,GPIO_NUM_4);
    Wire.begin();
    gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_7, 1);
    delay(1000);
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!
      ESP_LOGI("IMU","HELLO WORLD222");
  if (!bno.begin())
  {
        ESP_LOGI("IMU","No BNO055 detected");
    while (1);
  }
 ESP_LOGI("IMU","DONE WORLD");

  delay(1000);
  bno.setMode(OPERATION_MODE_NDOF);
  loop();
}

void loop(void)
{
  //
   //ESP_LOGI("IMU","enter loo ");
   
  unsigned long tStart = micros();
  sensors_event_t orientationData , linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;

  // velocity of sensor in the direction it's facing
  headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

  if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    //enough iterations have passed that we can print the latest data
    ESP_LOGI("IMU","Heading: %f, Position:  %f, %f  Speed:  %f, ",orientationData.orientation.x, xPos , yPos, headingVel);
   
    
    printCount = 0;
  }
  else {
    printCount = printCount + 1;
  }



  while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000))
  {
    //poll until the next sample is ready
  }
}



*/



class Qutee
{
  public:
    typedef NNQuteeController<NN_INPUT_SIZE, CONFIG_NB_HIDDEN_LAYERS, CONFIG_NB_NEURONS_PER_LAYER,NN_OUTPUT_SIZE> Policy_t;
    typedef Eigen::TensorFixedSize< float , Eigen::Sizes<NN_INPUT_SIZE, 1> > State_t;
    typedef Eigen::TensorFixedSize< float , Eigen::Sizes<NN_OUTPUT_SIZE, 1> > Actions_t;
    const float DEG_2_RAD = 0.01745329251f; //trig functions require radians, BNO055 outputs degrees

    Qutee():
    _dxl_port(UART_NUM_1),
    _tft(TFT_CS, TFT_DC, TFT_RST),
    _bno(55, 0x28)
    {        
          
    }
    void control_loop(float duration_s);
    void init();
    void scan();
    void control_step(State_t& state_to_fill, Actions_t& actions_to_fill);   
    void tft_update_data_screen(const State_t& state, const Actions_t& actions, float freq);
    Policy_t& get_policy(){return _policy;}

    
  private:
    void send_actions(const Actions_t& actions);
    void get_motor_positions(State_t& state_ref, size_t offset);
    void get_state(State_t& state_ref);
    void init_motors();
    void init_imu();
    void init_tft();
    void displaySensorStatus(void);
    void tft_init_data_screen();
    void tft_load_screen();
// ----- Attributes ---- //
    static const int DXL_IDs[];
    static const uint8_t DXL_ID_CNT = 12;
    QuteeDxlPortHandler _dxl_port;
    Dynamixel2Arduino _dxl;
    Adafruit_ST7789 _tft;
    Adafruit_BNO055 _bno;

    Policy_t _policy;
    State_t _state;
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