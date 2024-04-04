#include "Qutee.hpp"
#include <inttypes.h>


#define VALUE(string) #string
#define TO_LITERAL(string) VALUE(string)

const int Qutee::DXL_IDs[] = {11, 12, 13, 21, 22, 23,  31, 32, 33, 41, 42, 43};

void Qutee::init(){
  
  //Wire.setPins(GPIO_NUM_3,GPIO_NUM_4);
  //Wire.begin();
  init_tft();
  tft_load_screen();
  _dxl.setPort(_dxl_port);
  int32_t baud = 3000000;
  int8_t  protocol = 2;       
  _dxl.setPortProtocolVersion((float) protocol);
  ESP_LOGI("DXL: ","PROTOCOL %i", protocol);
  ESP_LOGI("DXL: ","BAUDRATE %ld\n", baud);
  _dxl.begin(baud);
  
  // Fill the members of structure to syncRead using external user packet buffer
  _sr_infos.packet.p_buf = _user_pkt_buf;
  _sr_infos.packet.buf_capacity = _user_pkt_buf_cap;
  _sr_infos.packet.is_completed = false;
  _sr_infos.addr = SR_START_ADDR;
  _sr_infos.addr_length = SR_ADDR_LEN;
  _sr_infos.p_xels = _info_xels_sr;
  _sr_infos.xel_count = 0;  
  
  for(int i = 0; i < DXL_ID_CNT; i++){
    _info_xels_sr[i].id = DXL_IDs[i];
    _info_xels_sr[i].p_recv_buf = (uint8_t*)&_sr_data[i];
    _sr_infos.xel_count++;
  }
  _sr_infos.is_info_changed = true;
  
  // Fill the members of structure to syncWrite using internal packet buffer
  _sw_infos.packet.p_buf = nullptr;
  _sw_infos.packet.is_completed = false;
  _sw_infos.addr = SW_START_ADDR;
  _sw_infos.addr_length = SW_ADDR_LEN;
  _sw_infos.p_xels = _info_xels_sw;
  _sw_infos.xel_count = 0;
  
  for(int i = 0; i < DXL_ID_CNT; i++){
    _info_xels_sw[i].id = DXL_IDs[i];
    _info_xels_sw[i].p_data = (uint8_t*)&_sw_data[i].goal_position;
    _sw_infos.xel_count++;
  }
  _sw_infos.is_info_changed = true;
  
  init_imu();

  // init battery monitor
  if (!this->_maxlipo.begin()) {
    ESP_LOGI("BATTERY: ","Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!\n");
    while (1) 
        std::this_thread::sleep_for(std::chrono::microseconds(100000));
  }
  
  ESP_LOGI("BATTERY: ","Found MAX17048 with Chip ID: %d \n",  this->_maxlipo.getChipID());
  this->_maxlipo.reset();
  
  init_motors();
  scan();
  tft_init_data_screen();
  go_to_neutral_pose();
}

void Qutee::init_imu(){

  Wire.setPins(GPIO_NUM_3,GPIO_NUM_4);
  Wire.begin();

  // POWER UP I2C
  gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_7, 1);
  ESP_LOGI("IMU","START IMU PROCESS.");
  if (!this->_bno.begin())
    {
      ESP_LOGI("IMU","No BNO055 detected");
      while (1);
    }
  ESP_LOGI("IMU","Init imu Done");
  delay(1000);
  ESP_LOGI("IMU","Setting NDOF mode");
  this->load_imu_calibration();
  this->_bno.setMode(OPERATION_MODE_IMUPLUS);
  displaySensorStatus();
  ESP_LOGI("IMU","IMU DONE.");
    
}

void Qutee::init_tft(){
  // turn on backlite 
  gpio_set_direction((gpio_num_t)TFT_BACKLITE, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)TFT_BACKLITE, 1);
  // turn on the TFT / I2C power supply
  gpio_set_direction((gpio_num_t)TFT_I2C_POWER, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)TFT_I2C_POWER, 1);
  
  delay(10);
  this->_tft.init(135, 240);
  this->_tft.setRotation(3);

}

void Qutee::tft_load_screen() {
  this->_tft.setTextWrap(false);
  this->_tft.fillScreen(ST77XX_BLACK);
  this->_tft.setCursor(0, 30);
  this->_tft.setTextColor(ST77XX_GREEN);
  this->_tft.setTextSize(2);
  this->_tft.println("Hi! My name is");
  this->_tft.println(TO_LITERAL(CONFIG_QUTEE_NAME));
  this->_tft.setTextColor(ST77XX_RED);
  this->_tft.println("Loading!");
  delay(1000);
}

void Qutee::tft_init_data_screen() {
  int lo = 10; // line offset
  int io = 32; // item offset
  this->_tft.fillScreen(ST77XX_BLACK);
  this->_tft.setCursor(0, 0);
  this->_tft.setTextColor(ST77XX_GREEN,ST77XX_BLACK);
  this->_tft.setTextSize(1);
  this->_tft.setCursor(160, 0);   this->_tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK);   this->_tft.print("  Freq: ");
  this->_tft.setCursor(0, 0);    this->_tft.setTextColor(ST77XX_RED,ST77XX_BLACK);    this->_tft.println("State: ");

  this->_tft.setTextColor(ST77XX_BLUE,ST77XX_BLACK); 
  this->_tft.setCursor(0*io, 1*lo); this->_tft.print("ox: ");  this->_tft.setCursor(2*io, 1*lo);  this->_tft.print(" oy: "); this->_tft.setCursor(4*io, 1*lo);  this->_tft.print(" oz: "); 
  this->_tft.setCursor(0*io, 2*lo); this->_tft.print("ax: ");  this->_tft.setCursor(2*io, 2*lo);  this->_tft.print(" ay: "); this->_tft.setCursor(4*io, 2*lo);  this->_tft.print(" az: "); 
  this->_tft.setCursor(0*io, 3*lo); this->_tft.print("11: ");  this->_tft.setCursor(2*io, 3*lo);  this->_tft.print(" 12: "); this->_tft.setCursor(4*io, 3*lo);  this->_tft.print(" 13: "); 
  this->_tft.setCursor(0*io, 4*lo); this->_tft.print("21: ");  this->_tft.setCursor(2*io, 4*lo);  this->_tft.print(" 22: "); this->_tft.setCursor(4*io, 4*lo);  this->_tft.print(" 23: "); 
  this->_tft.setCursor(0*io, 5*lo); this->_tft.print("31: ");  this->_tft.setCursor(2*io, 5*lo);  this->_tft.print(" 32: "); this->_tft.setCursor(4*io, 5*lo);  this->_tft.print(" 33: "); 
  this->_tft.setCursor(0*io, 6*lo); this->_tft.print("41: ");  this->_tft.setCursor(2*io, 6*lo);  this->_tft.print(" 42: "); this->_tft.setCursor(4*io, 6*lo);  this->_tft.print(" 43: "); 

  this->_tft.setCursor( 0, 8*lo); this->_tft.setTextColor(ST77XX_RED,ST77XX_BLACK); this->_tft.println("Actions: ");

  this->_tft.setTextColor(ST77XX_BLUE,ST77XX_BLACK); 
  this->_tft.setCursor(0*io, 9*lo);  this->_tft.print("11: ");  this->_tft.setCursor(2*io, 9*lo);   this->_tft.print(" 12: "); this->_tft.setCursor(4*io, 9*lo);   this->_tft.print(" 13: "); 
  this->_tft.setCursor(0*io, 10*lo); this->_tft.print("21: ");  this->_tft.setCursor(2*io, 10*lo);  this->_tft.print(" 22: "); this->_tft.setCursor(4*io, 10*lo);  this->_tft.print(" 23: "); 
  this->_tft.setCursor(0*io, 11*lo); this->_tft.print("31: ");  this->_tft.setCursor(2*io, 11*lo);  this->_tft.print(" 32: "); this->_tft.setCursor(4*io, 11*lo);  this->_tft.print(" 33: "); 
  this->_tft.setCursor(0*io, 12*lo); this->_tft.print("41: ");  this->_tft.setCursor(2*io, 12*lo);  this->_tft.print(" 42: "); this->_tft.setCursor(4*io, 12*lo);  this->_tft.print(" 43: "); 

}

void Qutee::tft_update_data_screen(const State_t& state, const Actions_t& actions, float freq) {
  int lo = 10; // line offset
  int io = 32; // item offset
  char buf[6]; // 5 characters + NUL
  this->_tft.setTextSize(1);
  this->_tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK); 
  sprintf(buf, "%-2.2f",freq);
  this->_tft.setCursor(200 , 0);   this->_tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK); this->_tft.print(buf);

  this->_tft.setTextSize(1);
  // State
  for(size_t i=0; i < NN_INPUT_SIZE; i++){
    sprintf(buf, "%- 1.2f",state(i,0) );
    this->_tft.setCursor( ((i%3) *2 +1)*io, (i/3 + 1)*lo); this->_tft.print(buf); 
  }
   // Actions
  for(size_t i=0; i < NN_OUTPUT_SIZE; i++){
    sprintf(buf, "%- 1.2f",actions(i,0) );
    this->_tft.setCursor( ((i%3) *2 +1)*io, (i/3 + 9)*lo); this->_tft.print(buf); 
  }
 
}

// function to initialise the non-volatile storage of the qutee. Mostly used to store calibration data. 
void Qutee::init_NVS(){ 
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
}

void Qutee::manage_imu_calibration_nvs(uint8_t* calibration_data, size_t size, bool write = false){
  this->init_NVS();  
  ESP_LOGI("NVS STORAGE: ","Opening Non-Volatile Storage (NVS) handle... ");
  nvs_handle_t nvs_handle;
  esp_err_t err;
  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
  
  if (err != ESP_OK) {
    ESP_LOGE("NVS STORAGE: ","Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  } else {
      ESP_LOGI("NVS STORAGE: ","Done\n");
      size_t  required_size = size * sizeof(uint8_t);
      ESP_LOGI("NVS STORAGE: ","handling %d bits.", required_size);
      if(write){// WRITE
	// Write value
	err = nvs_set_blob(nvs_handle, "imu_cal", calibration_data, required_size);
	if (err != ESP_OK) {ESP_LOGE("NVS STORAGE: ","Write Failed, error: %s",  esp_err_to_name(err)); } else{ESP_LOGI("NVS STORAGE: ","Write Done");}
	// Commit
	err = nvs_commit(nvs_handle);
	if (err != ESP_OK) {ESP_LOGE("NVS STORAGE: ","Commit Failed, error: %s", esp_err_to_name(err)); } else{ESP_LOGI("NVS STORAGE: ","Commit Done"); }
      }
      else{ // READ
	err = nvs_get_blob(nvs_handle, "imu_cal", calibration_data, &required_size);
	if (err != ESP_OK) {ESP_LOGE("NVS STORAGE: ","Read Failed, error: %s", esp_err_to_name(err)); } else{ESP_LOGI("NVS STORAGE: ","Read Done, read %d bits.", required_size); }
      }
      // Close
      nvs_close(nvs_handle);
  }  
}



void Qutee::load_imu_calibration()
{
  uint8_t calibration_data[23]; // adding one value to be set to 42 as sanity check. If loaded value is not 42, this means that data were not properly saved. 
  this->manage_imu_calibration_nvs(calibration_data, 23, false);// false to READ data
  if(calibration_data[22] == 42) // proper value loaded
    {
      ESP_LOGI("CALIBRATION DATA: "," successfully loaded saved values");
      this->_bno.setSensorOffsets(calibration_data);

      ESP_LOGI("CALIBRATION DATA saved: ","");
      for(size_t i =0; i<23; i++)
	ESP_LOGI("","%d : %d", i, calibration_data[i]);
    }
  else
    ESP_LOGI("CALIBRATION DATA: ","no existing calibration data. Calibration is required");
}


void Qutee::calibration()
{
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;

  this->_tft.setTextWrap(false);
  this->_tft.fillScreen(ST77XX_BLACK);
  this->_tft.setTextColor(ST77XX_BLUE,ST77XX_BLACK); 
  this->_tft.setTextSize(1);
  
  uint8_t count =0;
  sensors_event_t  linearAccelData;
  float accel_amp;
  while(count<10){ // wait for being fullycalibrated and with low acceleration (i.e., learned to remove gravity) for 10 consecutive steps, i.e., one seconds.
    this->_bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    accel_amp = linearAccelData.acceleration.x*linearAccelData.acceleration.x + linearAccelData.acceleration.y*linearAccelData.acceleration.y + linearAccelData.acceleration.z*linearAccelData.acceleration.z;
    
    if(this->_bno.isFullyCalibrated() && accel_amp<0.01)
      count++;
    else
      count=0;
    this->_bno.getCalibration(&system, &gyro, &accel, &mag);
    ESP_LOGI("CALIBRATION: ","system %i, gyro %i, accel %i, mag %i, accel_amp %f\n",  system , gyro ,accel, mag, accel_amp);
    this->_tft.setCursor(0, 20);
    this->_tft.print("system: "); this->_tft.println(system);
    this->_tft.print("gyro: "); this->_tft.println(gyro);
    this->_tft.print("accel: "); this->_tft.println(accel);
    this->_tft.print("mag: "); this->_tft.println(mag);
    this->_tft.print("Accel amplitude: "); this->_tft.println(accel_amp);
    this->_tft.print("x: "); this->_tft.print(linearAccelData.acceleration.x);
    this->_tft.print("y: "); this->_tft.print(linearAccelData.acceleration.y);
    this->_tft.print("z: "); this->_tft.print(linearAccelData.acceleration.z);
    std::this_thread::sleep_for(std::chrono::microseconds(100000));
  }
  this->_tft.fillScreen(ST77XX_BLACK);
  this->_tft.setCursor(0, 30);
  this->_tft.setTextColor(ST77XX_GREEN);
  this->_tft.setTextSize(4);
  this->_tft.println("CALIBRATION\n DONE! ");

  uint8_t calibration_data[23]; // adding one value to be set to 42 as sanity check. If loaded value is not 42, this means that data were not properly saved.
  if(this->_bno.getSensorOffsets(calibration_data)){
    calibration_data[22]=42; 
    ESP_LOGI("CALIBRATION DATA obtained: ","");
    for(size_t i =0; i<23; i++)
      ESP_LOGI("","%d : %d", i, calibration_data[i]);
    this->manage_imu_calibration_nvs(calibration_data, 23, true);// false to READ data
  }
  else
    ESP_LOGE("CALIBRATION: "," ERROR FAILED TO GET CALIBRATION PROFILE");
  this->_bno.setMode(OPERATION_MODE_IMUPLUS);

  
}

// Return the battery's voltage. Given that we use NIMH batteries, the voltage can exceed 5.14V, which is the maximum value that the battery monitor can report. When this happens, the monitor return unexpectedly low values (e.g., 0.01V, or 2.1V). Therefore, this function return 5.15V when the measured value is below 3.5V. 
float Qutee::battery_voltage()
{
  float voltage = this->_maxlipo.cellVoltage();
  if (voltage<3.5)
    {
      voltage = 5.15;
      ESP_LOGW("BATTERY: ","measured voltage unexpectedly low. We assume that it means that the battery's voltage is higher than 5.14V. Returning 5.15V instead. ");
    }
  return voltage;  
}



void Qutee::scan()
{
  // put your setup code here, to run once:
  int8_t found_dynamixel = 0;
  for(int id = 0; id < DXL_BROADCAST_ID; id++) {
      //iterate until all ID in each buadrate is scanned.
      if(this->_dxl.ping(id)) {
      ESP_LOGI("DXL: ","ID : %i  , Model Number: %i \n",id,this->_dxl.getModelNumber(id));
      found_dynamixel++;
      }
  }   
  ESP_LOGI("DXL: ","Total %i DYNAMIXEL(s) found!\n",found_dynamixel );
}

void Qutee::init_motors()
{
          // Turn off torque when configuring items in EEPROM area
  for(size_t i = 0; i<12; i++)
  {   
    this->_dxl.torqueOff(this->DXL_IDs[i]);
    this->_dxl.setOperatingMode(this->DXL_IDs[i], OP_POSITION);
    this->_dxl.torqueOn(this->DXL_IDs[i]);
    ESP_LOGI("DXL: ","TORQUE ON \n" );
    // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
    this->_dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_IDs[i], 0);
    this->set_PID_gains();
  }
}

void Qutee::set_PID_gains()
{
  // adjust these values depending on how your robot behaves. 0.05 looks like a good target to have on all motors during checkup. 
  uint16_t position_p_gain = 1600;
  uint16_t position_i_gain = 0;
  uint16_t position_d_gain = 40;
  // Set Position PID Gains
  for(size_t i = 0; i<12; i++)
  {
    this->_dxl.writeControlTableItem(POSITION_P_GAIN, this->DXL_IDs[i], position_p_gain);
    this->_dxl.writeControlTableItem(POSITION_I_GAIN, this->DXL_IDs[i], position_i_gain);
    this->_dxl.writeControlTableItem(POSITION_D_GAIN, this->DXL_IDs[i], position_d_gain);
  }
  return;
}

void Qutee::go_to_neutral_pose()
{
  Actions_t actions;
  actions = actions *0.0f;
  send_actions(actions);

}

void Qutee::displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  this->_bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  ESP_LOGI("IMU" ,"System Status: 0x%x, Self Test: 0x%x, System Error:  0x%x", system_status, self_test_results, system_error);   
  delay(500);
}

void Qutee::get_state(State_t& state_ref)
{    
  sensors_event_t orientationData , linearAccelData;
  this->_bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  this->_bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  state_ref.setValues({{this->DEG_2_RAD * orientationData.orientation.x},
                  {this->DEG_2_RAD * orientationData.orientation.y},
                  {this->DEG_2_RAD * orientationData.orientation.z},
                  {linearAccelData.acceleration.x}, 
                  {linearAccelData.acceleration.y},
                  {linearAccelData.acceleration.z}});

  this->get_motor_positions(state_ref,6);

}

void Qutee::send_actions(const Actions_t& actions){
  uint8_t i;
  // Insert a new Goal Position to the SyncWrite Packet
  for(i = 0; i < this->DXL_ID_CNT; i++){
    this->_sw_data[i].goal_position = (size_t) 2048+actions(i,0)*512;
  }

  // Update the SyncWrite packet status
  this->_sw_infos.is_info_changed = true;
  
  // Build a SyncWrite Packet and transmit to DYNAMIXEL  
  if(this->_dxl.syncWrite(&this->_sw_infos) == true){
    //ESP_LOGI("Control Loop","[SyncWrite] Success");
  } else {
    //ESP_LOGE("Control Loop","[SyncWrite] Fail, Lib error code: %i ",this->_dxl.getLastLibErrCode()); // TODO fix this error
  }

}

void Qutee::get_motor_positions(State_t& state_ref, size_t offset)
{
  // Transmit predefined SyncRead instruction packet
  // and receive a status packet from each DYNAMIXEL
  uint8_t i, recv_cnt;
  recv_cnt = this->_dxl.syncRead(&this->_sr_infos);
  if(recv_cnt > 0) {
    //ESP_LOGI("Control Loop","[SyncRead] Success, Received ID Count: %i ",recv_cnt);
    for(i = 0; i<recv_cnt; i++){
      state_ref(i+offset,0)= (this->_sr_data[i].present_position-2048.0f)/512.0f;
      //ESP_LOGI("Control Loop","  ID: %i, \t Present Position: %i",this->_sr_infos.p_xels[i].id), this->_sr_data[i].present_position);
    }
  }else{
    //ESP_LOGE("Control Loop"," [SyncRead] Fail, Lib error code: %i ",this->_dxl.getLastLibErrCode()); // TODO fix this error
  }

}

void Qutee::control_step(State_t& state_to_fill, Actions_t& actions_to_fill){
  //Get state

  this->get_state(state_to_fill);
  // Call policy
  this->_policy.pos(state_to_fill, actions_to_fill);
  // Send actions
  this->send_actions(actions_to_fill);
  // Print on screen. This takes ages, so disabled for now
  //this->tft_update_data_screen(_state, pos, 1000.0f / (float) full_period ); // period is in us        
 }

void Qutee::run_episode(){
  if(!this->_bno.isFullyCalibrated()) {this->calibration();}
  go_to_neutral_pose();
  std::this_thread::sleep_for(std::chrono::microseconds(1000000));
  int64_t start, sleep_duration;
  int64_t start_loop = esp_timer_get_time();
  for(size_t step = 0;step< NB_STEPS; step++)
  {  
    State_t state_to_fill;
    Actions_t actions_to_fill;

    start = esp_timer_get_time();

    control_step(state_to_fill,actions_to_fill);
    _states[step] = state_to_fill;
    _actions[step] = actions_to_fill;

    sleep_duration = PERIOD - (esp_timer_get_time() - start);
    //ESP_LOGI("Control Loop", "Period: %" PRId64" Start: %" PRId64" Sleep %" PRId64,period,  start, sleep_duration);
    if(sleep_duration>0)
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_duration));   
    else
      ESP_LOGE("Control Loop"," Error: loop duration: %" PRId64" longer than period: ", esp_timer_get_time() - start);
  }
  ESP_LOGI("Control Loop", "duration of the loop: %" PRId64, esp_timer_get_time() - start_loop);
 }



void Qutee::checkup(){
  this->_tft.fillScreen(ST77XX_BLACK);
  this->_tft.setTextSize(2);
  this->_tft.setCursor(0, 0);
  this->_tft.setTextColor(ST77XX_BLUE);
  this->_tft.println("CHECKUP");
  
  go_to_neutral_pose();
  std::this_thread::sleep_for(std::chrono::microseconds(1000000));
  int64_t start, sleep_duration;
  int64_t start_loop = esp_timer_get_time();
  State_t state;
  Actions_t actions;
  float action_value=0;
  int32_t step = 0;
  float er[12];
  for(size_t i=0;i<12;i++)
    er[i]=0;
			
  this->tft_init_checkup_screen();
  
  while(1)
  {  
    start = esp_timer_get_time();
    this->get_motor_positions(state,6);

    if(step % (2 * CONFIG_CONTROL_FREQUENCY)==0)
      this->tft_update_checkup_screen(er);

    for(size_t i=0;i<12;i++)
      er[i]= (step * er[i] + abs(state(6+i,0)-action_value))/(step+1.0f); //compute rolling average of error

    step++;

    //ESP_LOGI("Checkup", "Average execution error: 11:%f 12:%f 13:%f  21:%f 22:%f 23:%f  31:%f 32:%f 33:%f  41:%f 42:%f 43:%f ", er[0],er[1],er[2],er[3],er[4],er[5],er[6],er[7],er[8],er[9],er[10],er[11]);

    action_value = sin(0.5*6.28*(esp_timer_get_time() - start_loop)/1000000.0f); 
    actions.setConstant(action_value);
    this->send_actions(actions); // Sends a sin signal of 0.1Hz. 

    sleep_duration = (PERIOD - (esp_timer_get_time() - start)); 

    //ESP_LOGI("Control Loop", "Period: %" PRId64" Start: %" PRId64" Sleep %" PRId64,period,  start, sleep_duration);
    if(sleep_duration>0)
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_duration));   
    else
      ESP_LOGE("Checkup"," Error: loop duration: %" PRId64" longer than period: ", esp_timer_get_time() - start);
  }
  ESP_LOGI("Checkup", "duration of the loop: %" PRId64, esp_timer_get_time() - start_loop);
 }

void Qutee::tft_init_checkup_screen() {
  int lo = 10; // line offset
  int io = 32; // item offset
  this->_tft.fillScreen(ST77XX_BLACK);
  this->_tft.setCursor(0, 0);
  this->_tft.setTextColor(ST77XX_GREEN,ST77XX_BLACK);
  this->_tft.setTextSize(1);
  this->_tft.setCursor(0, 0);    this->_tft.setTextColor(ST77XX_RED,ST77XX_BLACK);    this->_tft.println("Error: ");

  this->_tft.setTextColor(ST77XX_BLUE,ST77XX_BLACK); 
  this->_tft.setCursor(0*io, 1*lo); this->_tft.print("11: ");  this->_tft.setCursor(2*io, 1*lo);  this->_tft.print(" 12: "); this->_tft.setCursor(4*io, 1*lo);  this->_tft.print(" 13: "); 
  this->_tft.setCursor(0*io, 2*lo); this->_tft.print("21: ");  this->_tft.setCursor(2*io, 2*lo);  this->_tft.print(" 22: "); this->_tft.setCursor(4*io, 2*lo);  this->_tft.print(" 23: "); 
  this->_tft.setCursor(0*io, 3*lo); this->_tft.print("31: ");  this->_tft.setCursor(2*io, 3*lo);  this->_tft.print(" 32: "); this->_tft.setCursor(4*io, 3*lo);  this->_tft.print(" 33: "); 
  this->_tft.setCursor(0*io, 4*lo); this->_tft.print("41: ");  this->_tft.setCursor(2*io, 4*lo);  this->_tft.print(" 42: "); this->_tft.setCursor(4*io, 4*lo);  this->_tft.print(" 43: "); 
}

void Qutee::tft_update_checkup_screen(const float* err) {
  int lo = 10; // line offset
  int io = 32; // item offset
  char buf[6]; // 5 characters + NUL
  this->_tft.setTextSize(1);
  this->_tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK); 
  this->_tft.setCursor(200 , 0);   this->_tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK); this->_tft.print(buf);

  this->_tft.setTextSize(1);
  // State
  for(size_t i=0; i < 12; i++){
    if(err[i]>0.1)
      this->_tft.setTextColor(ST77XX_RED,ST77XX_BLACK);
    else
      this->_tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK);
    sprintf(buf, "%- 1.2f",err[i] );
    this->_tft.setCursor( ((i%3) *2 +1)*io, (i/3 + 1)*lo); this->_tft.print(buf); 
  }
 
}


void Qutee::print_state(){
  State_t state;
  uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board (Taken from the lib example, probably wrong)
  //velocity = accel*dt (dt in seconds)
  //position = 0.5*accel*dt^2
  double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
  double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
  double xPos = 0, yPos = 0;

  int64_t start = esp_timer_get_time();

  while(1){
    this->get_state(state);
    double delta_t = (esp_timer_get_time() - start) /1000000.0;
    start = esp_timer_get_time();
    xPos = xPos + 0.5*delta_t*delta_t * state(3,0);
    yPos = yPos + 0.5*delta_t*delta_t * state(4,0);


    printf("%f, %f, %f, %f, %f, %f, %f, %f\n", state(0,0),state(1,0),state(2,0),state(3,0),state(4,0),state(5,0), xPos, yPos);
    //ESP_LOGI("STATE", "a: %f, b: %f, c: %f, x: %f, y: %f, z: %f", state(0,0),state(1,0),state(2,0),state(3,0),state(4,0),state(5,0));
  }
}




void Qutee::menu(){
  bool screen_on = false;
  gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
  gpio_set_direction(GPIO_NUM_1, GPIO_MODE_INPUT);
  gpio_set_direction(GPIO_NUM_2, GPIO_MODE_INPUT);

  while(1){
    if(!screen_on){
      screen_on=true;
      this->_tft.setTextWrap(false);
      this->_tft.fillScreen(ST77XX_BLACK);
      this->_tft.setTextSize(2);
      
      this->_tft.setCursor(0, 0);
      this->_tft.setTextColor(ST77XX_GREEN);
      this->_tft.println("Start Calibration");
      
      this->_tft.setCursor(0, 60);
      this->_tft.setTextColor(ST77XX_RED);
      this->_tft.println("Start Checkup");
      
      this->_tft.setCursor(0, 120);
      this->_tft.setTextColor(ST77XX_BLUE);
      this->_tft.println("Start ROS");

      ESP_LOGI("Menu", "Entering menu. Waiting for a button to be pressed");
    }

    
    delay(100);
    if(!gpio_get_level(GPIO_NUM_0))
      {
	this->calibration();
	screen_on=false; // set to false, to replot the screen when leaving calibration
      }
    if(gpio_get_level(GPIO_NUM_1))
      this->checkup(); // start checkup infinite loop
    if(gpio_get_level(GPIO_NUM_2)){
      this->_tft.fillScreen(ST77XX_BLACK); // clear menu before returning to the main function, where we launch ros and co.
      return;
    }

  }

}
