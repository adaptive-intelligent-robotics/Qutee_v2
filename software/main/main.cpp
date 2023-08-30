#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <esp_log.h>
#include <math.h>
#include <hal/misc.h>
#include <thread>
#include <stdlib.h>
#include <eigen3/Eigen/Eigen> // EIGEN Should be loaded before Arduino. Otherwise we have some definition issues. 
#include "Arduino.h"
#include "Dynamixel2Arduino.h"
#include "utility/port_handler.h"

#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
//#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/float32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}




#include "QuteeDxlPortHandler.hpp"
#include <Wire.h>
#include "SparkFun_ISM330DHCX.h"
#include "SparkFun_MMC5983MA_Arduino_Library.h"

#include "QuteeController.hpp"
#include "NNQuteeController.hpp"




 

SFE_MMC5983MA myMag;
SparkFun_ISM330DHCX myISM; 
// Structs for X,Y,Z data
sfe_ism_data_t accelData; 
sfe_ism_data_t gyroData; 




float amplitude = 0;


rcl_publisher_t publisher;
//sensor_msgs__msg__Imu msg;
std_msgs__msg__Float32MultiArray msg;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{

	RCLC_UNUSED(last_call_time);
	if (timer != NULL && myISM.checkStatus() ) {
        myISM.getAccel(&accelData);
        myISM.getGyro(&gyroData);
        msg.layout.dim.size = 1;
        msg.layout.dim.capacity = 1; 
        msg.layout.dim.data= (std_msgs__msg__MultiArrayDimension*) malloc(msg.layout.dim.capacity*sizeof(std_msgs__msg__MultiArrayDimension));
        msg.layout.dim.data[0].label.capacity = 5;
        msg.layout.dim.data[0].label.size = 5;
        msg.layout.dim.data[0].label.data = "label";
        msg.layout.dim.data[0].size = 9;
        msg.layout.dim.data[0].stride = 9;
        msg.layout.data_offset = 0;

        msg.data.capacity = 9;
        msg.data.size = 9;
        msg.data.data = (float*) malloc(msg.data.capacity*sizeof(float));
        msg.data.data[0] = accelData.xData;
        msg.data.data[1] = accelData.yData;
        msg.data.data[2] = accelData.zData;
        msg.data.data[3] = gyroData.xData;
        msg.data.data[4] = gyroData.yData;
        msg.data.data[5] = gyroData.zData;
        msg.data.data[6] = (myMag.getMeasurementX()- 131072.0)/131072.0 *8;
        msg.data.data[7] = (myMag.getMeasurementY()- 131072.0)/131072.0 *8;
        msg.data.data[8] = (myMag.getMeasurementZ()- 131072.0)/131072.0 *8;
        
 

        
		ESP_LOGI("IMU Publisher","DATA SENT");
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

	}


}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "Qutee_IMU_publisher", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
		"Qutee_IMU"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}


NNQuteeController<9,1,8,12> nncontroller;

/*
static void do_retransmit(const int sock)
{
    int len;
    char rx_buffer[128];
    do {
        std::this_thread::sleep_for(std::chrono::microseconds(10000));
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0)
        {
         if(errno != EWOULDBLOCK) ESP_LOGE(TAGWIFI, "Error occurred during receiving: errno %d", errno);
         else{ // No new message and here to send potential ones. 
            if( myISM.checkStatus() ){
                myISM.getAccel(&accelData);
                myISM.getGyro(&gyroData);
                Eigen::Matrix< float , 9 , 1> imu_data;
                imu_data << accelData.xData,
                            accelData.yData,
                            accelData.zData,
                            gyroData.xData, 
                            gyroData.yData, 
                            gyroData.zData,
                            ((double)myMag.getMeasurementX()- 131072.0)/131072.0 *8,
                            ((double)myMag.getMeasurementY()- 131072.0)/131072.0 *8, 
                            ((double)myMag.getMeasurementZ()- 131072.0)/131072.0 *8;

                auto pos = nncontroller.pos(imu_data);
                std::stringstream ss;
                ss << imu_data.transpose()<< "\n" << pos.transpose()<< "\n\n";
                std::string imu_dat_str = ss.str();
                // send() can return less bytes than supplied length.
                // Walk-around for robust implementation.
                int to_write = imu_dat_str.size();
                while (to_write > 0) {
                    int written = send(sock, imu_dat_str.c_str() + (imu_dat_str.size() - to_write), to_write, 0);
                    if (written < 0) {
                        ESP_LOGE(TAGWIFI, "Error occurred during sending: errno %d", errno);
                    }
                    to_write -= written;
                }
            }
         }
        } else if (len == 0) {
            ESP_LOGW(TAGWIFI, "Connection closed");

        } else {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAGWIFI, "Received %d bytes: %s", len, rx_buffer);

            if(rx_buffer[0] =='A'){
              ESP_LOGI(TAGWIFI, "Received Started with A"); 
                char *p = rx_buffer;
                while (*p) { // While there are more characters to process...
                  if ( isdigit(*p) || ( (*p=='-'||*p=='+') && isdigit(*(p+1)) )) {
                      // Found a number
                      long val = strtol(p, &p, 10); // Read number
                      ESP_LOGI(TAGWIFI, "FOUND NUMBER %ld", val);
                      if (val <=100 && val>=0) 
                        amplitude = val/100.0;
                      else
                        ESP_LOGE(TAGWIFI, "INVALID NUMBER RECEIVED");
                  } else {  
                      // Otherwise, move on to the next character.
                      p++;
                  }
              }
          }

            // send() can return less bytes than supplied length.
            // Walk-around for robust implementation.
            int to_write = len;
            while (to_write > 0) {
                int written = send(sock, rx_buffer + (len - to_write), to_write, 0);
                if (written < 0) {
                    ESP_LOGE(TAGWIFI, "Error occurred during sending: errno %d", errno);
                }
                to_write -= written;
            }
        }
    } while (len > 0 || (len ==-1 && errno == EWOULDBLOCK));
}
*/


//This namespace is required to use Control table item names
using namespace ControlTableItem;

void init_imu(){
    gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_7, 1);
    ESP_LOGI("IMU","START IMU PROCESS.");
    Wire.setPins(GPIO_NUM_3,GPIO_NUM_4);
    Wire.begin();

    if( !myISM.begin() ){
		ESP_LOGI("IMU","Did not start.");
		while(1);
	}
    if (!myMag.begin() ){
        ESP_LOGI("MAG","Did not start.");
		while(1);
    }

	// Reset the device to default settings. This if helpful is you're doing multiple
	// uploads testing different settings. 
	myISM.deviceReset();
    myMag.softReset();

	// Wait for it to finish reseting
	while( !myISM.getDeviceReset() ){ 
        ESP_LOGI("IMU","Reseting.");
		delay(1);
	} 
    ESP_LOGI("IMU","Reset. Applying settings.");
	delay(100);
	
	myISM.setDeviceConfig();
	myISM.setBlockDataUpdate();
	
	// Set the output data rate and precision of the accelerometer
	myISM.setAccelDataRate(ISM_XL_ODR_104Hz);
	myISM.setAccelFullScale(ISM_4g); 

	// Set the output data rate and precision of the gyroscope
	myISM.setGyroDataRate(ISM_GY_ODR_104Hz);
	myISM.setGyroFullScale(ISM_500dps); 

	// Turn on the accelerometer's filter and apply settings. 
	myISM.setAccelFilterLP2();
	myISM.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);

	// Turn on the gyroscope's filter and apply settings. 
	myISM.setGyroFilterLP1();
	myISM.setGyroLP1Bandwidth(ISM_MEDIUM);
    ESP_LOGI("IMU","IMU DONE.");

}

void robot_control_task(void * arg)
{
    Dynamixel2Arduino dxl;
    QuteeDxlPortHandler dxl_port(UART_NUM_1);
    dxl.setPort(dxl_port);
    int32_t baud = 1000000;
    int8_t  protocol = 2;       
    // put your setup code here, to run once:
    int8_t index = 0;
    int8_t found_dynamixel = 0;

    dxl.setPortProtocolVersion((float) protocol);
    ESP_LOGI("SCAN: ","PROTOCOL %i", protocol);

    ESP_LOGI("SCAN: ","BAUDRATE %i\n", baud);
    dxl.begin(baud);
    int DXL_IDs[] = {11, 12, 13, 21, 22, 23,  31, 32, 33, 41, 42, 43};

    /*for(int id = 0; id < DXL_BROADCAST_ID; id++) {
        //iterate until all ID in each buadrate is scanned.
        if(dxl.ping(id)) {
        ESP_LOGI(TAG,"ID : %i  , Model Number: %i \n",id,dxl.getModelNumber(id));
        DXL_ID = id;
        found_dynamixel++;
        }
    }
    ESP_LOGI(TAG,"Total %i DYNAMIXEL(s) found!\n",found_dynamixel );
        // Do your own thing

    if (found_dynamixel==0){return;}    
    */


     // Turn off torque when configuring items in EEPROM area
     for(size_t i = 0; i<12; i++)
      {   
        dxl.torqueOff(DXL_IDs[i]);
        dxl.setOperatingMode(DXL_IDs[i], OP_POSITION);
        dxl.torqueOn(DXL_IDs[i]);

        // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
        dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_IDs[i], 0);
    }





    Eigen::VectorXf params = Eigen::VectorXf::Zero(24);
    params(2) = 0.5;
    params(5) = 0.5;
    params(8) = 0.5;
    params(11) = 0.5;
    params(14) = 0.5;
    params(17) = 0.5;
    params(20) = 0.5;
    params(23) = 0.5;

              
    QuteeController ctrl(params);

    const int begin_time = esp_timer_get_time();
    while(1){
        std::this_thread::sleep_for(std::chrono::microseconds(1000));   


        /*if (ctrl.parameters()[0]!= amplitude)
        {   
            params(0) = amplitude;
            params(3) = amplitude;
            params(6) = amplitude;
            params(9) = amplitude;
            params(12) = amplitude;
            params(15) = amplitude;
            params(18) = amplitude;
            params(21) = amplitude;
            ctrl.set_parameters(params);
        }
         
        float time = float( esp_timer_get_time() - begin_time )/1000000.0;
        Eigen::VectorXf pos = ctrl.pos(time);*/

        if( myISM.checkStatus() ){
        myISM.getAccel(&accelData);
        myISM.getGyro(&gyroData);
        Eigen::Matrix< float , 9 , 1> imu_data;
        imu_data << accelData.xData,
                    accelData.yData,
                    accelData.zData,
                    gyroData.xData, 
                    gyroData.yData, 
                    gyroData.zData,
                    ((double)myMag.getMeasurementX()- 131072.0)/131072.0 *8,
                    ((double)myMag.getMeasurementY()- 131072.0)/131072.0 *8, 
                    ((double)myMag.getMeasurementZ()- 131072.0)/131072.0 *8;

        auto pos = nncontroller.pos(imu_data);


        for(size_t i = 0; i<12; i++){
          //dxl.setGoalPosition(DXL_IDs[i], (size_t) 2048+pos[i]*4096.0/(2*3.14));
          dxl.setGoalPosition(DXL_IDs[i], (size_t) 2048+pos[i]*256);
          ESP_LOGI("TEST","ID:%i: %f ", DXL_IDs[i], (size_t) 2048+pos[i]*4096.0/(2*3.14)); 
        }
        //int cur_pos= dxl.getPresentPosition(DXL_ID);
        //ESP_LOGI(TAG,"Present_Position(raw) :%i  VS Target Pos : %i ", cur_pos, target_pos ); 
        //delay(10);
        }


    }

}

extern "C" void app_main()
{ 
    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    #endif
    
    init_imu();


    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
   /* xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);  */

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(robot_control_task,
            "uros_task",
            CONFIG_ROBOT_CTRL_APP_STACK,
            NULL,
            CONFIG_ROBOT_CTRL_APP_TASK_PRIO,
            NULL);  

    
}
