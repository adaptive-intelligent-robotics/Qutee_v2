#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include <esp_log.h>
#include <math.h>
#include <hal/misc.h>
#include <thread>
#include <stdlib.h>
#include "Qutee.hpp"


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






// EIGEN SHOULD BE AT THE END. OTHERWISE, issues with HIGH/LOW macros
#include <eigen3/unsupported/Eigen/CXX11/Tensor>






#define DOMAIN_ID 3

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
//sensor_msgs__msg__Imu msg;
std_msgs__msg__Float32MultiArray recvmsg;

Qutee robot;



void weights_receiver(const void * msgin)
{
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    ESP_LOGI("Weights receiver:","%i weights received", msg->data.size);  
    std::copy(msg->data.data, msg->data.data + msg->data.size, robot.get_policy().get_weights().data());
    return;
}


/*void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
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


}*/




void micro_ros_task(void * arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// Create init_options.
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	RCCHECK(rcl_init_options_set_domain_id(&init_options, DOMAIN_ID));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

	// Setup support structure.
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, CONFIG_QUTEE_NAME, "", &support));

 	// Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
		"weight_receiver"));
    
    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recvmsg, &weights_receiver, ON_NEW_DATA));

    // Spin forever
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(100000);
	}

	// Free resources.
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}











void robot_control_task(void * arg)
{   
    while(1){
        std::this_thread::sleep_for(std::chrono::microseconds(1000));   
        robot.control_step();

    }

}












extern "C" void app_main()
{ 
   /*#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    #endif*/
    
        
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
