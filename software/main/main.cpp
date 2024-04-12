// Define the logging level to be verbose
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

// Include necessary libraries
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

#include <type_utilities.h>


#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
//#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/float32.h>
#include "qutee_interface/msg/rollout_res.h"
#include "qutee_interface/msg/weights.h"
#include "qutee_interface/srv/status.h"
#include "qutee_interface/srv/rollout.h"
#include <rclc/rclc.h>
#include <rclc/executor.h>

//#include <std_srvs/srv/set_bool.h>
//#include "example_interfaces/srv/add_two_ints.h" // for tests with services

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

// Define macros for checking return codes from RCL functions
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}






// Include Eigen library
// EIGEN SHOULD BE AT THE END. OTHERWISE, issues with HIGH/LOW macros
#include <eigen3/unsupported/Eigen/CXX11/Tensor>



// Define DOMAIN_ID
#define DOMAIN_ID 0

// Declare Qutee object as global variable
Qutee robot;

// Declare variables for ROS messaging
static rcl_allocator_t allocator;
static rclc_support_t support;


static rcl_node_t node;

static char time_label[5] = "time";
static char action_label[7] = "action";
static char state_label[6] = "state";
static char weights_label[8] = "weights";
static char res_message = '\0';


qutee_interface__srv__Rollout_Response res_rollout;
qutee_interface__srv__Rollout_Request req_rollout;


/*// Initialize messages for ROS communication
void init_messages(qutee_interface__srv__Rollout_Request* request, qutee_interface__srv__Rollout_Response* response){

  static micro_ros_utilities_memory_conf_t conf_res = {0};
  micro_ros_utilities_memory_rule_t rules_res[] = {
    {"state.dim", 2},
    {"state.data", 45},//NB_STEPS*NN_INPUT_SIZE},

    {"state.dim", 2},
    {"state.data", 45}//NB_STEPS*NN_INPUT_SIZE},
  };
  //conf_res.rules = rules_res;
  //conf_res.n_rules = sizeof(rules_res) / sizeof(rules_res[0]);
  
  static micro_ros_utilities_memory_conf_t conf_req = {0};
  micro_ros_utilities_memory_rule_t rules_req[] = {
    {"weights.dim", 2},
    {"weights.data", 45}
  };
  // conf_req.rules = rules_req;
  // conf_req.n_rules = sizeof(rules_req) / sizeof(rules_req[0]);
  
  ESP_LOGI("ROS: ","start init message"); 
  bool success_res = micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(qutee_interface, srv, Rollout_Response),
							       &response,
							       conf_res);
  ESP_LOGI("ROS: ","suc res: %d", success_res);
  bool success_req = micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(qutee_interface, srv, Rollout_Request),
							       &request,
							       conf_req);
  ESP_LOGI("ROS: ","suc req: %d", success_req);
  ESP_LOGI("ROS: ","done init message"); 
  return;
  }*/


  
// Initialize messages for ROS communication
void init_messages(qutee_interface__srv__Rollout_Request& request, qutee_interface__srv__Rollout_Response& response){
    size_t nb_steps = NB_STEPS;
    size_t action_size = NN_OUTPUT_SIZE;
    size_t state_size = NN_INPUT_SIZE;
    // WARNING: we are currently limited to 32768 bytes in our response, while accounting for headers and other elements, it is safe to consider that we have 32000bytes available to store NB_STEPS*(NN_OUTPUT_SIZE+NN_INPUT_SIZE)*4 
    size_t nb_weights = robot.get_policy().get_number_weights();
    ESP_LOGI("ROS: ","Init message: step %d, action %d, state %d, nb_weights %d", nb_steps, action_size, state_size, nb_weights);
    // The structure of the message is: Time x action/state Dim.
    response.actions.layout.dim.size = 2;
    response.actions.layout.dim.capacity = 2; 
    response.actions.layout.dim.data= (std_msgs__msg__MultiArrayDimension*) malloc(response.actions.layout.dim.capacity*sizeof(std_msgs__msg__MultiArrayDimension));
    
    response.actions.layout.dim.data[0].label.capacity = 5;
    response.actions.layout.dim.data[0].label.size = 5;
    response.actions.layout.dim.data[0].label.data = time_label;
    response.actions.layout.dim.data[0].size = nb_steps;
    response.actions.layout.dim.data[0].stride = nb_steps * action_size ;
        
    response.actions.layout.dim.data[1].label.capacity = 7;
    response.actions.layout.dim.data[1].label.size = 7;
    response.actions.layout.dim.data[1].label.data = action_label;
    response.actions.layout.dim.data[1].size = action_size;
    response.actions.layout.dim.data[1].stride = action_size ;
    response.actions.layout.data_offset = 0;

    
    response.actions.data.capacity =  nb_steps * action_size;
    response.actions.data.size =  nb_steps * action_size;
    response.actions.data.data = (float*) malloc(response.actions.data.capacity*sizeof(float));

    /// now we redo the same with states
    response.states.layout.dim.size = 2;
    response.states.layout.dim.capacity = 2; 
    response.states.layout.dim.data= (std_msgs__msg__MultiArrayDimension*) malloc(response.states.layout.dim.capacity*sizeof(std_msgs__msg__MultiArrayDimension));

    response.states.layout.dim.data[0].label.capacity = 5;
    response.states.layout.dim.data[0].label.size = 5;
    response.states.layout.dim.data[0].label.data = time_label;
    response.states.layout.dim.data[0].size = nb_steps;
    response.states.layout.dim.data[0].stride = nb_steps * state_size ;
    
    response.states.layout.dim.data[1].label.capacity = 6;
    response.states.layout.dim.data[1].label.size = 6;
    response.states.layout.dim.data[1].label.data = state_label;
    response.states.layout.dim.data[1].size = state_size;
    response.states.layout.dim.data[1].stride = state_size ;
    response.states.layout.data_offset = 0;
    
    
    response.states.data.capacity =  nb_steps * state_size;
    response.states.data.size =  nb_steps * state_size;
    response.states.data.data = (float*) malloc(response.states.data.capacity*sizeof(float));

    
        
    /// now we redo the same with Weights
    request.weights.layout.dim.size = 1;
    request.weights.layout.dim.capacity = 1; 
    request.weights.layout.dim.data= (std_msgs__msg__MultiArrayDimension*) malloc(request.weights.layout.dim.capacity*sizeof(std_msgs__msg__MultiArrayDimension));
    
    request.weights.layout.dim.data[0].label.capacity = 8;
    request.weights.layout.dim.data[0].label.size = 8;
    request.weights.layout.dim.data[0].label.data = weights_label;
    request.weights.layout.dim.data[0].size = nb_weights;
    request.weights.layout.dim.data[0].stride = nb_weights ;
    request.weights.layout.data_offset = 0;

    request.weights.data.capacity =  nb_weights;
    request.weights.data.size =  nb_weights;
    request.weights.data.data = (float*) malloc(request.weights.data.capacity*sizeof(float));
    
    
    return;
}


// Create response message for ROS communication
void create_response(qutee_interface__srv__Rollout_Response* response)
{
    ESP_LOGI("ROS: ","Step1");  
    size_t action_size = NN_OUTPUT_SIZE;
    size_t state_size = NN_INPUT_SIZE;
    ESP_LOGI("ROS: ","step2: %i", robot.get_list_actions().size() );  
    // The structure of the message is: Time x action/state Dim.
    for(size_t i =0; i<robot.get_list_actions().size(); i++)
    {
        std::copy(robot.get_list_actions()[i].data(), robot.get_list_actions()[i].data()+action_size, response->actions.data.data+ i*action_size);   
    }
    ESP_LOGI("ROS: ","Step3: %i", robot.get_list_states().size());  
    /// now we redo the same with states
    for(size_t i =0; i<robot.get_list_states().size(); i++)
    {
        std::copy(robot.get_list_states()[i].data(), robot.get_list_states()[i].data()+state_size, response->states.data.data+ i*state_size);   
    }
    return;
}

/*void get_num_params_callback(const void * req, void * res)
{
    ESP_LOGI("ROS: ","Entered the num_params callback");  
    qutee_interface__srv__GetNumParams_Request * req_in = (qutee_interface__srv__GetNumParams_Request *) req; // This should be empty
    qutee_interface__srv__GetNumParams_Response * res_in = (qutee_interface__srv__GetNumParams_Response *) res;

    res_in->num_params=robot.get_policy().get_number_weights();

    return;
}*/

// Callback function for ROS subscriber
void rollout_callback(const void * req, void * res)
{
    ESP_LOGI("ROS: ","Entered the Rollout callback");  
    qutee_interface__srv__Rollout_Request * weights_msg = (qutee_interface__srv__Rollout_Request *) req;
    qutee_interface__srv__Rollout_Response * transitions = (qutee_interface__srv__Rollout_Response *) res; 
    
    ESP_LOGI("ROS: ","First Weight: %f last Weight: %f", *weights_msg->weights.data.data,*(weights_msg->weights.data.data+weights_msg->weights.data.size-1 )); 
    ESP_LOGI("ROS: ","INIT First Weight: %f last Weight: %f", *robot.get_policy().get_weights().data(),*(robot.get_policy().get_weights().data() + 321));
    // Copy weight to the robot's policy
    std::copy(weights_msg->weights.data.data, weights_msg->weights.data.data+weights_msg->weights.data.size, robot.get_policy().get_weights().data());    
    ESP_LOGI("ROS: ","AFTER First Weight: %f last Weight: %f", *robot.get_policy().get_weights().data(),*(robot.get_policy().get_weights().data() + 321));    

    // Run the episode
    uint32_t begin_time = esp_log_timestamp();
    robot.run_episode();

    uint32_t duration = esp_log_timestamp() - begin_time;
    ESP_LOGI("Control Loop","\tTimings: FREQ: %f", (float) CONFIG_EPISODE_DURATION * 1000.0f / (float) duration);

    // Send the transitions
    create_response(transitions);
    ESP_LOGI("Control Loop","DONE");
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


void status_callback(const void * req, void * res){

  //qutee_interface__srv___Request * req_in = (qutee_interface__srv__Status_Request *) req;
  qutee_interface__srv__Status_Response * res_in = (qutee_interface__srv__Status_Response *) res;

  printf("Service Status request received\n");
  
  res_in->battery = robot.battery_voltage();
  printf("Service getting nb weights\n");
  res_in->number_weights = robot.get_policy().get_number_weights();
  printf("Service done\n");

  // we can probably move such static init to the message init instead. 
  res_in->error_message.data = &res_message;
  res_in->error_message.size = 1;
  res_in->error_message.capacity = 1; 
  
}




// Micro-ROS task function
// this function init ros node, creates the subscribers and publishers and spins infinitely.
void micro_ros_task(void * arg)
{
    allocator = rcl_get_default_allocator();
 
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
    RCCHECK(rclc_node_init_default(&node, "qutee_node", CONFIG_QUTEE_NAME, &support));



    
    // create executor
    rclc_executor_t executor;
    unsigned int num_handles = 2; // we have two handles, one for each service
    RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
    
    // Create service for Status
    ESP_LOGI("UROS","Init status service");
    rcl_service_t service_status;
    RCCHECK(rclc_service_init_default(&service_status, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(qutee_interface, srv, Status), "status"));

    qutee_interface__srv__Status_Response res_status;
    qutee_interface__srv__Status_Request req_status;

    RCCHECK(rclc_executor_add_service(&executor, &service_status, &req_status, &res_status, status_callback));
    ESP_LOGI("UROS","DONE");
    
    // Create service for Rollout
    ESP_LOGI("UROS","Starting Rollout service");
    rcl_service_t service_rollout;
    RCCHECK(rclc_service_init_default(&service_rollout, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(qutee_interface, srv, Rollout), "rollout"));
    ESP_LOGI("UROS","message");
    ESP_LOGI("UROS","alloc");
    init_messages(req_rollout, res_rollout);
    ESP_LOGI("UROS","exec");
    RCCHECK(rclc_executor_add_service(&executor, &service_rollout, &req_rollout, &res_rollout, rollout_callback));
    ESP_LOGI("UROS","DONE");
    
    /*
    // Create publisher.
    RCCHECK(rclc_publisher_init_default(
					&publisher,
					&node,
					ROSIDL_GET_MSG_TYPE_SUPPORT(qutee_interface, msg, RolloutRes),
					"qutee_rollout_results"));

    // Create subscriber.
    subscriber = rcl_get_zero_initialized_subscription();
    RCCHECK(rclc_subscription_init_default(
					   &subscriber,
					   &node,
					   ROSIDL_GET_MSG_TYPE_SUPPORT(qutee_interface, msg, Weights),
					   "qutee_weights_to_evaluate"));

    
    // Create executor.
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    unsigned int rcl_wait_timeout = 1000;   // in ms
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
    
    // Add timer and subscriber to executor.
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &weights, &rollout_callback, ON_NEW_DATA));
    */
    
    // Spin forever
    while(1){
      RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10000)));
      usleep(100000);
    }

    // Free resources
    //TODO IF YOU NEED TO, Here Qutee is killed when not in use. 
}










// Entry point of the application
extern "C" void app_main()
{ 
    // Initialize robot and perform calibration
    robot.init();
    robot.menu();
    // if the user selects "launch ros" in the menu, the menu returns here. 

    // Initialize network interface if needed
    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    #endif
    
    // Create task to handle Micro-ROS operations
    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL); 

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    /*xTaskCreate(robot_control_task,
            "uros_task",
            CONFIG_ROBOT_CTRL_APP_STACK,
            NULL,
            CONFIG_ROBOT_CTRL_APP_TASK_PRIO,
            NULL);  

    */
}
