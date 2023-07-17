#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <esp_log.h>
#include <math.h>
#include <hal/misc.h>

#include <stdlib.h>

#include "Arduino.h"
#include "Dynamixel2Arduino.h"
#include "utility/port_handler.h"

#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"


#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "QuteeDxlPortHandler.hpp"
#include <Wire.h>
#include "SparkFun_ISM330DHCX.h"

SparkFun_ISM330DHCX myISM; 
// Structs for X,Y,Z data
sfe_ism_data_t accelData; 
sfe_ism_data_t gyroData; 



#define PORT                       CONFIG_EXAMPLE_PORT
#define KEEPALIVE_IDLE             CONFIG_EXAMPLE_KEEPALIVE_IDLE
#define KEEPALIVE_INTERVAL         CONFIG_EXAMPLE_KEEPALIVE_INTERVAL
#define KEEPALIVE_COUNT            CONFIG_EXAMPLE_KEEPALIVE_COUNT
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL
#define EXAMPLE_MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN

int amplitude = 0;

static const char *TAGWIFI = "wifi softAP";


//bool did_collision_happen_since_last_check() {
//    bool ret = UART1.int_raw.rs485_clash_int_raw;
//    UART1.int_clr.rs485_clash_int_clr = 1;  // clear bit
//    return ret;
// }

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAGWIFI, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAGWIFI, "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.ap.ssid, EXAMPLE_ESP_WIFI_SSID);
    strcpy((char*)wifi_config.ap.password, EXAMPLE_ESP_WIFI_PASS);
    wifi_config.ap.ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID);
    wifi_config.ap.channel = EXAMPLE_ESP_WIFI_CHANNEL;
    wifi_config.ap.max_connection = EXAMPLE_MAX_STA_CONN;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
   
  
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAGWIFI, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
 
}





static void do_retransmit(const int sock)
{
    int len;
    char rx_buffer[128];

    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAGWIFI, "Error occurred during receiving: errno %d", errno);
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
                      ESP_LOGI(TAGWIFI, "FOUNDN NUMBER %ld", val);
                      if (val <=512 && val>=0) 
                        amplitude = val;
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
    } while (len > 0);
}

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }
#ifdef CONFIG_EXAMPLE_IPV6
    else if (addr_family == AF_INET6) {
        struct sockaddr_in6 *dest_addr_ip6 = (struct sockaddr_in6 *)&dest_addr;
        bzero(&dest_addr_ip6->sin6_addr.un, sizeof(dest_addr_ip6->sin6_addr.un));
        dest_addr_ip6->sin6_family = AF_INET6;
        dest_addr_ip6->sin6_port = htons(PORT);
        ip_protocol = IPPROTO_IPV6;
    }
#endif

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAGWIFI, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
    // Note that by default IPV6 binds to both protocols, it is must be disabled
    // if both protocols used at the same time (used in CI)
    setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
#endif

    ESP_LOGI(TAGWIFI, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAGWIFI, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAGWIFI, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAGWIFI, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAGWIFI, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {

        ESP_LOGI(TAGWIFI, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAGWIFI, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
#ifdef CONFIG_EXAMPLE_IPV6
        else if (source_addr.ss_family == PF_INET6) {
            inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
        }
#endif
        ESP_LOGI(TAGWIFI, "Socket accepted ip address: %s", addr_str);

        do_retransmit(sock);

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}




//This namespace is required to use Control table item names
using namespace ControlTableItem;

void init_imu(){
 Wire.begin();
    if( !myISM.begin() ){
		Serial.println("Did not begin.");
		while(1);
	}

	// Reset the device to default settings. This if helpful is you're doing multiple
	// uploads testing different settings. 
	myISM.deviceReset();

	// Wait for it to finish reseting
	while( !myISM.getDeviceReset() ){ 
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



extern "C" void app_main()
{ 
   
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAGWIFI, "ESP_WIFI_MODE_AP");
    wifi_init_softap();


#ifdef CONFIG_EXAMPLE_IPV4
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)AF_INET, 5, NULL);
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)AF_INET6, 5, NULL);
#endif

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




    float time  = 0.0;
    
    while(1){
        time = time +1.0;
        int target_pos = 2048+amplitude*sin(time/40.0);
        for(size_t i = 0; i<12; i++)
          dxl.setGoalPosition(DXL_IDs[i], target_pos);
        //int cur_pos= dxl.getPresentPosition(DXL_ID);
        //ESP_LOGI(TAG,"Present_Position(raw) :%i  VS Target Pos : %i ", cur_pos, target_pos ); 
        //delay(10);

        if( myISM.checkStatus() ){
		myISM.getAccel(&accelData);
		myISM.getGyro(&gyroData);
        ESP_LOGI("Accelerometer: ","X: %f, Y:%f, Z:%f \n", accelData.xData,accelData.yData,accelData.zData);
		ESP_LOGI("Gyroscope: ","X: %f, Y:%f, Z:%f \n", gyroData.xData,gyroData.yData,gyroData.zData);

	}

    }
}
