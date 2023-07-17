#include "QuteeDxlPortHandler.hpp"

QuteeDxlPortHandler::QuteeDxlPortHandler(uart_port_t uart_num)
:uart_num_(uart_num), TAG("ESP2Dynamixel")
{
    //this->TAG = "ESP2Dynamixel";
}

void QuteeDxlPortHandler::begin()
{
    begin(115200);
}

void QuteeDxlPortHandler::begin(int baud )
{
    baud_ = baud;
    ESP_LOGI(this->TAG.c_str(),"START CUSTOM PORT HANDLER");
    uart_config_t uart_config = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num_, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num_, GPIO_NUM_17, GPIO_NUM_18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num_, uart_buffer_size, \
                                    uart_buffer_size, 0, NULL, 0));

    uart_set_mode(UART_NUM_1, UART_MODE_RS485_APP_CTRL);
    UART1.rs485_conf.rs485rxby_tx_en = 0;  // don't send while receiving => collision avoidance
    UART1.rs485_conf.rs485tx_rx_en = 1;  // loopback (1), so collision detection works

    setOpenState(true);
    ESP_LOGI(this->TAG.c_str(),"START CUSTOM PORT HANDLER: DONE");
}

void QuteeDxlPortHandler::end()
{
    ESP_ERROR_CHECK(uart_driver_delete(uart_num_));
    setOpenState(false);
}
int QuteeDxlPortHandler::available()
{
    size_t len;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num_, &len));
    return len;
}

int QuteeDxlPortHandler::read()
{
    uint8_t val;
    uart_read_bytes(uart_num_, &val, 1, 100); // I randomly set the last param to 100 ... not sure
    return val;

}


size_t QuteeDxlPortHandler::write(uint8_t c) 
{
    size_t ret = 0;
    
    //ret = port_.write(c);
    ret = uart_write_bytes(uart_num_, &c, 1);
    ESP_ERROR_CHECK(uart_flush(uart_num_));
    
    return ret;
}

size_t QuteeDxlPortHandler::write(uint8_t *buf, size_t len) 
{
    size_t ret;

    //ret = port_.write(buf, len);
    ret = uart_write_bytes(uart_num_, buf, len);  
    ESP_ERROR_CHECK(uart_flush(uart_num_));

    return ret;     
}

