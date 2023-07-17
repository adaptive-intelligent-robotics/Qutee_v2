#ifndef QUTEE_DYNAMIXEL_PORT_HANDLER_HPP_
#define QUTEE_DYNAMIXEL_PORT_HANDLER_HPP_

#include <esp_log.h>
#include <math.h>
#include <hal/misc.h>
#include <hal/uart_types.h>
#include <soc/uart_periph.h>
#include <soc/dport_reg.h>
#include <soc/uart_struct.h>
#include <driver/gpio.h>
#include <driver/uart.h>

#include <stdlib.h>

#include "Arduino.h"
#include "Dynamixel2Arduino.h"
#include "utility/port_handler.h"

class QuteeDxlPortHandler : public DXLPortHandler
{
  public:
    QuteeDxlPortHandler(uart_port_t uart_num);
    virtual void begin();
    virtual void begin(int baud );
    virtual void end();
    virtual int available();
    virtual int read();
    virtual size_t write(uint8_t c) override;
    virtual size_t write(uint8_t *buf, size_t len) override;

  private:
    const uart_port_t uart_num_;
    unsigned long baud_;
    const String TAG;
};

#endif