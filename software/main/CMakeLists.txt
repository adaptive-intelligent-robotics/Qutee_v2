

idf_component_register(SRCS "main.cpp"
                            "Qutee.cpp"       
                            "QuteeDxlPortHandler.cpp"
                            "../components/Dynamixel2Arduino/src/Dynamixel2Arduino.cpp" 
                            "../components/Dynamixel2Arduino/src/actuator.cpp" 
                            "../components/Dynamixel2Arduino/src/utility/master.cpp" 
                            "../components/Dynamixel2Arduino/src/utility/port_handler.cpp" 
                            "../components/Dynamixel2Arduino/src/utility/slave.cpp" 
                            "../components/Dynamixel2Arduino/src/dxl_c/protocol.c"
                            "../components/Adafruit-ST7735-Library/Adafruit_ST7789.cpp"
                            "../components/Adafruit-ST7735-Library/Adafruit_ST7735.cpp"
                            "../components/Adafruit-ST7735-Library/Adafruit_ST77xx.cpp"
                            "../components/Adafruit_Seesaw/Adafruit_Crickit.cpp"
                            "../components/Adafruit_Seesaw/Adafruit_miniTFTWing.cpp"
                            "../components/Adafruit_Seesaw/Adafruit_NeoKey_1x4.cpp"
                            "../components/Adafruit_Seesaw/Adafruit_NeoTrellis.cpp"
                            "../components/Adafruit_Seesaw/Adafruit_seesaw.cpp"
                            "../components/Adafruit_Seesaw/Adafruit_TFTShield18.cpp"
                            "../components/Adafruit_Seesaw/seesaw_neopixel.cpp"
                            "../components/Adafruit_Seesaw/seesaw_servo.cpp"
                            "../components/Adafruit_Seesaw/seesaw_spectrum.cpp"
                            "../components/Adafruit_Sensor/Adafruit_Sensor.cpp"
                            "../components/Adafruit_BNO055/Adafruit_BNO055.cpp"
 

                    INCLUDE_DIRS "../components/Dynamixel2Arduino/src/." 
                                "../components/Dynamixel2Arduino/src/utility/." 
                                "../components/Dynamixel2Arduino/src/dxl_c/."
                                "../components/Adafruit-ST7735-Library/"
                                "../components/Adafruit_Seesaw/"
                                "../components/Adafruit_Sensor/"
                                "../components/Adafruit_BNO055/"
                 



                    REQUIRES "arduino-esp32" # Library requires Arduino
                             "eigen"
                             "micro_ros_espidf_component"
                             "Adafruit-GFX-Library"
                             "Adafruit_BusIO"

                    )
set_target_properties(${COMPONENT_LIB} PROPERTIES COMPILE_FLAGS "-Wno-maybe-uninitialized -Wimplicit-function-declaration -fpermissive")

