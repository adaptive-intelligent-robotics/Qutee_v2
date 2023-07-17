idf_component_register(SRCS "main.cpp" 
                            "QuteeDxlPortHandler.cpp"
                            "../components/Dynamixel2Arduino/src/Dynamixel2Arduino.cpp" 
                            "../components/Dynamixel2Arduino/src/actuator.cpp" 
                            "../components/Dynamixel2Arduino/src/utility/master.cpp" 
                            "../components/Dynamixel2Arduino/src/utility/port_handler.cpp" 
                            "../components/Dynamixel2Arduino/src/utility/slave.cpp" 
                            "../components/Dynamixel2Arduino/src/dxl_c/protocol.c"
                            "../components/SparkFun_6DoF_ISM330DHCX_Arduino_Library/src/sfe_ism330dhcx.cpp"
                            "../components/SparkFun_6DoF_ISM330DHCX_Arduino_Library/src/sfe_ism_shim.cpp"
                            "../components/SparkFun_6DoF_ISM330DHCX_Arduino_Library/src/sfe_bus.cpp"
                            "../components/SparkFun_6DoF_ISM330DHCX_Arduino_Library/src/st_src/ism330dhcx_reg.c"
                            "../components/SparkFun_MMC5983MA_Magnetometer_Arduino_Library/src/SparkFun_MMC5983MA_IO.cpp"
                            "../components/SparkFun_MMC5983MA_Magnetometer_Arduino_Library/src/SparkFun_MMC5983MA_Arduino_Library.cpp"
                    INCLUDE_DIRS "../components/Dynamixel2Arduino/src/." 
                                "../components/Dynamixel2Arduino/src/utility/." 
                                "../components/Dynamixel2Arduino/src/dxl_c/."
                                "../components/SparkFun_6DoF_ISM330DHCX_Arduino_Library/src/."
                                "../components/SparkFun_6DoF_ISM330DHCX_Arduino_Library/src/st_src"
                                "../components/SparkFun_MMC5983MA_Magnetometer_Arduino_Library/src"

                    REQUIRES "arduino-esp32" # Library requires Arduino
                    )
set_target_properties(${COMPONENT_LIB} PROPERTIES COMPILE_FLAGS "-Wno-maybe-uninitialized -Wimplicit-function-declaration")

