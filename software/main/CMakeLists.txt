idf_component_register(SRCS "main.cpp" "../components/Dynamixel2Arduino/src/Dynamixel2Arduino.cpp" "../components/Dynamixel2Arduino/src/actuator.cpp" "../components/Dynamixel2Arduino/src/utility/master.cpp" "../components/Dynamixel2Arduino/src/utility/port_handler.cpp" "../components/Dynamixel2Arduino/src/utility/slave.cpp" "../components/Dynamixel2Arduino/src/dxl_c/protocol.c"
                    INCLUDE_DIRS "../components/Dynamixel2Arduino/src/." "../components/Dynamixel2Arduino/src/utility/." "../components/Dynamixel2Arduino/src/dxl_c/."
                    REQUIRES "arduino-esp32" # Library requires Arduino
                    )
set_target_properties(${COMPONENT_LIB} PROPERTIES COMPILE_FLAGS "-Wno-maybe-uninitialized -Wimplicit-function-declaration")

