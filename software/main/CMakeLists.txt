
if(CMAKE_GENERATOR STREQUAL "Unix Makefiles")
set(submake "$(MAKE)")
else()
set(submake "make")
endif()

if (${IDF_VERSION_MAJOR} EQUAL 5)
    set(CMAKE_C_STANDARD 17)
else()
    set(CMAKE_C_STANDARD 11)
endif()

set(APP_COLCON_META "${PROJECT_DIR}/app-colcon.meta")
if(NOT EXISTS "${APP_COLCON_META}")
set(APP_COLCON_META "")
endif()

set(EXTRA_ROS_PACKAGES "${PROJECT_DIR}/extra_ros_packages")
if(NOT EXISTS "${EXTRA_ROS_PACKAGES}")
set(EXTRA_ROS_PACKAGES "${COMPONENT_DIR}/extra_packages")
endif()

if(${CONFIG_MICRO_ROS_ESP_EMBEDDEDRTPS_MIDDLEWARE})
message(STATUS "Using embeddedRTPS middleware")
set(MIDDLEWARE "embeddedrtps")
else()
message(STATUS "Using Micro XRCE-DDS middleware")
set(MIDDLEWARE "microxrcedds")
endif()

# Create a list of packages to get includes
idf_build_get_property(PACKAGES BUILD_COMPONENTS)

# Add includes for each package
foreach(package ${PACKAGES})
    message([AUTHOR_WARNING] "BLA BLA lsit: ${package}")
    idf_component_get_property(${package}_dir ${package} COMPONENT_DIR)
    idf_component_get_property(${package}_includes ${package} INCLUDE_DIRS)
    list(TRANSFORM ${package}_includes PREPEND " -I${${package}_dir}/")
    string(REPLACE ";" "" ${package}_includes_string "${${package}_includes}")
endforeach()

# Create a string with all packages with a for loop
foreach(package ${PACKAGES})
    string(APPEND IDF_INCLUDES "${${package}_includes_string}")
endforeach()

string(APPEND IDF_INCLUDES " -I/micro_ros_espidf_component/components/micro_ros_espidf_component/include/std_msgs/") #ugly hack, but it works. 

message([AUTHOR_WARNING] "BLA BLA include: ${IDF_INCLUDES}")

execute_process(
    WORKING_DIRECTORY ${COMPONENT_DIR}
    COMMAND
        ${submake} -j -f libquteeinterface.mk 
                X_CC=${CMAKE_C_COMPILER}
                X_AR=${CMAKE_AR}
                X_STRIP=${CMAKE_STRIP}
                X_CFLAGS=${CMAKE_C_FLAGS}
                X_CXX=${CMAKE_CXX_COMPILER}
                X_CXXFLAGS=${CMAKE_CXX_FLAGS}
                C_STANDARD=${CMAKE_C_STANDARD}
                MIDDLEWARE=${MIDDLEWARE}
                BUILD_DIR=${CMAKE_BINARY_DIR}
                IDF_INCLUDES=${IDF_INCLUDES}
                IDF_PATH=${IDF_PATH}
                IDF_TARGET=${IDF_TARGET}
                APP_COLCON_META=${APP_COLCON_META}
                IDF_VERSION_MAJOR=${IDF_VERSION_MAJOR}
                IDF_VERSION_MINOR=${IDF_VERSION_MINOR}
                EXTRA_ROS_PACKAGES=${EXTRA_ROS_PACKAGES}
)


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
                            "../components/Adafruit_MAX1704X/Adafruit_MAX1704X.cpp"
 

                    INCLUDE_DIRS "../components/Dynamixel2Arduino/src/." 
                                "../components/Dynamixel2Arduino/src/utility/." 
                                "../components/Dynamixel2Arduino/src/dxl_c/."
                                "../components/Adafruit-ST7735-Library/"
                                "../components/Adafruit_Seesaw/"
                                "../components/Adafruit_Sensor/"
                                "../components/Adafruit_BNO055/"
                                "../components/Adafruit_MAX1704X/"
                 



                    REQUIRES "arduino" # Library requires Arduino
                             "eigen"
                             "micro_ros_espidf_component"
                             "Adafruit-GFX-Library"
                             "Adafruit_BusIO"

                    )
set_target_properties(${COMPONENT_LIB} PROPERTIES COMPILE_FLAGS "-Wno-maybe-uninitialized -Wimplicit-function-declaration -fpermissive")


add_prebuilt_library(libquteeinterface ""
                        REQUIRES micro_ros_espidf_component)

set_target_properties(libquteeinterface PROPERTIES IMPORTED_LOCATION ${COMPONENT_DIR}/libquteeinterface.a)
target_include_directories(libquteeinterface INTERFACE ${COMPONENT_DIR}/include)




file(GLOB include_folders LIST_DIRECTORIES true CONFIGURE_DEPENDS ${COMPONENT_DIR}/include/*)
foreach(include_folder ${include_folders})
    message([AUTHOR_WARNING] "${include_folder}")
    if(IS_DIRECTORY ${include_folder})
        get_filename_component(folder_name ${include_folder} NAME)
        target_include_directories(libquteeinterface INTERFACE ${COMPONENT_DIR}/include/${folder_name})
        target_include_directories(${COMPONENT_LIB} PRIVATE ${COMPONENT_DIR}/include/${folder_name}) # I don't get why this is not directly propagated to COMPONENT_LIB when I target_link_libraries, so doing it manually, even if ugly.
    endif()
endforeach()

add_dependencies(${COMPONENT_LIB} libquteeinterface)
target_link_libraries(${COMPONENT_LIB} libquteeinterface)

include(CMakePrintHelpers)
cmake_print_properties(TARGETS libquteeinterface ${COMPONENT_LIB} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES INCLUDE_DIRECTORIES)

set_directory_properties( PROPERTIES ADDITIONAL_CLEAN_FILES
    "${COMPONENT_DIR}/include;${COMPONENT_DIR}/install;${COMPONENT_DIR}/log;" )

add_custom_target(clean-quteeinterface COMMAND ${submake} -f libquteeinterface.mk clean
                            WORKING_DIRECTORY ${COMPONENT_DIR})
