## To build ##
Start a docker container:
'docker run -it --rm --user espidf --volume="/etc/timezone:/etc/timezone:ro" -v  $(pwd):/micro_ros_espidf_component -v  /dev:/dev --privileged --workdir /micro_ros_espidf_component microros/esp-idf-microros:foxy /bin/bash'

Then, do 'idf.py menuconfig', set the configuration for your robot (more to come later), finally build: 'idf.py build'. 
To flash your robot you can do: 
'/opt/esp/python_env/idf5.0_py3.8_env/bin/'python ../opt/esp/idf/components/esptool_py/esptool/esptool.py -p (PORT) -b 460800 --before default_reset --after hard_reset --chip esp32s3  write_flash --flash_mode dio --flash_size 2MB --flash_freq 40m 0x0 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/QuteeV2.bin' You have to replace (PORT) with the port you want to use. 



To create new message: 
docker run -it --rm --volume="/etc/timezone:/etc/timezone:ro" -v  $(pwd):/micro_ros_espidf_component -v  /dev:/dev --privileged --workdir /micro_ros_espidf_component --entrypoint /bin/bash microros/micro-ros-agent:humble
 
 and then follow: 
 https://micro.ros.org/docs/tutorials/advanced/create_new_type/