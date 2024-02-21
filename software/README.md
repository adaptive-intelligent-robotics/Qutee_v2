# Building instructions:
## Requirements:
Please make sure that you cloned this repository with the --recursive option: 
`git clone --recursive https://github.com/adaptive-intelligent-robotics/Qutee_v2.git`
(this will take some times as it will also downloard the arduino-esp32 library, which is a few GB large).

The rest of this Readme assumes that you are in the `Qutee_v2/software` folder. 

## Docker build environment:
Build the docker container qutee_idf that we use as a build environment: 
```
docker build -t qutee_idf Docker/.
```

Then, launch and shell into the qutee_idf container: 
```
docker run -it --rm --user espidf --volume="/etc/timezone:/etc/timezone:ro" -v $(pwd):/micro_ros_espidf_component -v /dev:/dev --privileged --workdir /micro_ros_espidf_component qutee_idf /bin/bash
```

To build the Qutee's firmware now that you are inside the container:
```
idf.py build
```

Successful builds will show at the end: `Successfully created esp32s3 image.`

## Configure the Qutee firmware:

Still inside the container, do 'idf.py menuconfig' to open the configuration menu.
You can for instance, change the name of your Qutee in `QUTEE settings -> Name of the Qutee robot`
You can also change the wifi credentials in `micro-ROS Settings -> WiFi Configuration`

Don't forget to rebuild the firmware after any change in the configuration. 

## To flash:
To flash your robot: 
```
python $IDF_PATH/components/esptool_py/esptool/esptool.py -p (PORT) -b 460800 --before default_reset --after hard_reset --chip esp32s3  write_flash --flash_mode dio --flash_size 2MB --flash_freq 40m 0x0 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/QuteeV2.bin
```
You have to replace **(PORT)** with the port you want to use, such as `/dev/cu.usbmodem1201`



## To create new micro-ros messages: 
```
docker run -it --rm --volume="/etc/timezone:/etc/timezone:ro" -v  $(pwd):/micro_ros_espidf_component -v  /dev:/dev --privileged --workdir /micro_ros_espidf_component --entrypoint /bin/bash microros/micro-ros-agent:humble
```
 
and then follow: 
https://micro.ros.org/docs/tutorials/advanced/create_new_type/

## In case the build fails:
Try to remove all the files generated during the build process:
```
rm -rf ./build ./main/build ./components/micro_ros_espidf_component/micro_ros_*
```
