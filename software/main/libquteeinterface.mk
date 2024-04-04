MICROROS_DIR = $(shell pwd)/../components/micro_ros_espidf_component/
UROS_DIR = $(MICROROS_DIR)/micro_ros_src
BUILD_DIR ?= $(MICROROS_DIR)/build

QUTEE_DIR = $(shell pwd)/

DEBUG ?= 0

ifeq ($(DEBUG), 1)
	BUILD_TYPE = Debug
else
	BUILD_TYPE = Release
endif

CFLAGS_INTERNAL := $(X_CFLAGS) -ffunction-sections -fdata-sections
CXXFLAGS_INTERNAL := $(X_CXXFLAGS) -ffunction-sections -fdata-sections

all: $(QUTEE_DIR)/libquteeinterface.a

clean:
	rm -rf $(QUTEE_DIR)/libquteeinterface.a; \
	rm -rf $(QUTEE_DIR)/include; \




$(QUTEE_DIR)/install:
	cd $(QUTEE_DIR); \
	unset AMENT_PREFIX_PATH; \
	PATH="$(subst /opt/ros/$(ROS_DISTRO)/bin,,$(PATH))"; \
	. $(MICROROS_DIR)/micro_ros_dev/install/local_setup.sh; \
	. $(MICROROS_DIR)/micro_ros_src/install/local_setup.sh; \
	colcon build \
		--merge-install \
		--packages-ignore-regex=.*_cpp \
		--metas $(QUTEE_DIR)/colcon.meta $(APP_COLCON_META) \
		--cmake-args \
		"--no-warn-unused-cli" \
		-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=OFF \
		-DTHIRDPARTY=ON \
		-DBUILD_SHARED_LIBS=OFF \
		-DBUILD_TESTING=OFF \
		-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
		-DCMAKE_TOOLCHAIN_FILE=$(MICROROS_DIR)/esp32_toolchain.cmake \
		-DCMAKE_VERBOSE_MAKEFILE=OFF \
        -DIDF_INCLUDES='${IDF_INCLUDES}' \
		-DCMAKE_C_STANDARD=$(C_STANDARD) \
		-DUCLIENT_C_STANDARD=$(C_STANDARD);

$(QUTEE_DIR)/libquteeinterface.a: $(QUTEE_DIR)/install
	mkdir -p $(QUTEE_DIR)/libquteeinterface; cd $(QUTEE_DIR)/libquteeinterface; \
	for file in $$(find $(QUTEE_DIR)/install/lib/ -name '*.a'); do \
		folder=$$(echo $$file | sed -E "s/(.+)\/(.+).a/\2/"); \
		mkdir -p $$folder; cd $$folder; $(X_AR) x $$file; \
		for f in *; do \
			mv $$f ../$$folder-$$f; \
		done; \
		cd ..; rm -rf $$folder; \
	done ; \
	$(X_AR) rc -s libquteeinterface.a *.obj; cp libquteeinterface.a $(QUTEE_DIR); \
	cd ..; rm -rf libquteeinterface; \
	cp -R $(QUTEE_DIR)/install/include $(QUTEE_DIR)/include;

