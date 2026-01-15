# Platform configuration for ADCAM SDK
# Sets platform-specific variables for generating platform_config.h

message(STATUS "Platform configuration: NVIDIA=${NVIDIA}, RPI=${RPI}, NXP=${NXP}")

if(NVIDIA)
    set(PLATFORM_NAME "NVIDIA Jetson Orin Nano")
    set(PLATFORM_CAPTURE_DEVICE "vi-output, adsd3500")
    set(PLATFORM_VIDEO_PREFIX "vi-output")
    set(PLATFORM_MEDIA_CONTROLLER "/dev/media")
    set(PLATFORM_RESET_GPIO_NAME "PAC.00")
    set(PLATFORM_RESET_GPIO_PIN 0)  # Not used for NVIDIA (uses named GPIO)
    message(STATUS "Configured for NVIDIA Jetson platform")
    
elseif(RPI)
    set(PLATFORM_NAME "Raspberry Pi 5")
    set(PLATFORM_CAPTURE_DEVICE "rp1-cfe")
    set(PLATFORM_VIDEO_PREFIX "rp1-cfe")
    set(PLATFORM_MEDIA_CONTROLLER "/dev/media")
    set(PLATFORM_RESET_GPIO_NAME "")  # Empty - will be resolved dynamically from debugfs
    set(PLATFORM_RESET_GPIO_PIN 34)   # Physical pin number for debugfs lookup
    message(STATUS "Configured for Raspberry Pi platform, GPIO Pin=${PLATFORM_RESET_GPIO_PIN}")
    
else()
    message(FATAL_ERROR "No platform defined! Use -DNVIDIA=ON or -DRPI=ON")
endif()

# Generate platform_config.h from template
# Note: When included from libaditof/sdk/CMakeLists.txt, CMAKE_CURRENT_BINARY_DIR = build/libaditof/sdk
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/platform/platform_config.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/platform_config.h
    @ONLY
)
