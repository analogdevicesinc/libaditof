# Platform configuration for ADCAM SDK
# Sets platform-specific variables for generating platform_config.h

message(STATUS "Platform configuration: NVIDIA=${NVIDIA}, RPI=${RPI}, NXP=${NXP}")

if(NVIDIA)
    set(PLATFORM_NAME "NVIDIA Jetson Orin Nano")
    set(PLATFORM_CAPTURE_DEVICE "vi-output, adsd3500")
    set(PLATFORM_VIDEO_PREFIX "vi-output")
    set(PLATFORM_MEDIA_CONTROLLER "/dev/media")
    set(PLATFORM_RESET_GPIO "PAC.00")
    set(PLATFORM_RESET_GPIO_PIN 0)  # Not used for NVIDIA (uses named GPIO)
    set(PLATFORM_RESET_PULSE_US 100000)   # 100ms low pulse
    set(PLATFORM_RESET_DELAY_US 10000000) # 10s default delay
    message(STATUS "Configured for NVIDIA Jetson platform")
    
elseif(RPI)
    set(PLATFORM_NAME "Raspberry Pi 5")
    set(PLATFORM_CAPTURE_DEVICE "rp1-cfe")
    set(PLATFORM_VIDEO_PREFIX "rp1-cfe")
    set(PLATFORM_MEDIA_CONTROLLER "/dev/media")
    set(PLATFORM_RESET_GPIO "")  # Empty - will be resolved dynamically from debugfs
    set(PLATFORM_RESET_GPIO_PIN 34)   # Physical pin number for debugfs lookup
    set(PLATFORM_RESET_PULSE_US 100000)  # 100ms low pulse
    set(PLATFORM_RESET_DELAY_US 2000000) # 2s default delay
    message(STATUS "Configured for Raspberry Pi platform ")
    
elseif(NXP)
    set(PLATFORM_NAME "NXP i.MX 8")
    set(PLATFORM_CAPTURE_DEVICE "mxc_isi")
    set(PLATFORM_VIDEO_PREFIX "mxc_isi")
    set(PLATFORM_MEDIA_CONTROLLER "/dev/media")
    set(PLATFORM_RESET_GPIO "gpio64") 
    set(PLATFORM_RESET_GPIO_PIN 0)    # Not used for NXP
    set(PLATFORM_RESET_PULSE_US 1000000) # 1s low pulse
    set(PLATFORM_RESET_DELAY_US 7000000) # 7s default delay
    message(STATUS "Configured for NXP i.MX platform ")
    
else()
    message(FATAL_ERROR "No platform defined! Use -DNVIDIA=ON, -DRPI=ON, or -DNXP=ON")
endif()

# Generate platform_config.h from template
# Note: When included from libaditof/sdk/CMakeLists.txt, CMAKE_CURRENT_BINARY_DIR = build/libaditof/sdk
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/platform/platform_config.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/platform_config.h
    @ONLY
)
