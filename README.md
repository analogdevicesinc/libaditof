# Analog Devices 3D ToF SDK 

## Overview

The ADI ToF SDK is designed to support ADI's ToF hardware.

```
ToF Imager (ADSD3100 or ADSD3030)
            ||
            ||
    ADSD3500 (Dual or Single)
            ||
            || (MIPI for Frame Data)
            || (I2C or SPI for Control)
            ||
------------------------------------------
            |                      (Linux) 
       Host Driver
            |
            |
         libaditof


|| - Hardware Connection
 | - Software Connection
```

The SDK provides an API to control the ToF camera, AB, and depth frame streams.

License : [![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

