### Windows Build

Note, while it is possible to build for Windows, it maybe deprecated in the near future.

#### Dependencies

In addition the depth compute libraries are required. 

You can get the two library from the ADCAM release software, but please note in which case it is under an evaluation license.

For a non-eval license please contact us at *tof@analog.com*.

These libraries must be in a folder called **libs** that in one level below the cloned ADCAM repo folder. For example:
```
c:\dev\ADCAM> tree ../libs
../libs
├── libtofi_compute.dll
└── libtofi_config.dll
```

```
git clone https://github.com/analogdevicesinc/libaditof
cd libaditof
git submodule update --init
git checkout <branch or tag>
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 17 2022" ..
cmake --build . --config RELEASE -j 8
```
