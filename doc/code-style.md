# Coding Style

We have opted to use clang-tidy with the LLVM option for our coding style.

1. sudo apt install clang-tidy
2. Build the code (see the instructions on the main page). This can be either the camera kit code or libaditof.
3. Scan all SDK code:
```
run-clang-tidy libaditof/sdk/ -p build -checks=llvm-* 2>&1 | tee clang-tidy.log
```
   Or scan a file:
```
clang-tidy libaditof/sdk/src/cameras/itof-camera/camera_itof.cpp -p build -checks=llvm-*
```
