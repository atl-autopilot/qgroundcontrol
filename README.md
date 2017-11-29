# m4lib

Library to communicate with M4 on ST16.

It is used by [QGroundControl (DataPilot)](https://github.com/YUNEEC/qgroundcontrol) and the [Yuneec SDK](https://github.com/YUNEEC/Yuneec_SDK_Builder).

## Test Build

To do a quick test compilation of the library, do:

Without the ST16 define:
```
mkdir build && cd build
cmake ..
make
```

With the ST16 define (but still compiled natively):
```
mkdir build && cd build
cmake -DPLATFORM:STRING=Android_x86 ..
make
```
