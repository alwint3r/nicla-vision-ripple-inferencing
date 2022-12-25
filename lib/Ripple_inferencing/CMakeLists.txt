file(GLOB EDGEIMPULSE_INFERENCING_SRCS 
    src/*.cpp
    src/**/*.cpp
    src/*.c
    src/**/*.c
    src/edge-impulse-sdk/*.cpp
    src/edge-impulse-sdk/*.c
    src/edge-impulse-sdk/**/*.cpp
    src/edge-impulse-sdk/**/*.c
    src/edge-impulse-sdk/dsp/*.cpp
    src/edge-impulse-sdk/dsp/*.c
    src/edge-impulse-sdk/dsp/**/*.cpp
    src/edge-impulse-sdk/dsp/**/*.c
    src/edge-impulse-sdk/anomaly/*.cpp
    src/edge-impulse-sdk/anomaly/*.c
    src/edge-impulse-sdk/anomaly/**/*.cpp
    src/edge-impulse-sdk/anomaly/**/*.c
    src/edge-impulse-sdk/porting/*.cpp
    src/edge-impulse-sdk/porting/*.c
    src/edge-impulse-sdk/porting/**/*.cpp
    src/edge-impulse-sdk/porting/**/*.c
    src/edge-impulse-sdk/porting/arduino/*.cpp
    src/edge-impulse-sdk/porting/arduino/*.c
    src/edge-impulse-sdk/porting/arduino/**/*.cpp
    src/edge-impulse-sdk/porting/arduino/**/*.c
    src/edge-impulse-sdk/CMSIS/*.cpp
    src/edge-impulse-sdk/CMSIS/*.c
    src/edge-impulse-sdk/CMSIS/**/*.cpp
    src/edge-impulse-sdk/CMSIS/**/*.c
    src/edge-impulse-sdk/CMSIS/Core/*.cpp
    src/edge-impulse-sdk/CMSIS/Core/*.c
    src/edge-impulse-sdk/CMSIS/Core/**/*.cpp
    src/edge-impulse-sdk/CMSIS/Core/**/*.c
    src/edge-impulse-sdk/CMSIS/DSP/Source/*.cpp
    src/edge-impulse-sdk/CMSIS/DSP/Source/*.c
    src/edge-impulse-sdk/CMSIS/DSP/Source/**/*.cpp
    src/edge-impulse-sdk/CMSIS/DSP/Source/**/*.c
    src/edge-impulse-sdk/CMSIS/NN/Source/*.cpp
    src/edge-impulse-sdk/CMSIS/NN/Source/*.c
    src/edge-impulse-sdk/CMSIS/NN/Source/**/*.cpp
    src/edge-impulse-sdk/CMSIS/NN/Source/**/*.c
    src/edge-impulse-sdk/tensorflow/lite/*.cpp
    src/edge-impulse-sdk/tensorflow/lite/*.c
    src/edge-impulse-sdk/tensorflow/lite/**/*.cpp
    src/edge-impulse-sdk/tensorflow/lite/**/*.c
    src/edge-impulse-sdk/tensorflow/lite/c/*.cpp
    src/edge-impulse-sdk/tensorflow/lite/c/*.c
    src/edge-impulse-sdk/tensorflow/lite/c/**/*.cpp
    src/edge-impulse-sdk/tensorflow/lite/c/**/*.c
    src/edge-impulse-sdk/tensorflow/lite/core/api/*.cpp
    src/edge-impulse-sdk/tensorflow/lite/core/api/*.c
    src/edge-impulse-sdk/tensorflow/lite/core/api/**/*.cpp
    src/edge-impulse-sdk/tensorflow/lite/core/api/**/*.c
    src/edge-impulse-sdk/tensorflow/lite/kernels/internal/*.cpp
    src/edge-impulse-sdk/tensorflow/lite/kernels/internal/*.c
    src/edge-impulse-sdk/tensorflow/lite/kernels/internal/**/*.cpp
    src/edge-impulse-sdk/tensorflow/lite/kernels/internal/**/*.c
    src/edge-impulse-sdk/tensorflow/lite/micro/*.cpp
    src/edge-impulse-sdk/tensorflow/lite/micro/*.c
    src/edge-impulse-sdk/tensorflow/lite/micro/**/*.cpp
    src/edge-impulse-sdk/tensorflow/lite/micro/**/*.c
    src/edge-impulse-sdk/tensorflow/lite/micro/kernels/*.cpp
    src/edge-impulse-sdk/tensorflow/lite/micro/kernels/*.c
    src/edge-impulse-sdk/tensorflow/lite/micro/kernels/**/*.cpp
    src/edge-impulse-sdk/tensorflow/lite/micro/kernels/**/*.c
    src/edge-impulse-sdk/tensorflow/lite/micro/memory_planner/*.cpp
    src/edge-impulse-sdk/tensorflow/lite/micro/memory_planner/*.c
    src/edge-impulse-sdk/tensorflow/lite/micro/memory_planner/**/*.cpp
    src/edge-impulse-sdk/tensorflow/lite/micro/memory_planner/**/*.c
)

add_library(EdgeImpulseInferencing ${EDGEIMPULSE_INFERENCING_SRCS})

target_compile_definitions(EdgeImpulseInferencing PRIVATE ${ARDUINO_COMPILE_DEFINITIONS})

target_compile_options(EdgeImpulseInferencing PRIVATE
    $<$<COMPILE_LANGUAGE:C>:-std=gnu11>
    $<$<COMPILE_LANGUAGE:CXX>:-Wvla>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++14>

    ${ARDUINO_COMPILE_OPTIONS}
)

set(EDGEIMPULSE_INFERENCING_INCLUDE_DIRS
    ${ARDUINO_INCLUDE_DIRS}
    ${CMAKE_CURRENT_LIST_DIR}/src
)

# Append BLE library include back to ARDUINO_INCLUDE_DIRS at parent scope
set(ARDUINO_INCLUDE_DIRS
    "${EDGEIMPULSE_INFERENCING_INCLUDE_DIRS}"
    PARENT_SCOPE
)

target_include_directories(EdgeImpulseInferencing PRIVATE
    ${EDGEIMPULSE_INFERENCING_INCLUDE_DIRS}
)

