add_library(VL53L0X_Lib STATIC
    src/VL53L0X.cpp
)

#add target libraries
target_link_libraries(VL53L0X_Lib pico_stdlib hardware_i2c)
target_include_directories(VL53L0X_Lib PUBLIC inc/)