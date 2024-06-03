add_library(utility SHARED
    pid.h
    pid.cpp
    sct.h
    sct.cpp
    robot_message.h
    robot_message.cpp
    custom_battery_discharge_model.h
    custom_battery_discharge_model.cpp)
target_link_libraries(utility
    yaml-cpp
    argos3plugin_simulator_genericrobot)
