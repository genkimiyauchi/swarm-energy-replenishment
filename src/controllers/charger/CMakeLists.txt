add_library(charger SHARED 
  charger.h 
  charger.cpp)
target_link_libraries(charger
  utility
  argos3core_simulator
  argos3plugin_simulator_epuckcharger
  argos3plugin_simulator_genericrobot)
