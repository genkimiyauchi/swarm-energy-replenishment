add_library(worker SHARED 
  worker.h 
  worker.cpp)
target_link_libraries(worker
  utility
  argos3core_simulator
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot)
