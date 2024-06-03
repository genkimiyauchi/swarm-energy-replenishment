include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})

add_library(leaderworker_loop_functions MODULE 
  manualcontrol_qtuser_functions_nop.h
  manualcontrol_qtuser_functions_nop.cpp
  experiment_loop_functions_nop.h
  experiment_loop_functions_nop.cpp)

target_link_libraries(leaderworker_loop_functions
  utility
  worker
  charger
  protos
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_epuck
  argos3plugin_simulator_epuckcharger
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})
