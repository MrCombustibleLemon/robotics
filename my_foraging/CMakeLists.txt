add_library(my_foraging SHARED my_foraging.h my_foraging.cpp)
target_link_libraries(my_foraging
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
