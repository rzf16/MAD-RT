add_executable(mad_rt src/main.cpp src/hmm.cpp src/mad_rt.cpp src/smoothing.cpp src/state_validity.cpp src/util.cpp src/observation.cpp src/bimad_rt.cpp src/kd_tree.cpp src/nn_struct.cpp)
include_directories(include)
target_link_libraries(mad_rt ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS mad_rt DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
