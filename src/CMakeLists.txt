# add_executable(simviz simviz.cpp)
# target_link_libraries(simviz ${KUKA_TELEOP_COMMON_LIBRARIES})

add_executable(test controller_test.cpp)
target_link_libraries(test ${KUKA_TELEOP_COMMON_LIBRARIES})

add_executable(simviz_force simviz_force.cpp ${IMGUI_SOURCES})
target_link_libraries(simviz_force ${KUKA_TELEOP_COMMON_LIBRARIES})

add_executable(viewdata viewdataGUI.cpp ${IMGUI_SOURCES})
target_link_libraries(viewdata ${KUKA_TELEOP_COMMON_LIBRARIES} ${OpenGL_LIBRARIES})
