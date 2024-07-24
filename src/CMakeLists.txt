# add_executable(simviz simviz.cpp)
# target_link_libraries(simviz ${KUKA_TELEOP_COMMON_LIBRARIES})

add_library(yamlLoader yamlLoader.cpp)

add_executable(test controller_test.cpp)
target_link_libraries(test yamlLoader ${KUKA_TELEOP_COMMON_LIBRARIES})

add_executable(simviz_force simviz_force.cpp ${IMGUI_SOURCES})
target_link_libraries(simviz_force yamlLoader ${KUKA_TELEOP_COMMON_LIBRARIES})

add_executable(viewdata viewdataGUI.cpp ${IMGUI_SOURCES})
target_link_libraries(viewdata yamlLoader ${KUKA_TELEOP_COMMON_LIBRARIES} ${OpenGL_LIBRARIES})

# add_executable(yamlLoader yamlLoader.cpp)
# target_link_libraries(yamlLoader ${KUKA_TELEOP_COMMON_LIBRARIES})

# add_executable(eigen_test eigen_test.cpp)