############################################################################## 
# Sources 
############################################################################## 

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp) 


############################################################################## 
# Targets 
############################################################################## 

add_executable(trobot_node ${SOURCES})
add_dependencies(trobot_node trobot_generate_messages_cpp)
target_link_libraries(trobot_node ${catkin_LIBRARIES})



