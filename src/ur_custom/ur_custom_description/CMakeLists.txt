cmake_minimum_required(VERSION 3.8)
project(ur_custom_description)

# Check if the compiler being used is GNU's C++ compiler (g++) or Clang.
# Add compiler flags for all targets that will be defined later in the 
# CMakeLists file. These flags enable extra warnings to help catch
# potential issues in the code.
# Add options to the compilation process
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Locate and configure packages required by the project.
find_package(ament_cmake REQUIRED)
find_package(robotiq_description REQUIRED)
find_package(ur_description REQUIRED)
find_package(xacro REQUIRED)
find_package(ros_gz_sim REQUIRED)

# Copy necessary files to designated locations in the project
#install (
#        DIRECTORY urdf meshes launch config sdf
#        DESTINATION share/${PROJECT_NAME}
#)

# Define paths for input and output files
set(XACRO_FILE "${CMAKE_CURRENT_SOURCE_DIR}/urdf/ur3e_onrobot_2fg7_urdf.xacro")
set(URDF_FILE "${CMAKE_BINARY_DIR}/urdf/ur3e_onrobot_2fg7.urdf")
set(SDF_FILE "${CMAKE_BINARY_DIR}/sdf/ur3e_onrobot_2fg7.sdf")

add_custom_command(
        OUTPUT ${URDF_FILE}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/urdf
        COMMAND xacro ${XACRO_FILE} -o ${URDF_FILE}
        DEPENDS ${XACRO_FILE} ${CMAKE_CURRENT_SOURCE_DIR}/config/ur3e_controllers.yaml
        COMMENT "Generating URDF from XACRO"
)

# Generate SDF from URDF
add_custom_command(
        OUTPUT ${SDF_FILE}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/sdf
        COMMAND gz sdf -p ${URDF_FILE} > ${SDF_FILE}
        #COMMAND gz sdf -p ${URDF_FILE} > ${SDF_FILE}.tmp
        #COMMAND sed 's|model://|package://|g' ${SDF_FILE}.tmp > ${SDF_FILE}
        #COMMAND ${CMAKE_COMMAND} -E remove ${SDF_FILE}.tmp
        DEPENDS ${URDF_FILE}
        COMMENT "Generating SDF from URDF"
)

# Create ~/.gz/models/ and copy meshes
add_custom_command(
        OUTPUT ${CMAKE_BINARY_DIR}/gz_models.stamp
        COMMAND ${CMAKE_COMMAND} -E make_directory ~/.gz/models/ur_custom_description/meshes
        COMMAND ${CMAKE_COMMAND} -E make_directory ~/.gz/models/ur_custom_description/sdf
        COMMAND ${CMAKE_COMMAND} -E make_directory ~/.gz/models/ur_description/meshes/ur3e/visual
        COMMAND ${CMAKE_COMMAND} -E make_directory ~/.gz/models/ur_description/meshes/ur3e/collision
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/meshes ~/.gz/models/ur_custom_description/meshes
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_BINARY_DIR}/sdf ~/.gz/models/ur_custom_description/sdf
        COMMAND ${CMAKE_COMMAND} -E copy_directory /opt/ros/jazzy/share/ur_description/meshes/ur3e/visual ~/.gz/models/ur_description/meshes/ur3e/visual
        COMMAND ${CMAKE_COMMAND} -E copy_directory /opt/ros/jazzy/share/ur_description/meshes/ur3e/collision ~/.gz/models/ur_description/meshes/ur3e/collision
        COMMAND sh -c "grep -q 'export GZ_SIM_RESOURCE_PATH=~/.gz/models' ~/.bashrc || echo 'export GZ_SIM_RESOURCE_PATH=~/.gz/models' >> ~/.bashrc"
        COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_BINARY_DIR}/gz_models.stamp
        DEPENDS ${SDF_FILE} ${CMAKE_CURRENT_SOURCE_DIR}/meshes
        COMMENT "Setting up Gazebo models in ~/.gz/models/ and GZ_SIM_RESOURCE_PATH"
)

# Custom target to ensure URDF and SDF are built
add_custom_target(generate_models ALL
        DEPENDS ${URDF_FILE} ${SDF_FILE} ${CMAKE_BINARY_DIR}/gz_models.stamp
)

# Install generated files
install(FILES
        ${URDF_FILE}
        DESTINATION share/${PROJECT_NAME}/urdf
)
install(FILES
        ${SDF_FILE}
        DESTINATION share/${PROJECT_NAME}/sdf
)

# Install directories
install(DIRECTORY urdf meshes config sdf launch
        DESTINATION share/${PROJECT_NAME}
        PATTERN "*.xacro" EXCLUDE
)


ament_package()


