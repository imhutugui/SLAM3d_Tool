cmake_minimum_required(VERSION 2.8.3)
set(VCPKG_ROOT "E:/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE PATH "")
set(CMAKE_TOOLCHAIN_FILE ${VCPKG_ROOT})
project(interactive_slam)

set (IS_VERSION_MAJOR 1)
set (IS_VERSION_MINOR 0)
set (IS_VERSION_REVISION 5)


set(CMAKE_CXX_STANDARD 17)

find_package(GLM REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PCL CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED filesystem program_options)
find_package(yaml-cpp CONFIG REQUIRED)

find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})

find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)

add_subdirectory(thirdparty/ros)
###########
## Build ##
###########

include_directories(
  include
  thirdparty/gl3w
  thirdparty/imgui
  thirdparty/imgui/examples
  thirdparty/portable-file-dialogs
  thirdparty/KDBindings
  thirdparty/ndt_omp/include
  thirdparty/ros/include
  thirdparty/ros/console_bridge/include
  thirdparty/ros/roscpp_core/cpp_common/include
  thirdparty/ros/roscpp_core/rostime/include
  thirdparty/ros/roscpp_core/roscpp_serialization/include
  thirdparty/ros/roscpp_core/roscpp_traits/include
  thirdparty/ros/ros_comm/tools/rosbag_storage/include
  thirdparty/ros/ros_comm/utilities/roslz4/include
  include/utils
  include/mapping/Lego_Loam/include
  include/mapping/NDT/include
  include/mapping/SC-DLO/include
  /opt/ros/melodic/include
  ${GLM_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

###########
## Build ##
###########

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_definitions(-D_USE_MATH_DEFINES -DNOMINMAX)
add_library(ndt_omp STATIC
  thirdparty/ndt_omp/src/pclomp/gicp_omp.cpp
  thirdparty/ndt_omp/src/pclomp/ndt_omp.cpp
  thirdparty/ndt_omp/src/pclomp/voxel_grid_covariance_omp.cpp
)

# imgui
add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GL3W)

###########
## Build ##
###########

add_library(imgui STATIC
  thirdparty/gl3w/gl3w.c
  thirdparty/imgui/imgui.cpp
  thirdparty/imgui/imgui_demo.cpp
  thirdparty/imgui/imgui_draw.cpp
  thirdparty/imgui/imgui_widgets.cpp
  thirdparty/imgui/examples/imgui_impl_glfw.cpp
  thirdparty/imgui/examples/imgui_impl_opengl3.cpp
)
target_link_libraries(imgui
  ${OPENGL_LIBRARIES}
  glfw
)


add_library(guik STATIC
  src/glk/mesh.cpp
  src/glk/lines.cpp
  src/glk/colormap.cpp
  src/glk/glsl_shader.cpp
  src/glk/frame_buffer.cpp
  src/glk/pointcloud_buffer.cpp
  src/glk/primitives/primitives.cpp
  src/glk/loaders/ply_loader.cpp
  src/guik/gl_canvas.cpp
  src/guik/model_control.cpp
  src/guik/camera_control.cpp
  src/guik/projection_control.cpp
  src/guik/imgui_application.cpp
)

###########
## Build ##
###########
add_library(ndt STATIC
  src/mapping/NDT/src/ndt_mapping.cpp
)

target_link_libraries(ndt
    ${PCL_LIBRARIES} 
    Eigen3::Eigen
)

add_library(lego_loam STATIC
  src/mapping/Lego_Loam/src/featureAssociation.cpp
  src/mapping/Lego_Loam/src/imageProjection.cpp
  src/mapping/Lego_Loam/src/mapOptmization.cpp 
  src/mapping/Lego_Loam/src/Scancontext.cpp 
  src/mapping/Lego_Loam/src/utility.cpp
  src/mapping/Lego_Loam/src/dumpGraph.cpp
)

target_link_libraries(lego_loam PRIVATE
    ${PCL_LIBRARIES} 
    ${OpenCV_LIBRARIES} 
    Eigen3::Eigen
    # ${GTSAM_LIBRARIES}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3 /bigobj")

add_library(nanoflann STATIC
    src/mapping/SC-DLO/src/nano_gicp/nanoflann.cc 
)
target_link_libraries(nanoflann ${PCL_LIBRARIES})
target_include_directories(nanoflann PUBLIC include ${PCL_INCLUDE_DIRS})

add_library(nano_gicp STATIC 
    src/mapping/SC-DLO/src/nano_gicp/lsq_registration.cc 
    src/mapping/SC-DLO/src/nano_gicp/nano_gicp.cc
)
target_link_libraries(nano_gicp ${PCL_LIBRARIES} ${OpenMP_LIBS} nanoflann)
target_include_directories(nano_gicp PUBLIC include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(sc_dlio STATIC 
    src/mapping/SC-DLO/src/dlo/odom.cc 
    src/mapping/SC-DLO/src/Scancontext.cpp 
    src/mapping/SC-DLO/src/laserPosegraphOptimization.cpp
)
target_link_libraries(sc_dlio PRIVATE ${PCL_LIBRARIES} ${OpenMP_LIBS} gtsam nanoflann nano_gicp yaml-cpp::yaml-cpp)
target_include_directories(sc_dlio PUBLIC include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_executable(interactive_slam
  src/interactive_slam.cpp
  src/view/edge_view.cpp
  src/view/graph_slam.cpp
  src/view/interactive_graph.cpp
  src/view/interactive_mapping.cpp
  src/view/keyframe.cpp
  src/view/vertex_view.cpp
  src/view/information_matrix_calculator.cpp
  src/view/interactive_keyframe.cpp
  src/g2o/robust_kernel_io.cpp
  src/view/window/registration_methods.cpp
  src/view/window/version_modal.cpp
  src/view/window/graph_edit_window.cpp
  src/view/window/edge_refinement_window.cpp
  src/view/window/plane_detection_window.cpp
  src/view/window/plane_alignment_modal.cpp
  src/view/window/manual_loop_close_modal.cpp
  src/view/window/automatic_loop_close_window.cpp
)
target_link_libraries(interactive_slam
  ${PCL_LIBRARIES}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM3D_ADDONS}
  ${OPENGL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${BZIP2_LIBRARIES} ${lz4_LIBRARIES}
  glfw
  guik
  imgui
  ndt_omp
#   lego_loam
  sc_dlio
  ndt
  rosbag
  g2o::core g2o::stuff g2o::types_icp g2o::types_sba g2o::types_slam3d g2o::types_slam3d_addons g2o::solver_pcg g2o::solver_csparse g2o::solver_cholmod
)

file(COPY data DESTINATION .)
