cmake_minimum_required( VERSION 2.8 )
project(OpenSlam)

include(CMakeToolsHelpers OPTIONAL)
set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )
# 添加Pangolin依赖
find_package( Pangolin )
#添加opencv
find_package(OpenCV REQUIRED)
#添加g2o
find_package(G2O)
#添加Cholmod
find_package(Cholmod )
#加入链接库
include_directories(${Pangolin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${G2O_INCLUDE_DIRS} ${Cholmod_INCLUDE_DIRS})
#将src目录下的文件名称保存在MY_SRC下
aux_source_directory(./src MY_SRC)


add_executable(OpenSlam ${MY_SRC})
target_link_libraries(OpenSlam ${Pangolin_LIBRARIES} ${OpenCV_LIBS} g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba ${CHOLMOD_LIBRARIES} -lusb-1.0)
