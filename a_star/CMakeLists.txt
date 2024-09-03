cmake_minimum_required(VERSION 3.8)
project(fusion_libs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

message("eigen found - ${Eigen3_FOUND}")

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(fusion_lib src/fusion_lib.cpp)
target_compile_features(fusion_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(fusion_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${nav_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(fusion_lib PRIVATE "FUSION__BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS fusion_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(fusion_node src/fusion_node.cpp)
target_include_directories(fusion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(fusion_node fusion_lib)

install(TARGETS fusion_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)

  find_package(ament_cmake_gtest REQUIRED)
  
  # ament_lint_auto_find_test_dependencies()

  # message("Building test files ${BUILD_TESTING} Found ament_cmake_gtest ${ament_cmake_gtest_INCLUDE_DIRS}" )

  ament_add_gtest(test_cov_adapter
    src/test/test_cov_adapter.cpp
  )
  target_include_directories(test_cov_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_cov_adapter nav_msgs )

  ###

  ament_add_gtest(test_adaptable_matrix
  src/test/test_adaptable_e_matrix.cpp)

  target_include_directories(test_adaptable_matrix PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  )
  ament_target_dependencies(test_adaptable_matrix nav_msgs Eigen3)

  ament_add_gtest(test_adaptable_states
  src/test/test_adaptable_states.cpp)

  target_include_directories(test_adaptable_states PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  )
  ament_target_dependencies(test_adaptable_states nav_msgs Eigen3)


endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  fusion_lib
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
