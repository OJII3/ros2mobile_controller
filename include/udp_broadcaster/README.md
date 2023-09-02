# UDP Broadcaster

C++ class for ROS2 that broadcasts UDP packets to a given port in a given interval.

Place this class in another package.

## Example usage with `ament_cmake_auto`

```cmake
cmake_minimum_required(VERSION 3.8)
project(package_name)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()
- ament_auto_add_library(${PROJECT_NAME} SHARED src/package_name.cpp)
+ ament_auto_add_library(${PROJECT_NAME} SHARED src/package_name.cpp src/udp_broadcaster.cpp)
ament_auto_add_executable(${PROJECT_NAME}_bin src/main.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
```
