cmake_minimum_required(VERSION 3.8)
project(baseline_robot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config scenarios DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS scripts/navigation_path.py scripts/gps_truth.py scripts/sensor_adapter.py
           scripts/evaluate_navigation.py scripts/evaluate_uncertainty.py scripts/usbl_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES scripts/sensor_uncertainty.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(navigation_sensors test/test_navigation_sensors.py)
  ament_add_pytest_test(usbl test/test_usbl.py)
  ament_add_pytest_test(usbl_ekf test/test_usbl_ekf.py TIMEOUT 45)
  ament_add_pytest_test(sensor_uncertainty test/test_sensor_uncertainty.py)
endif()

ament_package()
