cmake_minimum_required(VERSION 3.0.2)
project(twin_bridge)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/bridge.py
  scripts/check_streams.py
  scripts/livox_adapter.py
  scripts/validated_inputs.py
  scripts/start_fastlivo.py
  scripts/start_mavros.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
