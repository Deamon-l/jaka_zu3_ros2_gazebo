cmake_minimum_required(VERSION 3.22)
project(jaka_zu3_moveit_config)

find_package(ament_cmake REQUIRED)

# 1. 先把所有的安装（Install）规则写好，告诉系统要复制什么
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# 2. 【核心修改】最后再放这句“封箱”命令！它必须在所有 install 的最下方
ament_package()