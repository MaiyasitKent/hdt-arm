cmake_minimum_required(VERSION 3.8)
project(hdt_arm_description)

find_package(ament_cmake REQUIRED)

# ติดตั้งทุก directory ที่จำเป็น
install(DIRECTORY
  config
  meshes
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# ติดตั้ง rviz (optional — สร้างก็ต่อเมื่อมี folder)
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
  OPTIONAL
)

# ติดตั้ง launch files ถ้ามี
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

ament_package()