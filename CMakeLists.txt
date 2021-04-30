cmake_minimum_required(VERSION 3.13)
project(company_geometry)
enable_testing()

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Boost REQUIRED COMPONENTS system log log_setup filesystem unit_test_framework program_options)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
find_package(PQXX REQUIRED)
find_package(FMT REQUIRED)

add_library(company_geometry SHARED geometry.cpp index.cpp receiver.cpp)

add_executable(test_geometry test.cpp)
# add_executable(example_rdp example_rdp.cpp)
add_executable(download_geometry download.cpp)

target_include_directories(company_geometry PUBLIC ./../libcompany_core)

target_link_libraries(company_geometry PRIVATE
  Eigen3::Eigen
  Threads::Threads
  ${Boost_LIBRARIES}
  ${PQXX_LIBRARIES}
)

target_link_libraries(download_geometry PRIVATE
  ${Boost_LIBRARIES}
  ${PQXX_LIBRARIES}
)

target_link_libraries(test_geometry PRIVATE
  company_geometry
  Eigen3::Eigen
  ${Boost_LIBRARIES}
  ${PQXX_LIBRARIES}
  Threads::Threads
  fmt
)

# target_link_libraries(example_rdp PRIVATE
#   company_geometry
#   Eigen3::Eigen
#   ${Boost_LIBRARIES}
#  )

add_test(NAME test_geometry COMMAND test_geometry)
INSTALL(TARGETS LIBRARY DESTINATION ${LIB_INSTALL_DIR})
INSTALL(FILES geometry.hpp DESTINATION ${INCLUDE_INSTALL_DIR}/company)
