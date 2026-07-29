cmake_minimum_required(VERSION 3.16)

project(kalibr_camimu_ceres LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(CERES_CAM_IMU_BUILD_APPS "Build command line tools" ON)
option(CERES_CAM_IMU_BUILD_TESTS "Build lightweight numeric checks" ON)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

add_library(ceres_cam_imu STATIC
  src/camera/camera_model.cpp
  src/camera/pinhole_radtan.cpp
  src/core/so3.cpp
  src/core/types.cpp
  src/initialization/camera_translation_initializer.cpp
  src/initialization/kalibr_style_multi_imu_initializer.cpp
  src/initialization/multi_imu_initializer.cpp
  src/initialization/orientation_gravity_initializer.cpp
  src/initialization/pose_spline_fit.cpp
  src/initialization/time_shift_initializer.cpp
  src/io/config_reader.cpp
  src/io/calibration_result_reader.cpp
  src/io/calibration_result_writer.cpp
  src/io/corner_csv_reader.cpp
  src/io/imu_csv_reader.cpp
  src/io/kalibr_result_parser.cpp
  src/io/pose_csv_reader.cpp
  src/optimizer/calibration_problem.cpp
  src/optimizer/parameter_delta_tracker.cpp
  src/optimizer/residual_statistics.cpp
  src/optimizer/state_snapshot.cpp
  src/optimizer/staged_optimizer.cpp
  src/processing/dataset_processing.cpp
  src/residuals/accelerometer_residual.cpp
  src/residuals/bias_motion_prior.cpp
  src/residuals/camera_reprojection_residual.cpp
  src/residuals/gyroscope_residual.cpp
  src/residuals/imu_model.cpp
  src/residuals/pose_motion_prior.cpp
  src/residuals/spline_motion_prior.cpp
  src/residuals/time_shift_prior.cpp
  src/target/aprilgrid.cpp
  src/trajectory/uniform_bspline.cpp
  src/variables/imu_intrinsics.cpp
)

target_include_directories(ceres_cam_imu
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CERES_INCLUDE_DIRS}
)

target_link_libraries(ceres_cam_imu
  PUBLIC
    Eigen3::Eigen
    ${CERES_LIBRARIES}
)

target_compile_options(ceres_cam_imu PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)

if(CERES_CAM_IMU_BUILD_APPS)
  add_executable(check_dataset apps/check_dataset.cpp)
  target_link_libraries(check_dataset PRIVATE ceres_cam_imu)

  add_executable(compare_kalibr_result apps/compare_kalibr_result.cpp)
  target_link_libraries(compare_kalibr_result PRIVATE ceres_cam_imu)

  add_executable(calibrate_cam_imu apps/calibrate_cam_imu.cpp)
  target_link_libraries(calibrate_cam_imu PRIVATE ceres_cam_imu)
endif()

if(CERES_CAM_IMU_BUILD_TESTS)
  enable_testing()
  add_executable(test_math tests/test_math.cpp)
  target_link_libraries(test_math PRIVATE ceres_cam_imu)
  add_test(NAME test_math COMMAND test_math)

  add_executable(test_kalibr_style_multi_imu_initializer
    tests/test_kalibr_style_multi_imu_initializer.cpp)
  target_link_libraries(test_kalibr_style_multi_imu_initializer
    PRIVATE ceres_cam_imu)
  add_test(
    NAME test_kalibr_style_multi_imu_initializer
    COMMAND test_kalibr_style_multi_imu_initializer)

  if(TARGET calibrate_cam_imu)
    add_test(
      NAME test_cli_kalibr_style_multi_imu
      COMMAND ${CMAKE_COMMAND}
        -DCALIBRATE_CAM_IMU=$<TARGET_FILE:calibrate_cam_imu>
        -DFIXTURE_GENERATOR=$<TARGET_FILE:test_kalibr_style_multi_imu_initializer>
        -DFIXTURE_ROOT=${CMAKE_CURRENT_BINARY_DIR}/test_fixture_one_cam_four_imus
        -DRESULT_PATH=${CMAKE_CURRENT_BINARY_DIR}/test_cli_kalibr_style_multi_imu_result.yaml
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_cli_kalibr_style_multi_imu.cmake)
    add_test(
      NAME test_cli_kalibr_style_multi_camera_multi_imu
      COMMAND ${CMAKE_COMMAND}
        -DCALIBRATE_CAM_IMU=$<TARGET_FILE:calibrate_cam_imu>
        -DFIXTURE_GENERATOR=$<TARGET_FILE:test_kalibr_style_multi_imu_initializer>
        -DFIXTURE_ROOT=${CMAKE_CURRENT_BINARY_DIR}/test_fixture_two_cams_two_imus
        -DRESULT_PATH=${CMAKE_CURRENT_BINARY_DIR}/test_cli_kalibr_style_multi_camera_multi_imu_result.yaml
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_cli_kalibr_style_multi_camera_multi_imu.cmake)
    add_test(
      NAME test_cli_kalibr_style_multi_imu_explicit_time
      COMMAND ${CMAKE_COMMAND}
        -DCALIBRATE_CAM_IMU=$<TARGET_FILE:calibrate_cam_imu>
        -DFIXTURE_GENERATOR=$<TARGET_FILE:test_kalibr_style_multi_imu_initializer>
        -DFIXTURE_ROOT=${CMAKE_CURRENT_BINARY_DIR}/test_fixture_one_cam_four_imus_explicit
        -DRESULT_PATH=${CMAKE_CURRENT_BINARY_DIR}/test_cli_kalibr_style_multi_imu_explicit_time_result.yaml
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_cli_kalibr_style_multi_imu_explicit_time.cmake)

    find_package(Python3 QUIET COMPONENTS Interpreter)
    if(Python3_Interpreter_FOUND)
      add_test(
        NAME test_benchmark_kalibr_style_single_path
        COMMAND ${CMAKE_COMMAND}
          -DPYTHON_EXECUTABLE=${Python3_EXECUTABLE}
          -DRUNNER=${CMAKE_CURRENT_SOURCE_DIR}/tools/run_docker_benchmark.py
          -DWORK_ROOT=${CMAKE_CURRENT_BINARY_DIR}/test_benchmark_kalibr_style_single_path
          -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_benchmark_kalibr_style_single_path.cmake)
    endif()
  endif()
endif()
