if(NOT DEFINED CALIBRATE_CAM_IMU OR NOT DEFINED FIXTURE_GENERATOR OR
   NOT DEFINED FIXTURE_ROOT OR
   NOT DEFINED RESULT_PATH)
  message(FATAL_ERROR "CLI test variables are incomplete")
endif()

file(REMOVE "${RESULT_PATH}.boundary")
execute_process(
  COMMAND "${FIXTURE_GENERATOR}" --generate-fixture "${FIXTURE_ROOT}" 2 2
  RESULT_VARIABLE fixture_result
  OUTPUT_VARIABLE fixture_stdout
  ERROR_VARIABLE fixture_stderr)
if(NOT fixture_result EQUAL 0)
  message(FATAL_ERROR
    "fixture generation failed (${fixture_result})\n${fixture_stdout}\n${fixture_stderr}")
endif()

file(REMOVE "${RESULT_PATH}")

execute_process(
  COMMAND "${CALIBRATE_CAM_IMU}"
    --corner-defaults
    --cam "${FIXTURE_ROOT}/camchain.yaml"
    --target "${FIXTURE_ROOT}/aprilgrid.yaml"
    --imu "${FIXTURE_ROOT}/imu0.yaml"
    --imu "${FIXTURE_ROOT}/imu1.yaml"
    --imu-data "${FIXTURE_ROOT}/imu0.csv"
    --imu-data "${FIXTURE_ROOT}/imu1.csv"
    --corners "${FIXTURE_ROOT}/cam0_corners.csv"
    --corners "${FIXTURE_ROOT}/cam1_corners.csv"
    --corner-poses "${FIXTURE_ROOT}/cam0_corner_poses.csv"
    --corner-poses "${FIXTURE_ROOT}/cam1_corner_poses.csv"
    --pose-fit-motion-lambda 0.0001
    --pose-fit-boundary-anchors
    --init-from-camchain
    --fix-camera-chain-extrinsics
    --imu-trim-edge-count 0
    --max-frames 20
    --max-imu-residuals 100
    --time-shift-prior-sigma 0.000001
    # Run a real optimizer step: with a tight per-camera prior this verifies
    # that camera 1 is anchored to its own independently estimated center,
    # not merely that the initializer printed the expected value.
    --max-iterations 2
    --output-result "${RESULT_PATH}"
  RESULT_VARIABLE command_result
  OUTPUT_VARIABLE command_stdout
  ERROR_VARIABLE command_stderr)

if(NOT command_result EQUAL 0)
  message(FATAL_ERROR
    "calibrate_cam_imu failed (${command_result})\n${command_stdout}\n${command_stderr}")
endif()
if(NOT command_stdout MATCHES
   "kalibr-style multi-IMU initialization: imus=2")
  message(FATAL_ERROR
    "CLI did not select the Kalibr-style multi-IMU path\n${command_stdout}")
endif()
if(NOT command_stdout MATCHES "initialized from camchain")
  message(FATAL_ERROR
    "CLI did not preserve camchain initialization for the camera chain\n${command_stdout}")
endif()
if(NOT EXISTS "${RESULT_PATH}")
  message(FATAL_ERROR "CLI did not write ${RESULT_PATH}")
endif()

file(READ "${RESULT_PATH}" result_yaml)
if(NOT result_yaml MATCHES "imu_time_offsets_s: \\[0, 0\\.01[0-9]*\\]")
  message(FATAL_ERROR
    "per-IMU zero-offset fixture was not wired in input order\n${result_yaml}")
endif()
if(NOT command_stdout MATCHES "camera1_time_shift_valid=1")
  message(FATAL_ERROR
    "camera1 time shift was not estimated independently\n${command_stdout}")
endif()
if(NOT result_yaml MATCHES "time_shift_s: -0\\.0149[0-9]+")
  message(FATAL_ERROR
    "camera1 time shift was not written to the camera chain\n${result_yaml}")
endif()
if(NOT command_stdout MATCHES
   "time shift prior residual: camera=1 prior_s=-0\\.01[0-9]+")
  message(FATAL_ERROR
    "camera1 did not receive its independent time-shift prior center\n${command_stdout}")
endif()
string(REGEX MATCH ", -0\\.12000[0-9e+.-]*\\]"
  camera1_baseline_position "${result_yaml}")
if(camera1_baseline_position STREQUAL "")
  message(FATAL_ERROR
    "camera1 baseline was not preserved\n${result_yaml}")
endif()
string(REGEX MATCHALL "camera_index:" camera_entries "${result_yaml}")
list(LENGTH camera_entries camera_count)
if(NOT camera_count EQUAL 2)
  message(FATAL_ERROR
    "expected two camera-chain entries, got ${camera_count}\n${result_yaml}")
endif()
execute_process(
  COMMAND "${CALIBRATE_CAM_IMU}"
    --corner-defaults
    --cam "${FIXTURE_ROOT}/camchain.yaml"
    --target "${FIXTURE_ROOT}/aprilgrid.yaml"
    --imu "${FIXTURE_ROOT}/imu0.yaml"
    --imu "${FIXTURE_ROOT}/imu1.yaml"
    --imu-data "${FIXTURE_ROOT}/imu0.csv"
    --imu-data "${FIXTURE_ROOT}/imu1.csv"
    --corners "${FIXTURE_ROOT}/cam0_corners.csv"
    --corners "${FIXTURE_ROOT}/cam1_corners.csv"
    --corner-poses "${FIXTURE_ROOT}/cam0_corner_poses.csv"
    --corner-poses "${FIXTURE_ROOT}/cam1_corner_poses.csv"
    --init-from-camchain
    --initial-time-shift-s 0.01
    --time-shift-max-search-s 0.01
    --time-shift-prior-sigma 0.001
    --imu-trim-edge-count 0
    --max-frames 20
    --max-imu-residuals 100
    --max-iterations 0
    --output-result "${RESULT_PATH}.boundary"
  RESULT_VARIABLE boundary_result
  OUTPUT_VARIABLE boundary_stdout
  ERROR_VARIABLE boundary_stderr)
if(boundary_result EQUAL 0 OR
   NOT boundary_stderr MATCHES "camera 1 .*refusing to anchor")
  message(FATAL_ERROR
    "secondary-camera boundary rejection was not protected\n${boundary_stdout}\n${boundary_stderr}")
endif()
