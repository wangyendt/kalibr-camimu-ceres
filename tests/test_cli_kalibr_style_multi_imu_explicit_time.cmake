if(NOT DEFINED CALIBRATE_CAM_IMU OR NOT DEFINED FIXTURE_GENERATOR OR
   NOT DEFINED FIXTURE_ROOT OR
   NOT DEFINED RESULT_PATH)
  message(FATAL_ERROR "CLI test variables are incomplete")
endif()

execute_process(
  COMMAND "${FIXTURE_GENERATOR}" --generate-fixture "${FIXTURE_ROOT}" 1 4
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
    --imu "${FIXTURE_ROOT}/imu2.yaml"
    --imu "${FIXTURE_ROOT}/imu3.yaml"
    --imu-data "${FIXTURE_ROOT}/imu0.csv"
    --imu-data "${FIXTURE_ROOT}/imu1.csv"
    --imu-data "${FIXTURE_ROOT}/imu2.csv"
    --imu-data "${FIXTURE_ROOT}/imu3.csv"
    --corners "${FIXTURE_ROOT}/cam0_corners.csv"
    --corner-poses "${FIXTURE_ROOT}/cam0_corner_poses.csv"
    --pose-fit-motion-lambda 0.0001
    --pose-fit-boundary-anchors
    --initial-time-shift-s 0.001
    --imu-trim-edge-count 0
    --max-frames 20
    --max-imu-residuals 100
    --max-iterations 0
    --output-result "${RESULT_PATH}"
  RESULT_VARIABLE command_result
  OUTPUT_VARIABLE command_stdout
  ERROR_VARIABLE command_stderr)

if(NOT command_result EQUAL 0)
  message(FATAL_ERROR
    "calibrate_cam_imu failed (${command_result})\n${command_stdout}\n${command_stderr}")
endif()
if(NOT command_stdout MATCHES
   "kalibr-style multi-IMU initialization: imus=4")
  message(FATAL_ERROR
    "an explicit time shift disabled the rest of the Kalibr-style initializer\n${command_stdout}")
endif()
if(NOT EXISTS "${RESULT_PATH}")
  message(FATAL_ERROR "CLI did not write ${RESULT_PATH}")
endif()

file(READ "${RESULT_PATH}" result_yaml)
if(NOT result_yaml MATCHES "time_shift_s: 0\\.001")
  message(FATAL_ERROR
    "explicit camera time shift did not retain highest priority\n${result_yaml}")
endif()
