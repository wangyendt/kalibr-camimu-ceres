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
    --imu-trim-edge-count 0
    --max-frames 20
    --max-imu-residuals 100
    --max-iterations 0
    --estimate-multi-imu-translation-prior
    --imu-chain-prior-refine-rotation-after-translation-prior
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
    "CLI did not select the Kalibr-style multi-IMU path\n${command_stdout}")
endif()
if(NOT EXISTS "${RESULT_PATH}")
  message(FATAL_ERROR "CLI did not write ${RESULT_PATH}")
endif()

file(READ "${RESULT_PATH}" result_yaml)
if(NOT result_yaml MATCHES "imu_time_offsets_s: \\[0, 0\\.01[0-9]*, -0\\.01[0-9]*, 0\\.02[0-9]*\\]")
  message(FATAL_ERROR
    "per-IMU zero-offset fixture was not wired in input order\n${result_yaml}")
endif()
if(NOT "${command_stdout}${command_stderr}" MATCHES
   "syntax/range checked but does not use these legacy initializer flags")
  message(FATAL_ERROR
    "legacy initializer flags were silently ignored\n${command_stdout}")
endif()
string(REGEX MATCHALL "r_b: \\[0, 0, 0\\]" zero_levers "${result_yaml}")
list(LENGTH zero_levers zero_lever_count)
if(NOT zero_lever_count EQUAL 5)
  message(FATAL_ERROR
    "expected one reference plus four zero translational seeds, got ${zero_lever_count}\n${result_yaml}")
endif()
