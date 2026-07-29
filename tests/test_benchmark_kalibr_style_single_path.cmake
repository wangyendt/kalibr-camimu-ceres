if(NOT DEFINED PYTHON_EXECUTABLE OR NOT DEFINED RUNNER OR
   NOT DEFINED WORK_ROOT)
  message(FATAL_ERROR "benchmark CLI test variables are incomplete")
endif()

set(dataset_root "${WORK_ROOT}/synthetic_session/cam_imu")
file(MAKE_DIRECTORY "${dataset_root}")
foreach(name IN ITEMS
    cam0_640x400_corners.pkl
    0_save_timestamp.txt
    aprilgrid.yaml
    cam0-camchain-640x400.yaml
    imu.yaml
    cam0_640x400_corners.csv
    cam0_640x400_corner_poses.csv
    data1.csv data2.csv data3.csv data4.csv)
  file(WRITE "${dataset_root}/${name}" "")
endforeach()

execute_process(
  COMMAND "${PYTHON_EXECUTABLE}" "${RUNNER}"
    --suite benchmark-multi-imu
    --multi-session synthetic_session
    --benchmark-root "${WORK_ROOT}"
    --out-root "${WORK_ROOT}/out"
    --kalibr-platform linux/arm64
    --print-only
  RESULT_VARIABLE command_result
  OUTPUT_VARIABLE command_stdout
  ERROR_VARIABLE command_stderr)

if(NOT command_result EQUAL 0)
  message(FATAL_ERROR
    "benchmark dry-run failed (${command_result})\n${command_stdout}\n${command_stderr}")
endif()
string(REGEX MATCHALL "Ceres default" default_runs "${command_stdout}")
list(LENGTH default_runs default_run_count)
if(NOT default_run_count EQUAL 5)
  message(FATAL_ERROR
    "expected one production Ceres run for joint plus each of four single-IMU cases, got ${default_run_count}\n${command_stdout}")
endif()
if(command_stdout MATCHES "Ceres (single|short|long|chain_)" OR
   command_stdout MATCHES "--staged" OR
   command_stdout MATCHES "--init-from-(kalibr|result)" OR
   command_stdout MATCHES "--estimate-camera-translation-prior")
  message(FATAL_ERROR
    "production Kalibr-style benchmark unexpectedly selected a legacy/candidate path\n${command_stdout}")
endif()
if(NOT command_stdout MATCHES "--estimate-time-shift-prior" OR
   NOT command_stdout MATCHES "--estimate-orientation-gravity-prior" OR
   NOT command_stdout MATCHES "--pose-motion-prior")
  message(FATAL_ERROR
    "production Kalibr-style benchmark is missing required initialization/solve options\n${command_stdout}")
endif()
