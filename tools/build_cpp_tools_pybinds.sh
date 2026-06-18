#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cpp_tools_root="${CPP_TOOLS_ROOT:-${repo_root}/third_party/cpp_tools}"
python_exe="${PYTHON:-python3}"
jobs="${JOBS:-}"
kalibr_apriltag_compat="${KALIBR_APRILTAG_COMPAT:-0}"
apriltag_gradient_step="${APRILTAG_GRADIENT_STEP:-}"

if [[ -z "${apriltag_gradient_step}" && "${kalibr_apriltag_compat}" == "1" ]]; then
  apriltag_gradient_step="3"
fi
if [[ -n "${apriltag_gradient_step}" && ! "${apriltag_gradient_step}" =~ ^[1-9][0-9]*$ ]]; then
  echo "APRILTAG_GRADIENT_STEP must be a positive integer, got: ${apriltag_gradient_step}" >&2
  exit 2
fi

if command -v "${python_exe}" >/dev/null 2>&1; then
  python_exe="$(command -v "${python_exe}")"
fi

if [[ -z "${jobs}" ]]; then
  if command -v sysctl >/dev/null 2>&1; then
    jobs="$(sysctl -n hw.ncpu 2>/dev/null || true)"
  fi
  if [[ -z "${jobs}" ]] && command -v nproc >/dev/null 2>&1; then
    jobs="$(nproc 2>/dev/null || true)"
  fi
  if [[ -z "${jobs}" ]] && command -v getconf >/dev/null 2>&1; then
    jobs="$(getconf _NPROCESSORS_ONLN 2>/dev/null || true)"
  fi
  if [[ -z "${jobs}" ]]; then
    jobs="4"
  fi
fi

if [[ ! -d "${cpp_tools_root}/cv/apriltag_detection" ]]; then
  "${repo_root}/tools/update_cpp_tools_sparse_submodule.sh"
fi

pybind11_dir="$("${python_exe}" -c 'import pybind11; print(pybind11.get_cmake_dir())')"
cmake_common=(
  -DCMAKE_BUILD_TYPE=Release
  "-DPYTHON_EXECUTABLE=${python_exe}"
  "-DPython_EXECUTABLE=${python_exe}"
  "-Dpybind11_DIR=${pybind11_dir}"
)

if [[ -n "${OpenCV_DIR:-}" ]]; then
  cmake_common+=("-DOpenCV_DIR=${OpenCV_DIR}")
elif [[ "$(uname -s)" == "Darwin" ]] && command -v brew >/dev/null 2>&1; then
  opencv_prefix="$(brew --prefix opencv 2>/dev/null || true)"
  if [[ -n "${opencv_prefix}" && -d "${opencv_prefix}/lib/cmake/opencv4" ]]; then
    cmake_common+=("-DOpenCV_DIR=${opencv_prefix}/lib/cmake/opencv4")
  elif [[ -d "/usr/local/lib/cmake/opencv4" ]]; then
    cmake_common+=("-DOpenCV_DIR=/usr/local/lib/cmake/opencv4")
  fi
elif [[ -d "/usr/local/lib/cmake/opencv4" ]]; then
  cmake_common+=("-DOpenCV_DIR=/usr/local/lib/cmake/opencv4")
fi

prepare_apriltag_source() {
  local source_dir="${cpp_tools_root}/cv/apriltag_detection"
  if [[ -z "${apriltag_gradient_step}" || "${apriltag_gradient_step}" == "1" ]]; then
    printf '%s\n' "${source_dir}"
    return
  fi

  local compat_dir="${repo_root}/build/cpp_tools_sources/apriltag_detection_step_${apriltag_gradient_step}"
  rm -rf "${compat_dir}"
  mkdir -p "${compat_dir}"
  cp "${source_dir}/CMakeLists.txt" "${compat_dir}/"
  cp "${source_dir}/apriltag_detection_pybind.cpp" "${compat_dir}/"
  cp -R "${source_dir}/src" "${compat_dir}/src"

  APRILTAG_GRADIENT_STEP="${apriltag_gradient_step}" perl -0pi -e 'my $step = $ENV{"APRILTAG_GRADIENT_STEP"}; s/for \(int y = 1; y < fimSeg\.getHeight\(\)-1; y\+\+\) \{\n    for \(int x = 1; x < fimSeg\.getWidth\(\)-1; x\+\+\) \{\n      float Ix = fimSeg\.get\(x\+1, y\) - fimSeg\.get\(x-1, y\);\n      float Iy = fimSeg\.get\(x, y\+1\) - fimSeg\.get\(x, y-1\);/int step = $step;\n  for (int y = step; y < fimSeg.getHeight()-step; y++) {\n    for (int x = step; x < fimSeg.getWidth()-step; x++) {\n      float Ix = fimSeg.get(x+step, y) - fimSeg.get(x-step, y);\n      float Iy = fimSeg.get(x, y+step) - fimSeg.get(x, y-step);/' \
    "${compat_dir}/src/ethz_apriltag2/src/TagDetector.cc"
  if ! grep -q "int step = ${apriltag_gradient_step};" "${compat_dir}/src/ethz_apriltag2/src/TagDetector.cc"; then
    echo "failed to patch apriltag gradient step in copied cpp_tools source" >&2
    exit 2
  fi
  printf '%s\n' "${compat_dir}"
}

build_module() {
  local name="$1"
  local source_dir="${2:-${cpp_tools_root}/cv/${name}}"
  local build_dir="${repo_root}/build/cpp_tools/${name}"
  cmake --fresh -S "${source_dir}" -B "${build_dir}" "${cmake_common[@]}"
  cmake --build "${build_dir}" -j "${jobs}"
}

apriltag_source_dir="$(prepare_apriltag_source)"
build_module apriltag_detection "${apriltag_source_dir}"
if [[ "${apriltag_source_dir}" != "${cpp_tools_root}/cv/apriltag_detection" ]]; then
  mkdir -p "${cpp_tools_root}/cv/apriltag_detection/lib"
  cp -f "${apriltag_source_dir}"/lib/apriltag_detection*.so "${cpp_tools_root}/cv/apriltag_detection/lib/" 2>/dev/null || true
  cp -f "${apriltag_source_dir}"/lib/apriltag_detection*.dylib "${cpp_tools_root}/cv/apriltag_detection/lib/" 2>/dev/null || true
fi
build_module camera_models

if [[ "$(uname -s)" == "Darwin" ]]; then
  opencv_prefixes=()
  if [[ -n "${OPENCV_PREFIX:-}" ]]; then
    opencv_prefixes+=("${OPENCV_PREFIX}")
  fi
  if command -v brew >/dev/null 2>&1; then
    brew_opencv_prefix="$(brew --prefix opencv 2>/dev/null || true)"
    if [[ -n "${brew_opencv_prefix}" ]]; then
      opencv_prefixes+=("${brew_opencv_prefix}")
    fi
  fi
  opencv_prefixes+=("/usr/local")
  for module_dir in \
    "${cpp_tools_root}/cv/apriltag_detection/lib" \
    "${cpp_tools_root}/cv/camera_models/lib"; do
    mkdir -p "${module_dir}"
    for opencv_prefix in "${opencv_prefixes[@]}"; do
      [[ -d "${opencv_prefix}/lib" ]] || continue
      cp -f "${opencv_prefix}"/lib/libopencv_*.dylib "${module_dir}/" 2>/dev/null || true
    done
  done
fi

"${python_exe}" - <<PY
import pathlib
import sys

root = pathlib.Path("${cpp_tools_root}")
sys.path.insert(0, str(root / "cv" / "apriltag_detection" / "lib"))
sys.path.insert(0, str(root / "cv" / "camera_models" / "lib"))
import apriltag_detection
import camera_models
print("cpp_tools pybind modules OK:", apriltag_detection.__file__, camera_models.__file__)
PY
