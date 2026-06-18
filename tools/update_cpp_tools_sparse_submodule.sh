#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
submodule_path="${repo_root}/third_party/cpp_tools"

if [[ ! -e "${submodule_path}/.git" ]]; then
  git -C "${repo_root}" submodule update --init third_party/cpp_tools
fi

git -C "${submodule_path}" sparse-checkout init --cone
git -C "${submodule_path}" sparse-checkout set cv/apriltag_detection cv/camera_models

if [[ -n "${CPP_TOOLS_REF:-}" ]]; then
  git -C "${submodule_path}" fetch origin "${CPP_TOOLS_REF}"
  git -C "${submodule_path}" checkout FETCH_HEAD
fi

git -C "${submodule_path}" status --short
