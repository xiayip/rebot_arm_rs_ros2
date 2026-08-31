#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
description_dir="$(cd "${script_dir}/.." && pwd)"
isaac_sim_root="${ISAAC_SIM_ROOT:-/opt/isaacsim}"
output_dir="${1:-${description_dir}/usd}"
importer="${isaac_sim_root}/standalone_examples/api/isaacsim.asset.importer.urdf/urdf_import.py"
xacro_file="${description_dir}/urdf/rebot_arm_rs.urdf.xacro"
temporary_dir="$(mktemp -d /tmp/rebot_arm_rs_isaac_XXXXXX)"
urdf_file="${temporary_dir}/rebot_arm_rs.urdf"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/rebot_arm_rs_isaac_matplotlib}"
staging_dir=""

cleanup() {
  rm -rf "${temporary_dir}"
  if [[ -n "${staging_dir}" && -d "${staging_dir}" ]]; then
    rm -rf "${staging_dir}"
  fi
}
trap cleanup EXIT

if ! command -v xacro >/dev/null 2>&1; then
  echo "xacro was not found; source the ROS 2 and workspace setup files first." >&2
  exit 1
fi
if [[ ! -x "${isaac_sim_root}/python.sh" ]]; then
  echo "Isaac Sim Python launcher not found: ${isaac_sim_root}/python.sh" >&2
  exit 1
fi
if [[ ! -f "${importer}" ]]; then
  echo "Isaac Sim URDF importer not found: ${importer}" >&2
  exit 1
fi

mkdir -p "${output_dir}"
staging_dir="$(mktemp -d "${output_dir}/.rebot_arm_rs_isaac_XXXXXX")"
xacro "${xacro_file}" use_mock_hardware:=true > "${urdf_file}"

"${isaac_sim_root}/python.sh" "${importer}" \
  --urdf "${urdf_file}" \
  --usd-path "${staging_dir}" \
  --ros-package "rebot_arm_rs_description:${description_dir}" \
  --fix-base \
  --joint-drive-type force \
  --joint-target-type position \
  --override-joint-stiffness 200 \
  --override-joint-damping 20

generated_robot_dir="${staging_dir}/rebot_arm_rs"
target_robot_dir="${output_dir}/rebot_arm_rs"
previous_robot_dir="${staging_dir}/previous_rebot_arm_rs"

if [[ ! -f "${generated_robot_dir}/rebot_arm_rs.usda" ]]; then
  echo "Isaac Sim did not generate the expected robot asset." >&2
  exit 1
fi

if [[ -d "${target_robot_dir}" ]]; then
  mv "${target_robot_dir}" "${previous_robot_dir}"
fi
if ! mv "${generated_robot_dir}" "${target_robot_dir}"; then
  if [[ -d "${previous_robot_dir}" ]]; then
    mv "${previous_robot_dir}" "${target_robot_dir}"
  fi
  exit 1
fi

echo "Generated ${target_robot_dir}/rebot_arm_rs.usda"
