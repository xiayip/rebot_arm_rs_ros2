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

cleanup() {
  rm -rf "${temporary_dir}"
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
xacro "${xacro_file}" use_mock_hardware:=true > "${urdf_file}"

"${isaac_sim_root}/python.sh" "${importer}" \
  --urdf "${urdf_file}" \
  --usd-path "${output_dir}" \
  --ros-package "rebot_arm_rs_description:${description_dir}" \
  --fix-base \
  --joint-drive-type force \
  --joint-target-type position \
  --override-joint-stiffness 200 \
  --override-joint-damping 20

echo "Generated ${output_dir}/rebot_arm_rs/rebot_arm_rs.usda"
