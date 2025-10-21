#!/usr/bin/env bash
#
# setup_thor_ws.sh
#
# Intended location: thor_trajectory_lib/ (repo root)
#
# Quick start (for your README):
#   mkdir thor_ws
#   cd thor_ws
#   git clone https://github.com/JRL-CARI-CNR-UNIBS/thor_trajectory_lib.git
#   cd thor_trajectory_lib
#   bash setup_thor_ws.sh -j 8 -b Release --install-deps
#
# What this does:
#   - Detects the colcon workspace root as the parent directory of this repo (../).
#   - Ensures a src/ folder exists and the current repo is under WS/src/thor_trajectory_lib (symlink if needed).
#   - Clones required dependencies into the SAME WS/src:
#       * cnr_common (branch: main)
#       * thor_core (branch: ros-free)
#       * trajectories_processors_lib (branch: main)
#   - Builds the workspace with colcon.
#   - Reminds you to follow cnr_common/README.md for any extra steps.
#
set -euo pipefail
cd ..
JOBS=${JOBS:-"$(nproc || echo 4)"}
BUILD_TYPE=${BUILD_TYPE:-"Release"}
UPDATE=false
CLEAN=false
INSTALL_DEPS=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    -j|--jobs) JOBS="$2"; shift 2;;
    -b|--build-type) BUILD_TYPE="$2"; shift 2;;
    --update) UPDATE=true; shift;;
    --clean) CLEAN=true; shift;;
    --install-deps) INSTALL_DEPS=true; shift;;
    -h|--help)
      grep -E '^# ' "$0" | sed 's/^# //'
      exit 0;;
    *)
      echo "Unknown argument: $1"; exit 1;;
  esac
done

# Find this repo's root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$SCRIPT_DIR"
if git -C "$SCRIPT_DIR" rev-parse --show-toplevel >/dev/null 2>&1; then
  REPO_ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel)"
fi

# # Define workspace as the parent of thor_trajectory_lib
# WS="$(cd "$REPO_ROOT/.." && pwd)"
# SRC="$WS/src"

# echo "Detected workspace root: $WS"
# mkdir -p "$SRC"

# # Ensure thor_trajectory_lib is visible at WS/src/thor_trajectory_lib
# if [[ "$REPO_ROOT" != "$SRC/thor_trajectory_lib" ]]; then
#   ln -sfn "$REPO_ROOT" "$SRC/thor_trajectory_lib"
# fi

echo "Ensuring dependencies live in the same workspace (WS/src)"
declare -A REPOS
REPOS["thor_core"]="https://github.com/JRL-CARI-CNR-UNIBS/thor_core.git@ros-free"
REPOS["trajectories_processors_lib"]="https://github.com/JRL-CARI-CNR-UNIBS/trajectories_processors_lib.git@master"

# pushd "$SRC" >/dev/null
for name in "${!REPOS[@]}"; do
  entry="${REPOS[$name]}"
  url="${entry%@*}"
  branch="${entry#*@}"

  if [[ -d "$name/.git" ]]; then
    echo "[=] $name already present."
    if $UPDATE; then
      echo "    -> updating $name (branch: $branch)"
      pushd "$name" >/dev/null
      git fetch --all --tags
      git checkout "$branch" || true
      git pull --ff-only || true
      popd >/dev/null
    fi
  else
    echo "[+] Cloning $name (branch: $branch)"
    git clone --branch "$branch" --depth 1 "$url" "$name"
  fi
done

git clone --recurse-submodules https://github.com/JRL-CARI-CNR-UNIBS/cnr_common.git
cd cnr_common
git submodule update --init --recursive
. update_submodules.sh

echo "QUIIIIIII"

# # Optional: minimal deps (toolchain + colcon + Eigen) — always consult cnr_common/README.md for your OS
# if $INSTALL_DEPS; then
#   if command -v apt-get >/dev/null 2>&1; then
#     sudo apt-get update
#     sudo apt-get install -y --no-install-recommends \
#       build-essential cmake git \
#       python3 python3-pip python3-colcon-common-extensions \
#       pkg-config libeigen3-dev
#   else
#     echo "NOTE: --install-deps only supports apt-get. For other platforms, follow cnr_common/README.md."
#   fi
# fi
cd ../..

sudo apt update
sudo add-apt-repository universe
sudo apt install python3-colcon-common-extensions
sudo apt -y install libboost-all-dev libeigen3-dev libyaml-cpp-dev libpoco-dev liblog4cxx-dev libgtest-dev
# Build
if $CLEAN; then
  echo "[*] Cleaning build/ install/ log/"
  rm -rf build install log
fi

if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon not found. Install with: sudo apt install python3-colcon-common-extensions"
  exit 1
fi

echo "[*] Building with colcon (type: $BUILD_TYPE, jobs: $JOBS)"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE="$BUILD_TYPE" --parallel-workers "$JOBS"

echo ""
echo "===================== DONE ====================="
echo "Source the workspace before using the packages:"
echo ""
