# thor_trajectory_lib

**Thor-based trajectory processing for robotic motion generation.**
This library provides a `ThorTrajectoryProcessor` that blends spline interpolation with a receding-horizon MPC step powered by **thor_math**. It exposes light configuration structs for horizon and cost weights, integrates kinodynamic constraints, and outputs time-parameterized position/velocity/acceleration along a path.

> C++20, exported as a shared library with a CMake config package. Depends on Eigen3, `thor_math`, `trajectories_processors_lib`, `cnr_logger`, and `cnr_param`.

---

## Features

- **Spline + MPC pipeline**: interpolate along a spline and refine with a QP step (receding horizon).
- **QP setup helpers**: simple structs for horizon (`QpIntervals`) and cost weights (`QpWeigth`).
- **Constraint handling**: applies kinematic & dynamic limits (pos/vel/acc/effort) and updates solver state.
- **Exported CMake target**: `thor_trajectory_lib::thor_trajectory_lib`.

---


## Installation (colcon workspace)

Follow these steps to fetch **thor_trajectory_lib** and its dependencies into a single colcon workspace and build everything.  
**Note:** Please also follow any OS-specific notes in [`cnr_common/README.md`](../cnr_common/README.md) after the build, if required.

```bash
# 1) Create a workspace and clone this repo
mkdir thor_ws
cd thor_ws
mkdir src
cd src
git clone https://github.com/JRL-CARI-CNR-UNIBS/thor_trajectory_lib.git

# 2) Run the workspace setup from inside the repo
cd thor_trajectory_lib
bash setup_thor_ws.sh -j 8 -b Release --install-deps
#   Flags:
#     -j / --jobs         : parallel build workers (default: nproc)
#     -b / --build-type   : Release | Debug | RelWithDebInfo (default: Release)
#     --update            : git fetch/pull if repos already exist
#     --clean             : wipe build/install/log before building
#     --install-deps      : minimal toolchain + colcon + Eigen (apt-based distros)

# 3) Source the workspace
cd ../..
source install/setup.bash
```

This script will:
- Create a workspace directory for the package and the src directory. symlink if needed).
- Clone the required dependencies into the `thor_ws/src` directory:
  - `JRL-CARI-CNR-UNIBS/cnr_common` (branch `main`)
  - `JRL-CARI-CNR-UNIBS/thor_core` (branch `ros-free`)
  - `JRL-CARI-CNR-UNIBS/trajectories_processors_lib` (branch `main`)
- Build everything with `colcon`.
- Source the newly created workspace
### Consuming in another CMake project

```cmake
find_package(thor_trajectory_lib REQUIRED)

add_executable(app main.cpp)
target_link_libraries(app PRIVATE thor_trajectory_lib::thor_trajectory_lib)
```

---

## Quick Start (C++)

```cpp
#include <openmore/trajectories_processors/thor_trajectory_processor.h>

using namespace openmore;

int main() {
  auto intervals = std::make_shared<QpIntervals>();
  intervals->nax = 7;
  intervals->nc  = 20;
  intervals->control_horizon = 0.4;
  intervals->st  = 0.02;

  auto weights = std::make_shared<QpWeigth>();
  weights->lambda_acc     = 1.0;
  weights->lambda_tau     = 0.0;
  weights->lambda_scaling = 0.1;
  weights->lambda_clik    = 0.0;
  weights->lambda_jerk    = 0.01;

  KinodynamicConstraintsPtr constraints = /* ... */;
  cnr_logger::TraceLoggerPtr logger     = /* ... */;
  std::string param_ns = "/thor";

  ThorTrajectoryProcessor proc(constraints, param_ns, logger);
  proc.init(constraints, param_ns, logger, weights, intervals);

  RobotStatePtr q0 = std::make_shared<RobotState>();
  q0->pos_ = {/* ... */};
  q0->vel_ = {/* ... */};
  proc.setInitialState(q0);

  double t = 0.0, target_scaling = 1.0, updated_scaling = 1.0;
  TrjPointPtr p = std::make_shared<TrjPoint>();
  bool ok = proc.interpolate(t, p, target_scaling, updated_scaling);
}
```

---

## API Overview

### Configuration structs

- `QpIntervals` — receding horizon parameters:
  - `int nax;`
  - `int nc;`
  - `double control_horizon;`
  - `double st;`

- `QpWeigth` — objective weights:
  - `lambda_acc`, `lambda_tau`, `lambda_scaling`, `lambda_clik`, `lambda_jerk`

### Core class

`class ThorTrajectoryProcessor : public SplineTrajectoryProcessor`

Key methods:

- Constructors with/without path and spline order.
- `bool init(..., QpWeigthPtr, QpIntervalsPtr)`
- `bool interpolate(double time, TrjPointPtr& p, const double target_scaling, double& updated_scaling) override`
- `void setWeigths(QpWeigthPtr)`
- `void setIntervals(QpIntervalsPtr)`
- `void setConstraints()` / `void setConstraints(const KinodynamicConstraintsPtr&)`
- `void setInitialState(RobotStatePtr)`

---

## Development Notes

- **Optimization flags**: Release builds enable `-Ofast -flto -funroll-loops`; Debug builds `-Og -g`.
- **Torque bounds** are deactivated by default (`thor.activateTorqueBounds(false)`).

---

## Directory Structure

```
include/openmore/trajectories_processors/thor_trajectory_processor.h
src/thor_trajectory_processor.cpp
cmake_config/thor_trajectory_libConfig.cmake.in
CMakeLists.txt
package.xml
```

---

## License

The repository doesn’t declare a license. Please add one (e.g., MIT or BSD).

---

## Acknowledgments

Built on **thor_math**, **trajectories_processors_lib**, **Eigen3**, **cnr_logger**, and **cnr_param**.
