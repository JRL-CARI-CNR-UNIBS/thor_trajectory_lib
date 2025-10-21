<p align="center">
  <img src="docs/thor_logo_transparent_background.png?raw=true" alt="Thor Logo" width="40%" style="display: block; margin: auto;">
</p>


**Thor-based trajectory processing for robotic motion generation.**
This library provides a `ThorTrajectoryProcessor` that blends spline interpolation with a receding-horizon MPC step powered by **thor_math**. It exposes light configuration structs for horizon and cost weights, integrates kinodynamic constraints, and outputs time-parameterized position/velocity/acceleration along a path.

> C++20, exported as a shared library with a CMake config package. Depends on Eigen3, [`thor_core`](https://github.com/JRL-CARI-CNR-UNIBS/thor_core) for MPC implementation, ,[`trajectories_processors_lib`](https://github.com/JRL-CARI-CNR-UNIBS/trajectories_processors_lib) for basic interpolation structures, and [`cnr_common`](https://github.com/JRL-CARI-CNR-UNIBS/cnr_common) for utility functions .
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

A complete example of the library usage for interpolation can be found in `example/test_thor.cpp`. The example provides a script for:

* Creation of all structures needed by the class.
* Initialization of the `ThorTrajectoryProcessor` object.
* The interpolation procedure.

### Trajectory Generation

At the current state, the trajectory must be given to the object from outside. To include the trajectory generation method inside the class, create a class that inherits from `ThorTrajectoryProcessor` and overrides the following methods:

* `bool ThorTrajectoryProcessor::computeTrj()`
* `bool ThorTrajectoryProcessor::computeTrj(const RobotStatePtr& initial_state, const RobotStatePtr& final_state)`
  
---

## API Overview

### Configuration structs

- `QpIntervals` — receding horizon parameters:
  - `int nax;`: number of degrees of freedom
  - `int nc;`: number of control and prediction instants (the instnts are supposed to coincide)
  - `double control_horizon;`: the MPC control horizon
  - `double st;`: the MPC sampling time

- `QpWeigth` — MPC cost function weights:
  - `double lambda_acc`: the weight on acceleration error
  - `double lambda_scaling`: the weight on scaling (trajectory speed adjustment)
  - `double lambda_pos`: the weight on position error 
  - `double lambda_jerk`: the weight on jerk minimization
  - The weight on velocity error is supposed to be equal to 1

- `robot_state` — Represents a robot state, including position, velocity, acceleration, and effort:
  - `std::vector<double> pos_`
  - `std::vector<double> vel_`
  - `std::vector<double> acc_`
  - `std::vector<double> eff_`

- `TrjPoint` — Represents a trajectory point, consisting of a robot state and a time from the trajectory start.
  - `robot_state state_`
  - `double time_from_start_`

- `KinodynamicConstraints` — Represents the kinodynamic constraints of the robot.
  - `Eigen::VectorXd min_pos_`
  - `Eigen::VectorXd min_vel_`
  - `Eigen::VectorXd min_acc_`
  - `Eigen::VectorXd min_eff_`
  - `Eigen::VectorXd max_pos_`
  - `Eigen::VectorXd max_vel_`
  - `Eigen::VectorXd max_acc_`
  - `Eigen::VectorXd max_eff_`

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

## Work in progress
This repository is a work in progress and is continuously evolving. As such, it is not free of bugs.
 **Please be careful if you use it on real hardware and ensure all necessary safety measures are in place**.

If you find errors or if you have some suggestions, [please let us know](https://github.com/JRL-CARI-CNR-UNIBS/thor_trajectory_lib/issues).

We are actively seeking support for further development. If you're interested, please reach out via email at <mailto::f.parma@phd.poliba.it>.

## How to cite
Plain text:
```
TODO
```

BibTex:
```
TODO
```

## Developer Contact
### **Authors**
- Federico Parma (<mailto::f.parma@phd.poliba.it>)
- Cesare Tonola (<mailto::c.tonola001@unibs.it>)
- Manuel Beschi (<mailto::manuel.beschi@unibs.it>)

## Acknowledgements
**OpenMORE** is developed with [CNR-STIIMA](http://www.stiima.cnr.it/) and [University of Brescia](https://www.unibs.it/en).