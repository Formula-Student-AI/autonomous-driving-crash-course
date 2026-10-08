# vehicle_models

A C++20 library (`vehicle_models`) of **vehicle motion / dynamics models** used by the
Bristol FSAI stack — most importantly by the simulator core
([`eufs_sim2`](../eufs_sim2)) to propagate the car's state forward in time.

Every model takes the **current state**, the **latest command** (acceleration +
steering angle), and a **time step `dt`**, and returns the **next state** — optionally
propagating the state **covariance** as well (EKF-style), which is what makes these
models reusable by both the simulator *and* state-estimation code.

> **Migration note.** This package replaces the old EUFS `eufs_models` package
> (from the previous `core-sim`, upstream lineage: <https://gitlab.com/eufs/public>).
> It was **fully rewritten** — different namespace (`eufs::vehicle_models`, not
> `eufs::models`), different layout (`include/vehicle_models/…`), a templated
> composition-based design, and `state_lib`-backed state vectors with covariance.
> If you find docs or code referring to `eufs::models`, `include/models/…`,
> `getWheelSpeeds`/`getSlipAngle`, `Noise`, or `eufs_plugins/gazebo_race_car_model`,
> those describe the **old** library and no longer apply.

---

## Package facts

| | |
| --- | --- |
| Library target | `vehicle_models` (also the ament export name) |
| Namespace | `eufs::vehicle_models` (state helpers in `eufs::vehicle_models::state`) |
| Language | C++20 (uses concepts and templates heavily) |
| Build | `ament_cmake`, mostly header-only; compiled units are [`src/ros.cpp`](src/ros.cpp) and [`src/steering_model.cpp`](src/steering_model.cpp) |
| Dependencies | `yaml-cpp`, `rclcpp`, `Eigen3`, [`state_lib`](../state_lib), `angles` |
| Test dep | [`eufs_gmock_matchers`](../eufs_gmock_matchers) (tests in [`test/`](test)) |
| License | MIT |

---

## Design in one picture

A vehicle model is **composed** from small, swappable physical sub-models rather than
written as one monolithic equation:

```text
DynamicBicycle  (full model: state + covariance propagation)
└─ BaseDynamicBicycle   (state-only propagation)
   ├─ PowertrainModel        drive/brake force
   ├─ DragModel              aerodynamic drag      (F = C_drag · v_x²)
   ├─ DownforceModel         aerodynamic downforce (F = C_down · v_x²)
   ├─ SteeringModel          rate-limited steering actuator
   └─ TyreModel × {front, rear}
        ├─ SlipAngleModel    (front / rear)
        └─ DownforceModel    (front / rear)
```

Each sub-model has a matching **C++20 concept** under
[`include/vehicle_models/constraint/`](include/vehicle_models/constraint). The concept is
the "socket": as long as your replacement satisfies the concept, you can plug a different
tyre/drag/powertrain implementation into `DynamicBicycle` via its template parameters
without touching the model itself.

---

## Core types

Defined under [`include/vehicle_models/types/`](include/vehicle_models/types):

| Type | File | What it holds |
| --- | --- | --- |
| `Param` | [`types/param.hpp`](include/vehicle_models/types/param.hpp) | All vehicle parameters, grouped: `inertia` (m, g, I_z), `kinematic` (l, w_front, l_F, l_R, axle_width), `tyre` (A, B, C, radius, rolling_resistance), `powertrain` (overdrive), `steering` (max_rate, max_angle), `aero` (c_down, c_drag), `input_ranges` (acc / vel / delta min-max). |
| `Command` | [`types/command.hpp`](include/vehicle_models/types/command.hpp) | The control input: `acceleration`, `steering_angle`. |
| `State` | [`types/state.hpp`](include/vehicle_models/types/state.hpp) | A simple kinematic state (`position` / `velocity` / `acceleration`, each an x,y,z,roll,pitch,yaw `KinematicVariable`). |

Helpers `ValidateCommand(cmd, input_ranges)` and `ValidateState<…>(state)` clamp inputs and
states into physically valid ranges (e.g. forward velocity is never negative).

### The model state vs. `state_lib`

The models are **templated on a state type** (`template <typename State> …`). In practice that
`State` is a [`state_lib`](../state_lib) state vector, which is what carries the **covariance**
and exposes `::Vector`, `::Matrix`, `::EigenMatrix`. The index layout (which row is `v_x`,
`v_yaw`, etc.) is defined in:

* [`state/vars.hpp`](include/vehicle_models/state/vars.hpp) — the variable indices
  (`_x, _y, _yaw, _v_x, _v_y, _v_yaw, _a_x, _a_y`)
* [`state/base_2d.hpp`](include/vehicle_models/state/base_2d.hpp) — builds the
  `Base2DState` / `Base2DVector` from those indices via `state_lib`.

This is why `vehicle_models` depends on `state_lib`: `state_lib` owns the state-vector /
covariance machinery, and `vehicle_models` provides the physics that propagates it.

---

## Models

Umbrella header: [`include/vehicle_models/models.hpp`](include/vehicle_models/models.hpp).
The base class is `VehicleModel<State, ProcessNoiseGenerator = ConstantNoise>`
([`models/vehicle_model.hpp`](include/vehicle_models/models/vehicle_model.hpp)) — every model
implements:

```cpp
virtual State Update(State &state, Command command, const rclcpp::Duration &dt) = 0;
Param &GetParam();
```

Most models come in two layers:

* a **`Base…`** variant that propagates the **state vector only**, and
* a full variant that *also* propagates the **covariance** (builds the Jacobian and applies
  the process noise).

| Model | File | Notes |
| --- | --- | --- |
| **Dynamic bicycle** | [`models/dynamic_bicycle/`](include/vehicle_models/models/dynamic_bicycle) | The main vehicle dynamics model. Tyre/downforce/drag/powertrain/steering composed in; this is what `eufs_sim2` runs. |
| Kinematic bicycle | [`models/kinematic_bicycle/`](include/vehicle_models/models/kinematic_bicycle) | Geometric (no tyre forces) bicycle model. |
| Uniform motion 2D | [`models/uniform_2d/`](include/vehicle_models/models/uniform_2d) | Constant-twist kinematic propagation + covariance; reused as a building block by the dynamic model. |
| Acceleration | [`models/acceleration/`](include/vehicle_models/models/acceleration) | Constant-acceleration propagation. |
| Angular velocity 2D | [`models/angular_vel_2d/`](include/vehicle_models/models/angular_vel_2d) | Yaw-rate propagation. |
| Angular acceleration 2D | [`models/angular_accel_2d/`](include/vehicle_models/models/angular_accel_2d) | Yaw-acceleration propagation. |

### Process noise

The covariance step uses a pluggable noise generator (template parameter
`ProcessNoiseGenerator`):

* `ConstantNoise` ([`process_noise/constant_noise.hpp`](include/vehicle_models/process_noise/constant_noise.hpp))
  — default; scales a fixed noise matrix by `dt`.
* `DynamicNoiseGenerator` ([`process_noise/dynamic_noise.hpp`](include/vehicle_models/process_noise/dynamic_noise.hpp))
  — scales the twist block of the noise by current speed (robot_localization-style dynamic noise).

> **`models.txt`.** [`models.txt`](models.txt) is a legacy index of model names, originally
> read by EUFS's `eufs_launcher` (which no longer exists in this repo). **Nothing in this
> repo reads it** — the model is selected at compile time in C++ (`eufs_sim2` instantiates
> `DynamicBicycle` directly), so this file is currently vestigial. It now lists only
> `DynamicBicycle`, the one end-to-end vehicle model actually implemented here. The other
> entries under `models/` (uniform 2D, acceleration, angular motion, kinematic bicycle) are
> internal motion-model building blocks, not standalone vehicle models, so they are
> deliberately left out.

---

## Configuration

Parameters live as YAML under [`config/`](config), one folder per model
(e.g. [`config/DynamicBicycle/`](config/DynamicBicycle)). The YAML keys mirror the `Param`
groups:

```yaml
inertia:    { m: 300.0, g: 9.81, I_z: 150.3 }
kinematic:  { l: 1.53, w_front: 0.493, axle_width: 1.2 }
tyre:       { A: 3.669, B: 2.246, C: 0.1003, radius: 0.2525, rolling_resistance: 0.0175 }
powertrain: { overdrive: 0.0497 }
steering:   { max_rate: ..., max_angle: ... }
aero:       { C_down: 0.974, C_drag: 0.0 }
input_ranges:
  acceleration: { min: -5.2, max: 5.2 }
  velocity:     { min: 0.0,  max: 30.0 }
  steering:     { min: -0.37, max: 0.37 }
```

The provided `ads-dv-*.yaml` files target the **ADS-DV** competition car.

> ⚠️ The committed values are explicitly flagged in-file as approximate / "sketchy
> calculations" — treat them as a starting point for tuning, not validated physical
> constants. `ads-dv-tuned.yaml` is the hand-tuned variant.

Loading params, two ways:

* From a YAML file: `param.SetFromYaml("<path>.yaml")`
* From ROS 2 node parameters: `eufs::vehicle_models::SetParamsFromNode(node)`
  ([`ros.hpp`](include/vehicle_models/ros.hpp) / [`src/ros.cpp`](src/ros.cpp))

---

## Usage example

Exactly how the simulator core uses it (see [`eufs_sim2/src/eufs_core.cpp`](../eufs_sim2/src/eufs_core.cpp)):

```cpp
#include "vehicle_models/models/dynamic_bicycle/dynamic_bicycle.hpp"
#include "vehicle_models/types/command.hpp"

using namespace eufs::vehicle_models;

// 1. Load parameters (from a YAML file, or from ROS node params).
Param param;
param.SetFromYaml("config/DynamicBicycle/ads-dv-tuned.yaml");

// 2. Construct the model with the params and an initial process-noise matrix.
DynamicBicycle<VehicleState, type::VehicleStateMember> model(
    param, VehicleState::EigenMatrix::Identity());

// 3. Each tick: feed in a command and dt, get the next state (state + covariance).
Command command{ .acceleration = 1.0, .steering_angle = 0.05 };
state = model.Update(state, command, dt);   // rclcpp::Duration dt
```

`VehicleState` / `type::VehicleStateMember` here are the `state_lib`-backed state and its
index layout provided by the consumer (see `eufs_sim2`'s `core` types).

---

## CMake setup

To use the library in another package:

```cmake
find_package(vehicle_models REQUIRED)

target_link_libraries(${PROJECT_NAME}
  vehicle_models::vehicle_models
)
```

(`state_lib`, `Eigen3`, `yaml-cpp`, `rclcpp` and `angles` are pulled in transitively as
exported dependencies.)

---

## Tests

Unit tests live in [`test/`](test) and run under `colcon test` (built when `BUILD_TESTING`
is on). They cover the dynamic bicycle, the kinematic building blocks (uniform 2D,
acceleration, angular accel 2D), and use the `eufs_gmock_matchers` helpers.
