# romea_ros2_description #


This packages provides:
* a generic way to define and validate **device configurations** in ROS 2 using Python
* reusable **Xacro building blocks** to compose robot and sensor descriptions

It helps standardize both configuration and description across projects.
* build **generic and reusable configurations**
* reduce user input complexity
* compose **modular robot descriptions**

---

## Device configuration concept

A **device configuration** is built by combining:

* a **device specification** (what is allowed)
* a **user configuration** (what is chosen)

From these two inputs, the system automatically generates a **complete and validated configuration**.

The specification defines how each parameter behaves. It can include:

* **scalar values**: fixed or default values
* **ranges**: numeric intervals
* **lists**: predefined allowed values
* **dependent parameters**: values computed from other parameters
* **structured dictionaries**: grouped parameters with fixed keys

The user configuration only needs to provide a subset of parameters.
Missing values are automatically completed using defaults or inferred rules.

---

## Example

### Specification

```yaml id="x1p9ra"
foo_device_specifications:
  enabled:
    default: true

  mode:
    default: "basic"
    list: ["basic", "advanced", "expert"]

  level:
    default: 5
    range: [0, 10]

  id:
    value: 42

  gains:
    defaults:
      kp: 1.0
      ki: 0.1
      kd: 0.01

  output:
    depend: mode
    dict:
      basic: 10
      advanced: 20
      expert: 50
```

### User configuration

```yaml id="p7z3ld"
foo_device:
  mode: "advanced"
  level: 7

  gains:
    kp: 2.0
    ki: 0.2
    kd: 0.02
```

### Resulting configuration

```yaml id="m3k9ts"
enabled: true
mode: "advanced"
level: 7
id: 42
gains:
  kp: 2.0
  ki: 0.2
  kd: 0.02
output: 20
```

### Python usage

```python id="v8q2cn"
from romea_common_description import DeviceConfiguration

config = DeviceConfiguration("foo_device", specs, user_config)

config.get("mode")     # "advanced"
config.get("level")    # 7
config.get("output")   # 20 (depends on mode)
config.get("id")       # 42 (non-modifiable)
```

---

## Xacro utilities

This package also provides reusable **Xacro building blocks** that simplify robot and sensor descriptions.

They provide:

* **argument handling helpers**: load and structure Xacro arguments in a consistent way
* **derived properties**: compute reusable properties from arguments
* **geometric primitives**: simple shapes (boxes, cylinders, etc.)
* **inertial elements**: ready-to-use inertia definitions

These elements help:

* reduce duplication in Xacro files
* standardize robot and sensor descriptions
* build modular and maintainable URDF structures
