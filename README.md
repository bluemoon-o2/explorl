# ExplorL

A lightweight, high-performance physics engine designed for reinforcement learning and teaching. ExplorL is built from scratch with a focus on modern C++ architecture, ease of understanding, and seamless Python integration.

## Core Philosophy

-   **Modern Architecture**: Built on C++20 and Modern OpenGL (Core Profile).
-   **High Performance**: Default SIMD support, Zero-Copy data transfer, and multi-threaded batch simulation.
-   **Teachable**: "Formula-as-Code" implementation where physics equations are clearly visible and commented.
-   **RL-Ready**: Native Gymnasium API support with fast reset mechanisms.

## Features

### Physics Core
-   **Numerical Stability**: Uses `Eigen 3.4+` for robust math operations.
-   **Dynamics**: Explicit separation of state and derivative, supporting Semi-Implicit Euler and RK4 integrators.
-   **Collision Detection**: Educational yet efficient implementation of GJK (Gilbert-Johnson-Keerthi) and EPA (Expanding Polytope Algorithm).
-   **Joints**: Support for Fixed, Hinge, and Slider joints.
-   **Asset Loading**: Experimental URDF loader with support for basic kinematic chains and mesh loading.

### Rendering
-   **Modern OpenGL**: Programmable pipeline using Shaders (GLSL).
-   **Visual Debugging**: Built-in support for visualizing velocity vectors, forces, contact points, and AABB.
-   **Headless Mode**: Support for software rendering for server-side training.

### Python Interface
-   **Lightweight Bindings**: Powered by `nanobind` for minimal overhead.
-   **NumPy Integration**: Direct access to physics state via NumPy views (Zero-Copy).
-   **Gymnasium Compatible**: Drop-in replacement for standard RL environments.

## Installation

### Prerequisites
-   **C++ Compiler**: MSVC 2019+ (Windows) or GCC/Clang with C++20 support.
-   **CMake**: Version 3.18 or higher.
-   **Python**: Version 3.8 or higher.

### Building from Source

1.  **Clone the repository**:
    ```bash
    git clone https://github.com/bluemoon-o2/explorl.git
    cd explorl
    ```

2.  **Build using the provided script**:
    ```bash
    python scripts.py
    ```

    *Alternatively, build manually via CMake:*
    ```bash
    mkdir build
    cd build
    cmake ..
    cmake --build . --config Release
    ```

3.  **Environment Setup**:
    The build process places the compiled Python extension (`.pyd` or `.so`) into `explorl/lib`. Ensure you run your scripts from the project root or add the project root to your `PYTHONPATH`.

## Usage

### Basic Python Example

```python
import explorl
from explorl.envs import CartPoleEnv

# Create environment
env = CartPoleEnv(render_mode="human")
obs, _ = env.reset()

print("Observation space:", env.observation_space_shape)

for _ in range(1000):
    # Random action: 0 (Left) or 1 (Right)
    action = 0 
    obs, reward, terminated, truncated, info = env.step(action)
    
    env.render()
    
    if terminated:
        env.reset()
```

### Running Tests
After building, C++ tests are available in the `build` directory (e.g., `build/src/tests/Release/` on Windows).
-   `test_physics_hello`: Basic physics simulation test.
-   `test_cartpole`: C++ implementation of CartPole.
-   `test_render`: Rendering engine demo.

## Directory Structure

-   `explorl/`: Python package and gymnasium environments.
-   `src/`: C++ source code.
    -   `dynamics/`: Rigid body dynamics, solvers, and joints.
    -   `collision/`: Collision detection algorithms (GJK, EPA).
    -   `render/`: OpenGL rendering engine.
    -   `bindings/`: Nanobind Python bindings.
    -   `math/`: Math utilities and type definitions.
-   `tests/`: Python tests and benchmarks.
