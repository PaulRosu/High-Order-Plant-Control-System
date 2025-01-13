# High-Order Plant Control System

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()

A real-time control system implementing polynomial-based path planning with direct jerk and snap control for smooth, stable, and adaptive movement.

## Overview

This project presents a novel approach to path planning that directly computes and applies jerk and snap control inputs in real-time. Unlike traditional minimum-jerk or minimum-snap algorithms that solve for position profiles offline, this system employs:

- Adaptive time horizons
- Dynamic error scaling 
- Real-time smoothing
- Direct jerk/snap control

The implementation consists of two main components:

1. MATLAB Plant Simulator
2. Qt/C++ Controller Implementation

## Key Features

- Real-time control with latency < 1ms
- Smooth, continuous motion without discontinuities
- Zero overshoot in >95% of test cases
- Handles random initial conditions
- Automated trial sequencing and data logging
- Comprehensive performance visualization

## Mathematical Model

The system models cursor location using state equations for position, velocity, and acceleration, with jerk as the control input:

latex
\begin{aligned}
x(k+1) &= x(k) + \Delta t \, v_x(k), \\
v_x(k+1) &= v_x(k) + \Delta t \, a_x(k), \\
a_x(k+1) &= a_x(k) + \Delta t \, j_x(k),
\end{aligned}


## Performance Results

Based on extensive testing:

| Control type | Min [s] | Max [s] | Avg [s] |
|--------------|---------|---------|---------|
| jerk (495 samples) | 16.8 | 44.8 | 26.2 |
| snap (470 samples) | 12.6 | 42.2 | 31.7 |

## Requirements

- MATLAB R2023b or newer
- Qt 6.8.0 or newer
- C++17 compatible compiler
- CMake 3.15+
- Python 3.8+ (for results analysis)

## Installation & Setup

1. Clone the repository:
```bash
git clone https://github.com/PaulRosu/High-Order-Plant-Control-System
cd High-Order-Plant-Control-System
```

2. MATLAB Setup:
   - Open MATLAB
   - Add the project's MATLAB directory to your path
   - Run `MousePositioningGeneralizat.m` to verify setup

3. Build Qt/C++ Controller:
```bash
mkdir build && cd build
cmake ..
cmake --build .
```

4. Python Environment Setup:
```bash
cd python_results_analysis
pip install -r requirements.txt
```

## Data Collection

The system automatically logs:
- Trial performance metrics to CSV files
- High-resolution PNG performance charts
- Convergence times and target positions
- Complete state trajectories

## Future Development

Planned extensions include:
- Integration with robotic hardware
- Advanced optimization layers
- Multipoint path constraints
- Full 3D kinematics support

## Documentation

For detailed technical information, refer to the included paper which provides comprehensive documentation of the mathematical foundations, implementation details, and performance analysis.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Author

Paul Rosu

## Citation

If you use this work in your research, please cite:

bibtex
@article{RosuPolynomial2024,
title={Polynomial-Based Path Planning with Direct Jerk and Snap Control},
author={Rosu, Paul},
year={2024}
}

For more information and access to all source code and result files, visit the [project repository](https://github.com/PaulRosu/High-Order-Plant-Control-System).

## Troubleshooting

Common issues and solutions:

1. CMake Build Errors
   - Ensure Qt 6.8.0 is properly installed and in PATH
   - Check CMAKE_PREFIX_PATH points to your Qt installation

2. MATLAB Simulation Issues
   - Verify MATLAB toolboxes: Control System, Signal Processing
   - Check path includes all required .m files

3. Performance Issues
   - Disable system power saving features
   - Close resource-intensive background applications
   - Monitor CPU usage during trials

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request