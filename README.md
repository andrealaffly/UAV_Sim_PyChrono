# High-fidelity PyChrono-based Simulator
[![BSD License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE.txt)
[![Website](https://img.shields.io/badge/Website-acslstack.com-green)](https://www.acslstack.com/)


## Introduction

The **UAV_Sim_PyChrono** is a high-fidelity PyChrono-based simulator designed for multi-rotor UAVs (Uncrewed Aerial Vehicles).


## Outlook on the Control Architecture

Autonomous UAVs with collinear propellers are inherently under-actuated. For this reason, the software includes:

- **Inner Loop**: Handles the rotational dynamics.
- **Outer Loop**: Handles the translational dynamics.

Both loops are governed by nonlinear equations of motion.

### Available Control Solutions

This software currently offers two control solutions for the inner and outer loops:

1. **Continuous-Time Feedback-Linearizing Control Law** combined with a **PID (Proportional-Integral-Derivative) Control Law**.
2. The above control law is augmented by a **Robust Model Reference Adaptive Control (MRAC) System**, incorporating a simplified quadratic-in-the-velocity aerodynamic model.

For further details on these control architectures, refer to the publications found [here](https://www.acslstack.com/Journals).

Future versions of the software will include additional control systems.

## Maintenance Team

- [**Andrea L'Afflitto**](https://github.com/andrealaffly)
- [**Mattia Gramuglia**](https://github.com/mattia-gramuglia)

For more information, visit [acslstack.com](https://www.acslstack.com/).

[![ACSL Flight Stack Logo](https://lafflitto.com/images/ACSL_Logo.jpg)](https://lafflitto.com/ACSL.html)

## Installation Instructions (Linux)
1. Open Visual Studio (VS) code
2. Navigate to 'File' -> 'Open Folder' -> select git repository
3. Open 'install' directory, then right click 'install_pychrono_linux.sh' -> 'Open in Integrated Terminal'
4. Run 'chmod +x install_pychrono_linux.sh' in terminal
5. Run './install_pychrono_linux.sh' in terminal
6. In termaial window, navigate to '...' -> 'Kill terminal' once Miniconda successfully installs
7. Right click 'install_pychrono_linux.sh' -> 'Open in Integrated Terminal' again
8. Rerun './install_pychrono_linux.sh'
9. Press 'Ctrl + Shift + P' and type 'Python: Select Interpreter' + Enter
10. Select 'chrono (3.10.18)'
11. Run main.py in VS code

---

This software is distributed under a permissive **3-Clause BSD License**.
