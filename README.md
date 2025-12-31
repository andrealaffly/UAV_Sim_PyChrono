# High-fidelity PyChrono-based Simulator for UAVs
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

This software currently offers an array of control solutions, spanning from classical continuous-time feedback-linearizing control laws combined with a PID (Proportional-Integral-Derivative) control law to robust MRAC (model reference adaptive control) systems that can enforce user-defined constraints. For additional details, see our [Wiki]([https://www.genome.gov/](https://github.com/andrealaffly/UAV_Sim_PyChrono/wiki))

For further details on these control architectures, refer to the publications found [here](https://www.acslstack.com/Journals).

Future versions of the software will include additional control systems.

## Maintenance Team

- [**Andrea L'Afflitto**](https://github.com/andrealaffly)
- [**Mattia Gramuglia**](https://github.com/mattia-gramuglia)

For more information, visit [acslstack.com](https://www.acslstack.com/).

[![ACSL Flight Stack Logo](https://lafflitto.com/images/ACSL_Logo.jpg)](https://lafflitto.com/ACSL.html)


---

This software is distributed under a permissive **3-Clause BSD License**.
