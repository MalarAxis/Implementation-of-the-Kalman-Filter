# 📍 Extended Kalman Filter: 2D Range-Bearing Tracking

This repository contains a simulation of a 2D tracking problem using the **Extended Kalman Filter (EKF)** with **nonlinear measurements**.

## 🧠 Project Overview

- 📌 Track a moving object in 2D using noisy range and bearing measurements.
- 🧮 Estimate position and velocity via an EKF.
- 📈 Visualize estimation accuracy, uncertainties, and trajectory.

## 📂 Contents

- `Kalman_filter.m`: Main MATLAB script implementing the EKF.
- `EKF_Theory_and_Analysis.ipynb`: Interactive Jupyter notebook with:
  - Mathematical derivation
  - Step-by-step explanation
  - Visualizations and insights

## 📋 Requirements

- MATLAB R2020b or newer (or any version with `mvnrnd`, `wrapToPi`)
- (Optional) Python + Jupyter + MATLAB Engine for notebook integration

## 🚀 Running the Simulation

1. Clone the repository:
    ```bash
    git clone https://github.com/yourusername/kalman-ekf-tracker.git
    cd kalman-ekf-tracker
    ```

2. Open `Kalman_filter.m` in MATLAB.

3. Run the script to simulate and generate all plots.

## 🧾 Output

- True vs estimated state (position & velocity)
- Trajectory comparison
- Covariance and variance analysis
- Innovation behavior

## 📓 Learn More

For full theory, derivations, and visual breakdowns, check the 📘 [EKF_Theory_and_Analysis.ipynb](docs/EKF_Theory_and_Analysis.ipynb) notebook.

## 📬 Contact

Have questions or suggestions? Feel free to open an issue or reach out.

---
