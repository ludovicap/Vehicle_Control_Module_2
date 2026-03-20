# 🚗 Vehicle Control Module 2 – Project

## 📌 Overview
Design and simulation of an LPV-based lateral controller for trajectory tracking of an autonomous vehicle in an urban scenario using MATLAB & Simulink.

---

## 📂 Structure
- `main.m` → run simulation  
- `scenario_build.m` → generate scenario  
- `reference_data.mat` → reference trajectory  
- `simulation.slx` → main Simulink model  
- `VehiclePlant.slx` → vehicle model  
- `fun_H2.m`, `AugmentedState.m` → controller design  
- `figures_pdf/` → report figures  
- `immagini_simulink/` → Simulink screenshots  

---

## ⚙️ Requirements
- MATLAB  
- Simulink  
- Automated Driving Toolbox  
- Control System Toolbox  

---

## ▶️ Run
    main

---

## 🚘 Description
- Urban driving scenario  
- Reference: position, yaw, velocity  
- LPV control (velocity scheduling)  
- Bicycle model + actuator  

---

## 📊 Outputs
- Trajectory tracking (reference vs actual)  
- Yaw tracking performance  
- Velocity tracking  
- Steering input behavior  
- Smooth LPV weights (sigmoid-based scheduling)  
- Robustness to noise/disturbances  
- Performance indices (RMSE, max steering)   

---

## 👩‍🎓 Author
Ludovica Perroni – University of Calabria
