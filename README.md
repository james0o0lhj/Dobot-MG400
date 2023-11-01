# Dobot-MG400

This repo contributes three python files

1. dobot_Robot.py: demo to control mg400 based on python api.   
   API source https://github.com/Dobot-Arm/TCP-IP-4Axis-Python-CMD
   
2. dobot_calib.py: Hand-to-Eye Calibration for mg400 with realsense D435.   
   Inspired by https://github.com/heretic1993/autoCalibration  
   Tutorial: https://programmersought.com/article/31073681999/#google_vignette
   
3. dobot_sim.py: pybullet-based simulation for dobot mg400  
   Fix the relationship between upper and lower links.