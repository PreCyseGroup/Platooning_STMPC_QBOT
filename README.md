# Platooning_STMPC_QBOT

A Set-Theoretic Control Strategy for a Platoon of Constrained Differential-Drive Robots with Inter-Vehicle Collision Avoidance

## Publication Information

**Authors:** Suryaprakash Rajkumar, Cristian Tiriolo, Walter Lucia  
**Accepted in:** IEEE Transactions on Automation Science and Engineering (TASE)  
**DOI:** [To be updated upon publication]

## Requirements

### Software Dependencies
```
MATLAB R2022B or later
Quanser QUARC software
```
### Hardwares Used
```
Quanser QBOT platform
Vicon Vero Motion tracking setup
Local Wifi Connection
```
## File Descriptions

### Parameter Files
```
leader_parameters.m
follower_1_parameters.m
follower_2_parameters.m
```
Preloads the necessary parameters for the leader and the followers.

### Hardware-in-the-Loop (HIL) Execution Files
```
QBOT_leader_3_agents.slx
QBOT_follower_1.slx
QBOT_follower_2.slx
```
Real-time implementation files for executing the platooning STMPC algorithms on the Quanser QBOT platform using QUARC software.

### Supporting Files
```
Vicon_server_multi_robot_test_3_robots.slx - simulink file for extracting VICON information, then publishing it through TCP
Start_robots.m - used to synchronize the clock on multiple simuinks

```
## Authors

- **Suryaprakash Rajkumar**
- **Cristian Tiriolo** 
- **Walter Lucia**

Research performed at Predictive Cyber-Physical System Security Group (PreCySe), Concordia University, Montreal, Canada


##  License

This project is licensed under the MIT License 

## Acknowledgments

This work was supported by  Natural Sciences and Engineering Research Council of Canada (NSERC) and Fonds de recherche du Québec – Nature et technologies (FRQNT).


