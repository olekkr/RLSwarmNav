## Decision Log:
- As of 2025-10-06 i have decided to focus on:
```
Creating a policy for drone swarm, moving in such a way that it mutually avoids collsions and moves to target destination.
Inference is done on an external computer, and sent via crazyRadio. 
```
Meaning there wont be a interpetable layer between observation and action. 
Cons: Will be harder to debug. 
Pros: simpler implementation; more robust; no phase transitions; demonstrates capabilities of RL;

---- 
2025-10-06 decided to use stable baselines as my RL model library.
To minimize time spent implementing the RL algorithms. After the base 

---
2025-10-21 
Decided to use high level absolute position controller as a fallback in case of hitting unexpected params. 

~~--- 
2025-10-24 Decided to do end to end learning, meaning the outputs actions are the set of possible attitude accelerations. 
~~
2025-11-03 Decided to use high-level controller for control, 
- which means i don need to use a physics engine i can just use a kinematic simulation.
- 
## TODO:
~~* [ ] Make a modified version of the pybullet environement in [link](https://github.com/utiasDSL/gym-pybullet-drones). (make single drone hover with PPO)~~
* [ ] Read the craazyflie *get started* docs
~~* [ ] Port the CrazyFlie attitude -> abs_attitude PID controller into new env ~~
* [ ] Implement guardrails for control policy for safety. "safety filter"
	* [ ] What is ORCA/VO?
	* [ ] write the safety rutine 
* [ ] Identify crazyflie-and sim-compatible API for observation and control 
	* [ ] Write the API out
* [ ] Add noise to the pybullet env
	* [ ] Latency
	* [ ] Jitter
	* [ ] Localization error
		* [ ] stochastic 
		* [ ] and Drift
	* [ ] Missing data
	* [ ] Observation clamping (similar to real obs)
	* [ ] 