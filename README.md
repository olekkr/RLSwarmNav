# RL drone-swarm planner



## Setup:
install following the instructions:
[gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones)

Make sure to setup `config.py` before running 

### Training the policy:
> python train.py
input a name, or leave it blank.


### Evaluating in simulation:
> python sim_eval.py \[path to model\]

If no path is provided, pick the path with the target policy

### Evaluating on real drones:
> python swarm_run.py \[path to model\]

If no path is provided, pick the path with the target policy
