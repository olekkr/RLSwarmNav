```
Coordinating multiple micro-drones in a shared airspace presents significant challenges in trajectory planning due to the
need for collision avoidance, scalability, and real-time decision-making.
Reinforcement Learning (RL) offers a promising approach for learning complex coordinative behaviors when
constructing a plan that minimizes distance traveled.
The goal of this project is to implement and train a reinforcement learning model capable of generating safe, dynamically
feasible trajectories for a swarm of micro-drones, ensuring collision-free flight while achieving assigned goals.
The approach will be trained and validated in simulation using realistic Crazyflie dynamics, with deployment on real
Crazyflie drones as a demonstration of feasibility.
Centralized and distributed planning strategies will be explored, evaluating trade-oTs between scalability,
communication requirements, and robustness. Exploration of centralized versus distributed control strategies is
considered optional and will be pursued if time permits.
```
## Contract clauses:
* [ ] Implement an RL-based trajectory planner for multiple drones in simulation, achieving collision free coordination.
* [ ] Evaluation of trajectory planner in simulation.
* [ ] Demonstration of trajectory planner on real Crazyflie drones.
* [ ] Evaluation of planner performance with real Crazyflie drones.
* [ ] Evaluation of trajectory planner when scaling up to more drones, in simulation and real life.
## Optional Features:
* [ ] Research distributed planning strategies and compare them to centralized ones.
* [ ] Implement a distributed implementation of trajectory planner.
* [ ] Test and evaluate distributed planner in sim and real life