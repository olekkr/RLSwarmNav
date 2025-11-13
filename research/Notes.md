Multi agent reinforcement learning (MARL) is a fusion of many fields and thus requires terminology from fields such as traditional single-agent reinforcement learning (RL), multiagent RL, Game theory. And other things such as evolutionary computation and optimization theory.

## Bacground
### Single agent RL 
Markov decision process (MDP) 
States, actions, actions, rewards, environment dynamics
$$ \left(X, U, f, \rho\right) $$ respectively.


$X$ is the set of all the possible environment states; $U$ is the finite set of all possible actions the agent can take;  $f$ is the function defining the probability distribution of the state after the agent takes an action $f : X \times U \times X \rightarrow [0, 1]$ ; finally $\rho: X \times U \times X \rightarrow \mathbb{R}$ is the reward function defining the agents purpose. 
... 




### Multi agent RL 


### Games
There exists many terms to define games in MARL
such as:
- static game ie. $X=\emptyset$ 
- terms regarding agent alignment:
	- competitive
	- cooperative
	- mixed 
- Also 
	 * zero-sum ie. agents total rewards sum to 0
	 * positive-sum ie. agents total rewards arent bounded
	 * general sum game
* repeated games ie. the static game is played by the same agents repeatedly 
* strategy : policy without a state $\sigma_i : U_i \rightarrow [0,1]$
* stochastic strategies: (cheese strat)
* Nash equilibrium: set of strategies where no agent can benefit by changing its strategy as long as other's strategies remain constant.
* 

## MARL characterisation 
### Taxonomy of MARL
homogeneous/heterogeneous
### Benefits of MARL
### Challenges in MARL 
* exploration v. exploitation
* convergence v. adaptation
* d
## MARL algorithms
### Team-Q
Assumes the optimal actions are unique and 
### Distributed Q-learning 

## Mechanisms for coordination