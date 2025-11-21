import numpy as np 
import functools

import gymnasium as gym

from pettingzoo.test import api_test, parallel_api_test
import numpy as np
import gymnasium as gym
from pettingzoo import ParallelEnv, AECEnv  
import matplotlib.pyplot as plt
import matplotlib.cm as cm

COLOR_MAP = cm.get_cmap("tab10")
DEFAULT_TRAIL_LEN = 20

world_size = 50
num_of_agents = 3



class SwarmSim(ParallelEnv):
    metadata = {"name": "swarm_sim_v0"}

    # --- NEW: simple storage for trails ---
    trail_len = DEFAULT_TRAIL_LEN
    trails = {}        # trails[agent] = list of np.array([x,y,z])


    def __init__(self, render_mode=None):
        self.possible_agents = [f"drone_{i}" for i in range(num_of_agents)]
        self.agents = self.possible_agents[:]    # active agents each episode

        self.render_mode = render_mode
        self._init_renderer(render_mode)

        # RNG
        self.random = np.random.default_rng()
        
        self._obs_spaces = {
            agent: gym.spaces.Box(-world_size, world_size, (3,), dtype=np.float32)
            for agent in self.possible_agents
        }
        self._act_spaces = {
            agent: gym.spaces.Box(-1, 1, (3,), dtype=np.float32)
            for agent in self.possible_agents
        }



    def observation_space(self, agent):
        return self._obs_spaces[agent]

    def action_space(self, agent):
        return self._act_spaces[agent]


    def reset(self, seed=None, options=None):
        if seed is not None:
            self.random = np.random.default_rng(seed)

        self.agents = self.possible_agents[:]

        # State for each agent
        self._state = {
            agent: self.random.uniform(-world_size, world_size, size=3).astype(np.float32)
            for agent in self.agents
        }

        self.old_state = {a: s.copy() for a, s in self._state.items()}

        observations = {a: self._get_obs(a) for a in self.agents}
        infos = {a: {"X": self._state[a]} for a in self.agents}

        self.trails = {agent: [] for agent in self.possible_agents}

        return observations, infos

    def step(self, actions):
        """
        actions: dict(agent -> np.array(3,))
        """
        if not self.agents:
            return {}, {}, {}, {}, {}

        rewards = {}
        terminations = {}
        truncations = {}
        infos = {}

        for agent, action in actions.items():
            self.old_state[agent] = self._state[agent].copy()

            # simple move
            self._state[agent] = self._state[agent] + action

            # reward = move toward origin
            rewards[agent] = (
                np.linalg.norm(self.old_state[agent])
                - np.linalg.norm(self._state[agent])
            ).item()

            # termination if inside radius
            terminations[agent] = bool(np.linalg.norm(self._state[agent]) < 1)

            # truncation if out of bounds
            obs = self._get_obs(agent)
            truncations[agent] = not self.observation_space(agent).contains(obs)

            infos[agent] = {"X": self._state[agent]}

        for agent in self.agents:
            pos = self._state[agent]
            trail = self.trails[agent]
            trail.append(pos.copy())
            if len(trail) > self.trail_len:
                trail.pop(0)


        # Remove finished agents
        self.agents = [
            a for a in self.agents if not (terminations[a] or truncations[a])
        ]

        # Observations for remaining agents
        observations = {a: self._get_obs(a) for a in self.agents}

        return observations, rewards, terminations, truncations, infos

    def _get_obs(self, agent):
        s = self._state[agent]
        return (0 - s) / (np.linalg.norm(s) + 1e-8)

    def _agent_color(self, agent: str):
        idx = int(agent.split("_")[1])       # "drone_2" -> 2
        return COLOR_MAP(idx)                # returns RGBA in [0,1]

    def _init_renderer(self, render_mode):
        self.render_mode = render_mode
        if render_mode == "human":
            plt.ion()
            fig = plt.figure()
            self.ax = fig.add_subplot(111, projection='3d')
            self._fignum = fig.number

    def render(self):
        if self.render_mode != "human":
            return

        if not plt.fignum_exists(self._fignum):
            return

        self.ax.clear()

        for agent in self.agents:
            pos = self._state[agent]
            col = self._agent_color(agent)

            # draw trail
            trail = self.trails.get(agent, [])
            if len(trail) > 1:
                t = np.array(trail)
                self.ax.plot(
                    t[:, 0], t[:, 1], t[:, 2],
                    color=col,
                    linewidth=1.5,
                    alpha=0.7,
                )

            # draw current pos
            self.ax.scatter(
                pos[0], pos[1], pos[2],
                color=col,
                s=60,
                edgecolor="black",
            )

        plt.draw()
        plt.pause(0.001)




    def close(self):
        if self.render_mode == "human":
            plt.ioff()
            plt.show()




if __name__ == "__main__":
    env = SwarmSim(render_mode="human")
    
    observations, infos = env.reset()

    for step in range(300):

        # simple demonstration policy: follow obs direction
        actions = {agent: obs for agent, obs in observations.items()}

        observations, rewards, terminations, truncations, infos = env.step(actions)

        print(f"Step {step}")
        for agent in env.possible_agents:
            if agent in infos:
                print(
                    f"  {agent}: pos={infos[agent]['X']} "
                    f"reward={rewards[agent]:.3f}"
                )

        env.render()

        if len(env.agents) == 0:
            print("Episode finished.\n")
            observations, infos = env.reset()

    input("done.\n")

