from stable_baselines3.common.env_checker import check_env
import stable_baselines3 as sb3 
import gymnasium as gym
import numpy as np 

import matplotlib.pyplot as plt

world_size = 10

class DroneSim(gym.Env):
    def __init__(self, render_mode = None): 
        #TODO: add sphere space
        self.action_space = gym.spaces.Box(-1,1, [3]) # simple XYZ coordinates defining movement step 
        self.observation_space = gym.spaces.Box(-world_size, world_size, [3]) # simple XYZ coordinates
        self.random = np.random.default_rng(seed=None)
        self._init_renderer(render_mode)

    def reset(self, *, seed=None, options=None): # rng
        self.random = np.random.default_rng(seed=seed) 
        self.observation_space.seed(seed=seed)
        self.state = self.observation_space.sample() # state is completely incapsulated by obs
        obs = self._get_obs()
        info = {"X" : self.state}
        return (obs, info)

    def step(self,action):
        """ returns: observation, reward, terminated, info """
        self.old_state = self.state.copy()
        self.state += action 
        obs = self._get_obs()
        reward = self._reward()
        truncated = not self.observation_space.contains(obs)
        terminated = bool(np.linalg.norm(self.state) < 1)
        return obs, reward, terminated, truncated, {"X":self.state}
    
    def _get_obs(self):
        return (0-self.state)/np.linalg.norm(self.state,2)

    def _reward(self): 
        return  (np.linalg.norm(self.old_state) - np.linalg.norm(self.state)).item()

    def _init_renderer(self, render_mode):
        # TODO: decompose renderer out of DroneSim class
        # TODO: add more efficent renderer based on PyQtGraph or done staticly - (after the training)
        self.render_mode = render_mode
        if self.render_mode == None: 
            return

        elif self.render_mode == "human": 
            plt.ion()
            fig = plt.figure()
            self.ax = fig.add_subplot(111, projection='3d')
            self._fignum =fig.number
        else:
            raise NotImplementedError

    def render(self):
        if self.render_mode == "human" and plt.fignum_exists(self._fignum): 
            line = np.concatenate([self.state.reshape(1,3), self.old_state.reshape(1,3)],axis=0) 
            self.ax.plot(*line.T, label="agent")
            plt.draw()
            plt.pause(0.00001)
        else:
            pass

    # def info(self): 
    #     pass 

    def close(self):
        if self.render_mode == None: 
            pass
        elif self.render_mode == "human":
            plt.ioff()
            plt.show()
        else:
            raise NotImplementedError


if __name__ == "__main__":
    check_env(DroneSim())
    np.set_printoptions(precision=3, sign=" ", floatmode="fixed")
    
    env = DroneSim("human") 
    obs, _info = env.reset()

    model = sb3.PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=50_000)

    for i in range(300):
        action, _state = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        print(f"{i:3} {info["X"]} {obs} {action} {reward:.3f}")
        env.render()
        # VecEnv resets automatically
        if terminated or truncated:
          obs, _info = env.reset()
    input("done.\n")
