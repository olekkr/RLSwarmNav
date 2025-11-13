from stable_baselines3.common.env_checker import check_env
import stable_baselines3 as sb3 
import gymnasium as gym
import numpy as np 

import matplotlib.pyplot as plt

world_dims = np.array([64,64,64])

class DroneSim(gym.Env):
    def __init__(self, render_mode = None): 
        #TODO: add sphere space
        self.action_space = gym.spaces.Box(-1,1, [3]) # simple XYZ coordinates defining movement step 
        self.observation_space = gym.spaces.Box(-world_dims, world_dims, [3]) # simple XYZ coordinates
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
        reward = self._reward().item()
        truncated = True if np.linalg.norm(self.state) < 1. else False
        terminated = not self.observation_space.contains(obs)
        return obs, reward, terminated, truncated, {"X":self.state}
    
    def _get_obs(self):
        return (0-self.state)/np.linalg.norm(self.state,2)

    def _reward(self): 
        return  (np.linalg.norm(self.old_state) - np.linalg.norm(self.state))

    def _init_renderer(self, render_mode):
        # TODO: decompose renderer out of DroneSim class
        # TODO: add more efficent renderer based on PyQtGraph or done staticly - after the trainin
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



# TODO: extend droneSim into petting zoo 



# class Trainer:
#     def __init__(self): 
#         pass 
#
#     def train(self): 
#         pass 
#
#     # def save(self): 
#     #     pass
#     #
#     # def load(self):
#     #     pass
#
#     def get_policy(self): 
#         pass 


class Runtime:
    def __init__(self): 
        pass 

    def run(self):
        pass 



if __name__ == "__main__":

    # episode_over = False 
    # total_reward = 0
    # iterations = 0 
    # while not episode_over:
    #     iterations += 1 
    #     if iterations > 100: 
    #         break
    #     # Choose an action: 0 = push cart left, 1 = push cart right
    #     action = env.action_space.sample()  # Random action for now - real agents will be smarter!
    #
    #     # Take the action and see what happens
    #     observation, reward, terminated, truncated, info = env.step(action)
    #     # reward: +1 for each step the pole stays upright
    #     # terminated: True if pole falls too far (agent failed)
    #     # truncated: True if we hit the time limit (500 steps)
    #     env.render()
    #     total_reward += reward
    #     episode_over = terminated or truncated

    # print(f"Episode finished! Total reward: {total_reward}")
    # env.close()
    
    env = DroneSim("human") 
    obs, _info = env.reset()

    model = sb3.PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=10_000)

    for i in range(1000):
        action, _state = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        print(i, info["X"], action, reward)
        env.render()
        # VecEnv resets automatically
        if terminated or truncated:
          obs, _info = env.reset()
    check_env(DroneSim())
