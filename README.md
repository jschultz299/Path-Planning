# Path-Planning

Motion Planning for robotics consists of moving a robot from a start state to a goal state while avoiding obstacles as well as obeying constraints such as joint and torque limits.

Path Planning is a subset of the larger field of Motion Planning. Path planning consists of finding the geometric path that connects a start state to a goal state, while avoiding obstacles. In path planning, however, we ignore robot dynamics and additional environmental and motion constraints.

In this repository, we explore two common path planning algorithms:
1. A* search algorithm
2. Rapidly Exploring Random Trees

## 1) A* Search Algorithm

A* is one of the most common and efficient graph search algorithms. It finds the optimal path from a start node to a goal node, connecting intermediate nodes via edges.

In this code, I have implemented the A* algorithm on the example Kevin Lynch presents in his video on Path Planning [here](https://youtu.be/ZI800-2jv38).



This project uses the [CartPole](https://www.gymlibrary.dev/environments/classic_control/cart_pole/) environment. The goal of the project is the train the agent to learn how to balance the pendulum upright for an extended period of time. The agent is rewarded if the pendulum remains upright within a certain range of joint angles.

To run the program:

```bash
python Pendulum.py
```

Below you can see a demonstration of the agent performing random actions for 5 episodes.

<img src="https://github.com/jschultz299/ReinforcementLearning/blob/main/OpenAiGym/Images/Pendulum/Demo%20Environment.gif" width=50%>

<br>

Training a PPO model with 20,000 timesteps

```bash
model = PPO('MlpPolicy', env, verbose=1, tensorboard_log=log_path)
model.learn(total_timesteps=20000, callback=eval_callback)
```

is sufficient for the agent to learn how to balance the pendulum, shown below.

<img src="https://github.com/jschultz299/ReinforcementLearning/blob/main/OpenAiGym/Images/Pendulum/Evaluate%20Model.gif" width = 50%>

Check out the tensorboard logs for the PPO model trained with 20k timesteps [here](https://github.com/jschultz299/ReinforcementLearning/tree/main/OpenAiGym/Images/Pendulum/Tensorboard%20Logs).

## 2) Breakout

This project uses the [Breakout](https://www.gymlibrary.dev/environments/atari/breakout/) environment. The goal of the project is to play the Atari game Breakout. The agent is rewarded for each brick it breaks, and penalized for losing lives.

To run the program:

```bash
python Breakout.py
```
Below you can see a demonstration of the agent playing Breakout with random actions.

<img src="https://github.com/jschultz299/ReinforcementLearning/blob/main/OpenAiGym/Images/Breakout/Demo%20Environment.gif" width = 50%>

Training an A2C model with 2 million timesteps

```bash
model = A2C('CnnPolicy', env, verbose=1, tensorboard_log=log_path)
model.learn(total_timesteps=100000)
```
results in an average reward of approximately 23 bricks broken per game, shown below.

<img src="https://github.com/jschultz299/ReinforcementLearning/blob/main/OpenAiGym/Images/Breakout/Evaluate%20Model.gif" width = 40%>

Training for significantly longer might improve performance.

Check out the logs for an A2C model trained with 100k timesteps [here](https://github.com/jschultz299/ReinforcementLearning/tree/main/OpenAiGym/Images/Breakout/Tensorboard_Logs).

## 3) Self Driving

This project uses the [CarRacing](https://www.gymlibrary.dev/environments/box2d/car_racing/) environment. The goal of this project is for the agent (the car) to drive along the track for as long as possible. The agent receives rewards for remaining on the track, and is penalized for leaving the track as well as penalzed slightly for each timestep. The actions the agent may take are the direction to turn the wheels as well as acceleration and braking. All of the actions are in the continuous space. The track is considered solved if the agent receives a total score of 900.

To run the program:

```bash
python SelfDriving.py
```
Below you can see a demonstration of the agent driving along the track with random inputs.

<img src="https://github.com/jschultz299/ReinforcementLearning/blob/main/OpenAiGym/Images/SelfDriving/Demo%20Environment.gif" width = 50%>

Training a PPO model with just 10,000 timesteps

```bash
model = PPO('CnnPolicy', env, verbose=1, tensorboard_log=log_path)
model.learn(total_timesteps=10000)
```
results in an agent with poor performance, shown below.

<img src="https://github.com/jschultz299/ReinforcementLearning/blob/main/OpenAiGym/Images/SelfDriving/Evaluate%2010k%20Model.gif" width = 50%>

This model achieved an average score of -40.6.

Check out the tensorboard logs for the PPO model trained with 10k timesteps [here](https://github.com/jschultz299/ReinforcementLearning/tree/main/OpenAiGym/Images/SelfDriving/Tensorboard_Logs/10k_Model).

Training the model for more timesteps, this time 200,000

```bash
model = PPO('CnnPolicy', env, verbose=1, tensorboard_log=log_path)
model.learn(total_timesteps=200000)
```
results in an agent with much better performance, shown below.

<img src="https://github.com/jschultz299/ReinforcementLearning/blob/main/OpenAiGym/Images/SelfDriving/Evaluate%20200k%20Model.gif" width = 50%>

This model achieved an average score of 436.2, and was even able to fully solve the track on occasion.

Check out the tensorboard logs for the PPO model trained with 10k timesteps [here](https://github.com/jschultz299/ReinforcementLearning/tree/main/OpenAiGym/Images/SelfDriving/Tensorboard_Logs/200k_Model).

Because the actions are continuous, the model has a hard time learning the appropriate actions to take. It is possible that training for longer might result in better performance. [NotAnyMike](https://github.com/NotAnyMike) tried adjusting the action space to only use discrete inputs, which simplified the problem somewhat. Check out his solution [here](https://notanymike.github.io/Solving-CarRacing/).

## 4) Super Mario Bros

This project uses the [SuperMarioBros](https://pypi.org/project/gym-super-mario-bros/) environment wrapper. The goal of the project is the train the agent to learn how to play Super Mario Bros. The agent is rewarded for moving to the right and for reaching the flag. The agent is penalized for how much time it takes to reach the flag and for deaths.

Mario has a number of actions he can take, which correspond to the input buttons on a game controller. There are three different [action spaces](https://github.com/Kautenja/gym-super-mario-bros/blob/master/gym_super_mario_bros/actions.py) with varying levels of complexity. In this project, the "Simple Movement" action space is used.

To run the program:

```bash
python SuperMarioBros.py
```
Below you can see a demonstration of the agent playing Super Mario Bros. with random actions.

INSERT VIDEO HERE

An additional layer of complexity is that we can stack frames on top of each other to give the agent a sense of 'memory'. In this case, we not only tell the agent to pay attention to the current frame, but also the previous 7 frames as well. Below you can see an image of the stacked frames.

INSERT IMAGE HERE...

COMING SOON...

## Acknowledgments
Much of the information here came from Kevin Lynch's book, [Modern Robotics: Mechanics, Planning, and Control](http://hades.mech.northwestern.edu/images/7/7f/MR.pdf) as well as his corresponding YouTube series, found [here](https://www.youtube.com/playlist?list=PLggLP4f-rq02vX0OQQ5vrCxbJrzamYDfx).


