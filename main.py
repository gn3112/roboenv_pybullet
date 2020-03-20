from arm import Arm
import time

agent = Arm(headless=True)

# Init replay buffer 
# Init networks (DQN: Q-networks)
# random policy add to buffer
# sample batch
# train network with batch
# log results/test



for i in range(50):
    time.sleep(0.1)
    agent.step([0,0,0.1])

agent.reset()

for i in range(100):
    time.sleep(0.1)
    agent.step([0.2,0.1,0])
