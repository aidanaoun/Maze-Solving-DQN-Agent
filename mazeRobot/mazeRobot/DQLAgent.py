import random
from collections import deque
import numpy as np
import tensorflow as tf

from tensorflow.keras import backend as K
from tensorflow.keras.layers import Dense, Input
from tensorflow.keras.models import Sequential
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.models import load_model


def build_q_network(state_size, action_size):
    builtNetwork = Sequential([
        Input(shape=(state_size,)),
        Dense(37, activation='relu'),
        Dense(37, activation='relu'),
        Dense(action_size, activation='linear')  # Output layer is Q-values that correspond to actions
    ])
    builtNetwork.compile(optimizer=Adam(learning_rate=0.0005), loss='mse')
    return builtNetwork



class ReplayBuffer:
    def __init__(self, max_size=100000):
        self.buffer = deque(maxlen=max_size)
    def add(self, event):
        self.buffer.append(event)
    def sample(self, batch_size):
        return random.sample(self.buffer, batch_size)  # returns batch_size events
    def clearBuffer(self, max_size=100000):
        self.buffer.clear()
    def __len__(self):
        return len(self.buffer)



class DQLAgent:
    def __init__(self, brain, state_size, action_size):
        model_to_load = "maze_model_final.keras"
        self.freshModel = False              #  SWITCH THIS TO CHOOSE BETWEEN PRETRAINED OR RANDOM MODEL
        randomActions = False                #  SWITCH THIS TO CHOOSE BETWEEN RANDOM OR DELIBERATE ACTIONS

        self.brain = brain
        self.state_size = state_size
        self.action_size = action_size
        self.gamma = 0.992
        self.epsilon = 1.0
        self.epsilon_min = 0.01
        self.batch_size = 75

        if randomActions:
            self.epsilon_decay = 0.999
        else: 
            self.epsilon_decay = 0.01            

        self.replayBuffer = ReplayBuffer()

        if self.freshModel:
            self.onlineNetwork = build_q_network(state_size, action_size)
            self.targetNetwork = build_q_network(state_size, action_size)
        else:
            self.onlineNetwork = load_model(model_to_load)
            self.targetNetwork = load_model(model_to_load)
        self.updateTargetNetwork()



    def updateTargetNetwork(self):
        self.targetNetwork.set_weights(self.onlineNetwork.get_weights())



    def getNextAction(self, state):
        if np.random.rand() < self.epsilon:
            return np.random.randint(self.action_size)
        state_tensor = tf.convert_to_tensor(np.array([state]), dtype=tf.float32)
        qValues = self.onlineNetwork(state_tensor, training=False).numpy()
        return np.argmax(qValues[0])



    def storeEvent(self, state, action, reward, next_state, done):
        self.replayBuffer.add((state, action, reward, next_state, done))



    def updateOnlineNetwork(self):
        if len(self.replayBuffer) < self.batch_size:
            return
        # Sample and convert to tensors immediately
        shuffledBatch = self.replayBuffer.sample(self.batch_size)
        states, actions, rewards, next_states, dones = zip(*shuffledBatch)
        state_tensor = tf.convert_to_tensor(np.array(states), dtype=tf.float32)
        next_state_tensor = tf.convert_to_tensor(np.array(next_states), dtype=tf.float32)
        
        # Batch Predictions
        onlineQValsBatch = self.onlineNetwork(state_tensor, training=False).numpy()
        nextStateQValsBatch = self.targetNetwork(next_state_tensor, training=False).numpy()

        # Vectorized Target Calculation 
        targets = onlineQValsBatch.copy()
        
        # Calculate the max Q-value for all next states at once and then Bellman Equation
        max_next_q = np.max(nextStateQValsBatch, axis=1)
        updated_q_values = rewards + (self.gamma * max_next_q * (1 - np.array(dones, dtype=np.float32)))
        
        # Update only the columns for the actions that were taken and train
        targets[np.arange(self.batch_size), actions] = updated_q_values
        self.onlineNetwork.train_on_batch(state_tensor, targets)

        # Decay epsilon
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay



    def saveModel(self, fileName="trained_maze_model.keras"): # Saves the architecture, weights, and optimizer state
        self.onlineNetwork.save(fileName)
        print(f"Model saved successfully as {fileName}")

