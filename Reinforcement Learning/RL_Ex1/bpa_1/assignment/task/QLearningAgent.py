import numpy as np
import util
from agent import Agent


# TASK 3

class QLearningAgent(Agent):

    def __init__(self, actionFunction, discount=0.9, learningRate=0.1, epsilon=0.3):
        """ A Q-Learning agent gets nothing about the mdp on construction other than a function mapping states to
        actions. The other parameters govern its exploration strategy and learning rate. """
        self.setLearningRate(learningRate)
        self.setEpsilon(epsilon)
        self.setDiscount(discount)
        self.actionFunction = actionFunction

        self.qInitValue = 0  # initial value for states
        self.Q = {}
        self.V = {}

    def setLearningRate(self, learningRate):
        self.learningRate = learningRate

    def setEpsilon(self, epsilon):
        self.epsilon = epsilon

    def setDiscount(self, discount):
        self.discount = discount

    def getValue(self, state):
        """ Look up the current value of the state. """
        # *********
        # TODO 3.1.
        self.V[state] = 0
        if state in self.Q:
            self.V[state] = max(self.Q[state].values())
        return self.V[state]
        # *********

    def getQValue(self, state, action):
        """ Look up the current q-value of the state action pair. """
        # *********
        # TODO 3.2.
        if state in self.Q:
            return self.Q[state][action]
        else:
            return 0
        # *********

    def getPolicy(self, state):
        """ Look up the current recommendation for the state. """
        # *********
        # TODO 3.3.
        if state not in self.Q:
            self.pi = self.getRandomAction(state)
        else:
            all_actions = self.Q[state]
            self.pi = util.Counter.argMax(all_actions)
        return self.pi
        # *********

    def getRandomAction(self, state):
        all_actions = self.actionFunction(state)
        if len(all_actions) > 0:
            # *********
            return np.random.choice(all_actions)
            # *********
        else:
            return "exit"

    def getAction(self, state):
        """ Choose an action: this will require that your agent balance exploration and exploitation as appropriate. """
        # *********
        # TODO 3.4.
        if state in self.Q:
            if np.random.rand() < self.epsilon:
                return self.getRandomAction(state)
            else:
                return self.getPolicy(state)
        else:
            return self.getRandomAction(state)
        # *********

    def update(self, state, action, nextState, reward):
        """ Update parameters in response to the observed transition. """
        # *********
        # TODO 3.5.
        if state not in self.Q:
            actionslist = self.actionFunction(state)
            self.Q[state] = {actlist: 0 for actlist in actionslist}
        else:
            bestQNext = self.getValue(nextState)
            self.Q[state][action] += self.learningRate * \
                (reward + (self.discount*bestQNext) -
                 self.getQValue(state, action))

        # *********
