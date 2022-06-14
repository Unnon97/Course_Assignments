from agent import Agent
import numpy as np

# TASK 2


class ValueIterationAgent(Agent):

    def __init__(self, mdp, discount=0.9, iterations=100):
        """
        Your value iteration agent take an mdp on
        construction, run the indicated number of iterations
        and then act according to the resulting policy.
        """
        self.mdp = mdp
        self.discount = discount
        self.iterations = iterations
        states = self.mdp.getStates()
        number_states = len(states)
        # *************
        #  TODO 2.1 a)
        self.V = [0] * number_states
        conv_var = True
        # ************
        newV = [0] * number_states
        for i in range(iterations):

            for sidx, s in enumerate(states):
                actions = self.mdp.getPossibleActions(s)

                # **************
                # TODO 2.1. b)

                if actions != ():
                    action_value = []
                    for a in actions:
                        trans_probs = self.mdp.getTransitionStatesAndProbs(
                            s, a)
                        #action_value = []
                        p = [ele[1] for ele in trans_probs]
                        next_state = [ele[0] for ele in trans_probs]
                        r = self.mdp.getReward(s, a, None)
                        V_ns_idx = [states.index(t) for t in next_state]
                        V_ns = [self.V[i] for i in V_ns_idx]
                        term22 = float(np.sum(np.array(p)*np.array(V_ns)))
                        tmp1 = r + self.discount*term22
                        action_value.append(tmp1)
                    newV[sidx] = max(action_value)

            if(conv_var and (np.abs(np.array(self.V)-np.array(newV)) < 0.000000001).all()):
                print("Conv at Iteration ", i)
                conv_var = False

            self.V = newV.copy()

    def getValue(self, state):
        """
        Look up the value of the state (after the indicated
        number of value iteration passes).
        """
        # **********
        # TODO 2.2
        states = self.mdp.getStates()
        idx = states.index(state)
        return self.V[idx]
        # **********

    def getQValue(self, state, action):
        """
        Look up the q-value of the state action pair
        (after the indicated number of value iteration
        passes).  Note that value iteration does not
        necessarily create this quantity and you may have
        to derive it on the fly.
        """
        # ***********
        # TODO 2.3.

        states = self.mdp.getStates()
        trans_probs = self.mdp.getTransitionStatesAndProbs(state, action)
        p = [ele[1] for ele in trans_probs]
        next_state = [ele[0] for ele in trans_probs]
        r = self.mdp.getReward(state, action, None)
        V_ns_idx = [states.index(t) for t in next_state]
        V_ns = [self.V[i] for i in V_ns_idx]

        term22 = float(np.sum(np.array(p)*np.array(V_ns)))
        tmp1 = r + self.discount*term22
        return tmp1
        # **********

    def getPolicy(self, state):
        """
        Look up the policy's recommendation for the state
        (after the indicated number of value iteration passes).
        """
        actions = self.mdp.getPossibleActions(state)
        actions = list(actions)
        if len(actions) < 1:
            return None

        else:
            # TODO 2.4
            arr = [self.getQValue(state, a) for a in actions]
            return actions[arr.index(max(arr))]

    def getAction(self, state):
        """
        Return the action recommended by the policy.
        """
        return self.getPolicy(state)

    def update(self, state, action, nextState, reward):
        """
        Not used for value iteration agents!
        """

        pass
