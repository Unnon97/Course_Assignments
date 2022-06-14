import numpy as np
from agent import Agent


# TASK 1

class PolicyIterationAgent(Agent):

    def __init__(self, mdp, discount=0.9, iterations=100):
        """
        Your policy iteration agent take an mdp on
        construction, run the indicated number of iterations
        and then act according to the resulting policy.
        """
        self.mdp = mdp
        self.discount = discount
        self.iterations = iterations

        states = self.mdp.getStates()
        number_states = len(states)

        # Policy initialization
        # ******************
        # TODO 1.1.a)
        self.V = {key: 0.0 for key in states}
        # *******************

        self.pi = {s: self.mdp.getPossibleActions(
            s)[-1] if self.mdp.getPossibleActions(s) else None for s in states}
        counter = 0
        first_change_flag = False
        while True:
            # Policy evaluation
            for i in range(iterations):
                newV = {}
                for s in states:
                    a = self.pi[s]
                    # *****************
                    # TODO 1.1.b)
                    if a is None:
                        newV[s] = 0.0
                    else:
                        newV[s] = self.getQValue(s, a)
                # update value estimate
                self.V = newV

            # ******************
            policy_stable = True
            for s in states:
                actions = self.mdp.getPossibleActions(s)
                if len(actions) < 1:
                    self.pi[s] = None
                else:
                    old_action = self.pi[s]
                    # ************
                    # TODO 1.1.c)
                    valmax = self.V[s]
                    amax = old_action
                    for a in actions:
                        val = self.getQValue(s, a)
                        if val > valmax:
                            valmax = val
                            amax = a
                    self.pi[s] = amax
                    if old_action != amax:
                        policy_stable = False
            # ****************
            counter += 1
            if self.V[self.mdp.getStartState()] != 0 and not first_change_flag:
                first_change_flag = True
                print("Start State: " + str(self.mdp.getStartState()) + " has non zero value: " + str(
                    self.V[self.mdp.getStartState()]) + " after " + str(counter) + " Counts of policy iteration")
            if policy_stable:
                break

        print("Policy converged after %i iterations of policy iteration" % counter)

    def getValue(self, state):
        """
        Look up the value of the state (after the policy converged).
        """
        # *******
        # TODO 1.2.
        return self.V[state]
        # ********

    def getQValue(self, state, action):
        """
        Look up the q-value of the state action pair
        (after the indicated number of value iteration
        passes).  Note that policy iteration does not
        necessarily create this quantity and you may have
        to derive it on the fly.
        """
        # *********
        # TODO 1.3.
        sPrimeList = self.mdp.getTransitionStatesAndProbs(state, action)
        val = self.mdp.getReward(state, action, None)
        for sp in sPrimeList:
            val += sp[1]*0.9*self.V[sp[0]]
        return val
        # **********

    def getPolicy(self, state):
        """
        Look up the policy's recommendation for the state
        (after the indicated number of value iteration passes).
        """
        # **********
        # TODO 1.4.
        return self.pi[state]
        # **********

    def getAction(self, state):
        """
        Return the action recommended by the policy.
        """
        return self.getPolicy(state)

    def update(self, state, action, nextState, reward):
        """
        Not used for policy iteration agents!
        """

        pass
