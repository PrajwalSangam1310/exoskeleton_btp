#!/usr/bin/env python3

class WalkingStateMachine:
    def __init__(self):
        self.states = [0,1,2]
        # 0 -> stop
        # 1 -> right_stance, right leg is forward
        # 2 -> left_stance, left leg is forward

        self.state_names = ["Stand", "left_stance", "right_stance"]

        self.all_transitions = [0,1,2,3,4,5,6]

        # 0 for dont do anyting
        # 1 put left foot in front, left leg is swing phase
        # 2 put right foot in front, right leg is swing phase
        # 3 put left step forward to stop, left leg is swing
        # 4 put right step forward to stop, right leg is swing
        # 5 put left step forward to start, left leg is swing
        # 6 put right step forward to start, right leg is swing

        self.transition_matrix = [
                                    [0,5,6],
                                    [4,0,2],
                                    [3,1,0]
                                ]

        self.transition = 0 # start with doing nothing
        self.state = 0 # start with stopping
        self.next_state = 0

    def set_state(self, state):
        self.state = state
    
    def set_next_state(self, next_state):
        self.next_state = next_state
    
    def update_transition(self):
        self.transition = self.transition_matrix[self.state][self.next_state]
    
    def get_transition(self):
        self.update_transition()
        return self.transition
