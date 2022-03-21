import random
import math
from constants import *


class FiniteStateMachine(object):
    """
    A finite state machine.
    """
    def __init__(self, state):
        self.state = state

    def change_state(self, new_state):
        self.state = new_state

    def update(self, agent):
        self.state.check_transition(agent, self)
        self.state.execute(agent)


class State(object):
    """
    Abstract state class.
    """
    def __init__(self, state_name):
        """
        Creates a state.

        :param state_name: the name of the state.
        :type state_name: str
        """
        self.state_name = state_name

    def check_transition(self, agent, fsm):
        """
        Checks conditions and execute a state transition if needed.

        :param agent: the agent where this state is being executed on.
        :param fsm: finite state machine associated to this state.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

    def execute(self, agent):
        """
        Executes the state logic.

        :param agent: the agent where this state is being executed on.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")


class MoveForwardState(State):
    def __init__(self):
        super().__init__("MoveForward")
        self.count = 0

    def check_transition(self, agent, state_machine):
        # if the roomba hits a wall
        if agent.get_bumper_state():
            agent.set_bumper_state(False) # Set bumper state to false
            state_machine.change_state(GoBackState()) # Go into GoBack State
        if self.count*SAMPLE_TIME >= MOVE_FORWARD_TIME: # If MoveFoward is called more times than MOVE_FOWARD_TIME
            state_machine.change_state(MoveInSpiralState())  # Go into MoveInSpiral state
            
    def execute(self, agent):
        self.count += 1
        agent.set_velocity(FORWARD_SPEED, 0)


class MoveInSpiralState(State):
    def __init__(self):
        super().__init__("MoveInSpiral")
        self.count = 0
    
    def check_transition(self, agent, state_machine):
        time = self.count*SAMPLE_TIME
        if (time >= MOVE_IN_SPIRAL_TIME):
            state_machine.change_state(MoveForwardState())
        if agent.get_bumper_state():
            agent.set_bumper_state(False) # Set bumper state to false
            state_machine.change_state(GoBackState()) # Go into GoBack State

    def execute(self, agent):
        self.count += 1
        time = self.count*SAMPLE_TIME
        agent.set_velocity(FORWARD_SPEED, FORWARD_SPEED/(INITIAL_RADIUS_SPIRAL + SPIRAL_FACTOR*time))

class GoBackState(State):
    def __init__(self):
        super().__init__("GoBack")
        self.count = 0
        

    def check_transition(self, agent, state_machine):
        time = self.count*SAMPLE_TIME
        if (time >= GO_BACK_TIME):
            state_machine.change_state(RotateState())

    def execute(self, agent):
        self.count += 1
        agent.set_velocity(BACKWARD_SPEED, 0) # set velocity to backwards

class RotateState(State):
    def __init__(self):
        super().__init__("Rotate")
        self.count = 0


    def check_transition(self, agent, state_machine):
        rotate_time = random.uniform(math.pi/(2*ANGULAR_SPEED), 3*math.pi/(2*ANGULAR_SPEED))
        if rotate_time <= self.count*SAMPLE_TIME:
            state_machine.change_state(MoveForwardState())
    
    def execute(self, agent):
        self.count  += 1
        agent.set_velocity(0, ANGULAR_SPEED)
        
        
