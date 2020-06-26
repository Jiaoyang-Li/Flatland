#!/usr/bin/env python 

import collections
from typing import Optional, List, Dict, Tuple

import numpy as np

from flatland.core.env import Environment
from flatland.core.env_observation_builder import ObservationBuilder
from flatland.core.env_prediction_builder import PredictionBuilder
from flatland.envs.agent_utils import RailAgentStatus, EnvAgent


class CustomObservationBuilder(ObservationBuilder):
    """
    Template for building a custom observation builder for the RailEnv class

    The observation in this case composed of the following elements:

        - transition map array with dimensions (env.height, env.width),\
          where the value at X,Y will represent the 16 bits encoding of transition-map at that point.
        
        - the individual agent object (with position, direction, target information available)

    """
    def __init__(self):
        super(CustomObservationBuilder, self).__init__()

    def set_env(self, env: Environment):
        super().set_env(env)
        # Note :
        # The instantiations which depend on parameters of the Env object should be 
        # done here, as it is only here that the updated self.env instance is available
        self.rail_obs = np.zeros((self.env.height, self.env.width))

    def reset(self):
        """
        Called internally on every env.reset() call, 
        to reset any observation specific variables that are being used
        """
        self.rail_obs[:] = 0        
        for _x in range(self.env.width):
            for _y in range(self.env.height):
                # Get the transition map value at location _x, _y
                transition_value = self.env.rail.get_full_transitions(_y, _x)
                self.rail_obs[_y, _x] = transition_value

    def get(self, handle: int = 0):
        """
        Returns the built observation for a single agent with handle : handle

        In this particular case, we return 
        - the global transition_map of the RailEnv,
        - a tuple containing, the current agent's:
            - state
            - position
            - direction
            - initial_position
            - target
        """

        agent = self.env.agents[handle]
        """
        Available information for each agent object : 

        - agent.status : [RailAgentStatus.READY_TO_DEPART, RailAgentStatus.ACTIVE, RailAgentStatus.DONE]
        - agent.position : Current position of the agent
        - agent.direction : Current direction of the agent
        - agent.initial_position : Initial Position of the agent
        - agent.target : Target position of the agent
        """

        status = agent.status
        position = agent.position
        direction = agent.direction
        initial_position = agent.initial_position
        target = agent.target

        
        """
        You can also optionally access the states of the rest of the agents by 
        using something similar to 

        for i in range(len(self.env.agents)):
            other_agent: EnvAgent = self.env.agents[i]

            # ignore other agents not in the grid any more
            if other_agent.status == RailAgentStatus.DONE_REMOVED:
                continue

            ## Gather other agent specific params 
            other_agent_status = other_agent.status
            other_agent_position = other_agent.position
            other_agent_direction = other_agent.direction
            other_agent_initial_position = other_agent.initial_position
            other_agent_target = other_agent.target

            ## Do something nice here if you wish
        """
        return self.rail_obs, (status, position, direction, initial_position, target)

