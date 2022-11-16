from re import X
from turtle import position
import numpy as np
import random
from math import inf


class Particle:
    """
    Represents a particle of the Particle Swarm Optimization algorithm.
    """
    def __init__(self, lower_bound, upper_bound):
        """
        Creates a particle of the Particle Swarm Optimization algorithm.

        :param lower_bound: lower bound of the particle position.
        :type lower_bound: numpy array.
        :param upper_bound: upper bound of the particle position.
        :type upper_bound: numpy array.
        """
        self.x = np.random.uniform(lower_bound, upper_bound)
        self.v = random.uniform(lower_bound-upper_bound, upper_bound-lower_bound)
        self.best = -inf
        self.best_pos = self.x

class ParticleSwarmOptimization:
    """
    Represents the Particle Swarm Optimization algorithm.
    Hyperparameters:
        inertia_weight: inertia weight.
        cognitive_parameter: cognitive parameter.
        social_parameter: social parameter.

    :param hyperparams: hyperparameters used by Particle Swarm Optimization.
    :type hyperparams: Params.
    :param lower_bound: lower bound of particle position.
    :type lower_bound: numpy array.
    :param upper_bound: upper bound of particle position.
    :type upper_bound: numpy array.
    """
    def __init__(self, hyperparams, lower_bound, upper_bound):
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.omega = hyperparams.inertia_weight
        self.rho_p = hyperparams.cognitive_parameter
        self.rho_g = hyperparams.social_parameter
        self.num_particles = hyperparams.num_particles
        self.particles = []
        self.best_pos = None
        self.best_value = -np.Inf
        self.count = 0

        for i in range(0, self.num_particles + 1):
            self.particles.append(Particle(lower_bound, upper_bound))
         
    def get_best_position(self):
        """
        Obtains the best position so far found by the algorithm.

        :return: the best position.
        :rtype: numpy array.
        """
        return self.best_pos

    def get_best_value(self):
        """
        Obtains the value of the best position so far found by the algorithm.

        :return: value of the best position.
        :rtype: float.
        """
    
        return self.best_value 

    def get_position_to_evaluate(self):
        """
        Obtains a new position to evaluate.

        :return: position to evaluate.
        :rtype: numpy array.
        """
        if self.count < self.num_particles:
            self.count += 1
        else:
            self.count = 0
            self.advance_generation()
        return self.particles[self.count].x 

    def advance_generation(self):
        """
        Advances the generation of particles. Auxiliary method to be used by notify_evaluation().
        """

        rp = random.uniform(0.0, 1.0)
        rg = random.uniform(0.0, 1.0)
        for particle in self.particles:
            particle.v = self.omega*particle.v + rp*self.rho_p*(particle.best_pos - particle.x) + rg*self.rho_g*(self.best_pos - particle.x)
            particle.x = particle.x + particle.v
            
            size = len(particle.x)
            for i in range(size):
                particle.v[i] = min(max(particle.v[i], -self.upper_bound[i]+self.lower_bound[i]), self.upper_bound[i]-self.lower_bound[i])
                particle.x[i] = min(max(particle.x[i], self.lower_bound[i]), self.upper_bound[i])
        
    def notify_evaluation(self, value):
        """
        Notifies the algorithm that a particle position evaluation was completed.

        :param value: quality of the particle position.
        :type value: float.
        """

        if value > self.particles[self.count].best:
            self.particles[self.count].best = value
            self.particles[self.count].best_pos = self.particles[self.count].x
        if value > self.best_value:
            self.best_value = value
            self.best_pos = self.particles[self.count].x
        
        
       

