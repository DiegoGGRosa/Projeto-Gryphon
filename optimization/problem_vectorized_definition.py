#!/usr/bin/env python3
#script: 2D multiobjective optimization using pymoo problem (vectorized)
#by Diego Rosa - the Pontifical Catholic University of Rio de Janeiro - PUC-Rio

import numpy as np
from pymoo.core.problem import Problem
from pymoo.core.problem import ElementwiseProblem
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.operators.crossover.sbx import SBX
from pymoo.operators.mutation.pm import PM
from pymoo.operators.sampling.rnd import FloatRandomSampling
from pymoo.termination import get_termination
from pymoo.optimize import minimize
import matplotlib.pyplot as plt

# variables: 

theta_base_deg = 30  # pitch angle in degrees 
theta_base_rad = np.deg2rad(theta_base_deg)
phi_base_deg = 0 # roll angle in degrees
phi_base_rad = np.deg2rad(phi_base_deg)
psi_base_deg = 0 # yaw angle in degrees
psi_base_rad = np.deg2rad(psi_base_deg)

theta = theta_base_rad
phi = phi_base_rad
psi = psi_base_rad

ac_zero = 0 # initial acceleration 

mass = 19.55 # robot mass in Kg 
gravity = 9.81 # gravity acceleration
weight = mass*gravity 
m = mass
g = gravity

r = 0.1016 # wheel radius in meter
h_cm = 0.1016 # height of the robot center of mass 
l = 0.31 # lenght between robot axis in meter
gama = 0.1 # longitudinal stability factor
mu = 0.5 # isotropic friction coefficient

# torque in rear wheels (T_A -> X[0]), in frontal wheels (T_B -> X[1]), and acceleration (a -> X[2]) are optimized

beta_gama = np.arctan((0.5*l-gama)/(r+h_cm)) # if the center of mass is not in the middle, use max between l_front and l_rear

# some 2D ref equations: https://www.engineersedge.com/mechanics_machines/vehicle-driving-slope-forces.htm
# according to this website (find more info), the maximum slope that PUMA+ can climb is 26 degrees
# explain in the thesis the calculation made in this website - that does not consider wheel slip

# F_NA = 0.5*weight*np.cos(theta) # simplified equation (for testing)
# F_NB = 0.5*weight*np.cos(theta) # simplified equation (for testing)

F_NA = (0.5*weight*np.cos(theta)+m*ac_zero*(h_cm/l)+weight*np.sin(theta)*(h_cm/l)) # slope with acceleration
F_NB = (0.5*weight*np.cos(theta)+m*ac_zero*(h_cm/l)-weight*np.sin(theta)*(h_cm/l)) # slope with acceleration

#F_NA = (0.5*weight*np.cos(theta)+m*X[2]*(h_cm/l)+weight*np.sin(theta)*(h_cm/l)) # slope with acceleration
#F_NB = (0.5*weight*np.cos(theta)+m*X[2]*(h_cm/l)-weight*np.sin(theta)*(h_cm/l)) # slope with acceleration

print(f"F_NA: {F_NA}\n")
print(f"F_NB: {F_NB}\n")

'''class TorqueDistribution(Problem):'''
class TorqueDistribution(ElementwiseProblem):

    def __init__(self):
        super().__init__(n_var=3, n_obj=2, n_ieq_constr=3, xl=np.array([0,0,0]), xu=np.array([7.21,7.21,4.0]))

    def _evaluate(self, X, out, *args, **kwargs):
                
        #f1_original = abs(beta_gama-np.arctan(X[2]*np.cos(theta)/(g-X[2]*np.sin(theta))))
        f1 = abs(beta_gama-np.arctan(X[2]*np.cos(theta)/(g-X[2]*np.sin(theta))))
        
        # it means f1 = abs(beta_gama-beta)

        #f2_original = abs((X[0]/r)/(0.5*weight*np.cos(theta))-mu)+abs((X[1]/r)/(0.5*weight*np.cos(theta))-mu)
        f2 = abs((X[0]/r)/(0.5*weight*np.cos(theta))-mu)+abs((X[1]/r)/(0.5*weight*np.cos(theta))-mu)
        
        # it means f2 = abs((T_A/r)/F_NA-mu)+abs((T_B/r)/F_NB-mu)
        # it considers that the weight is equally distributed between the 4 wheels (four wheels always in contact with the ground)
        
        #g1_original = (X[0])*np.cos(theta)+(X[1])*np.cos(theta)-F_NA*np.sin(theta)-F_NB*np.sin(theta)-m*(X[2])*np.cos(theta)
        g1 = (X[0])*np.cos(theta)+(X[1])*np.cos(theta)-((0.5*weight*np.cos(theta)+m*X[2]*(h_cm/l)+weight*np.sin(theta)*(h_cm/l)))*np.sin(theta)-((-0.5*weight*np.cos(theta)+m*X[2]*(h_cm/l)+weight*np.sin(theta)*(h_cm/l)))*np.sin(theta)-m*(X[2])*np.cos(theta)
        
        #g2_original = X[0]*np.sin(theta)+X[1]*np.sin(theta)+F_NA*np.cos(theta)+F_NB*np.cos(theta)-weight
        g2 = X[0]*np.sin(theta)+X[1]*np.sin(theta)+((0.5*weight*np.cos(theta)+m*X[2]*(h_cm/l)+weight*np.sin(theta)*(h_cm/l)))*np.cos(theta)+((-0.5*weight*np.cos(theta)+m*X[2]*(h_cm/l)+weight*np.sin(theta)*(h_cm/l)))*np.cos(theta)-weight
        
        #g3_original = X[0]*0.5*l*np.sin(theta)-X[0]*r*np.sin(theta)*np.sin(theta)-X[1]*0.5*l*np.sin(theta)+X[1]*r+X[0]*r*np.cos(theta)*np.cos(theta)+X[0]*h_cm*np.cos(theta)+X[1]*h_cm*np.cos(theta)+F_NB*h_cm*np.sin(theta)+F_NA*h_cm*np.sin(theta)+F_NB*0.5*l*np.cos(theta)-F_NA*0.5*l*np.cos(theta)
        g3 = X[0]*0.5*l*np.sin(theta)-X[0]*r*np.sin(theta)*np.sin(theta)-X[1]*0.5*l*np.sin(theta)+X[1]*r+X[0]*r*np.cos(theta)*np.cos(theta)+X[0]*h_cm*np.cos(theta)+X[1]*h_cm*np.cos(theta)+((-0.5*weight*np.cos(theta)+m*X[2]*(h_cm/l)+weight*np.sin(theta)*(h_cm/l)))*h_cm*np.sin(theta)+((0.5*weight*np.cos(theta)+m*X[2]*(h_cm/l)+weight*np.sin(theta)*(h_cm/l)))*h_cm*np.sin(theta)+((-0.5*weight*np.cos(theta)+m*X[2]*(h_cm/l)+weight*np.sin(theta)*(h_cm/l)))*0.5*l*np.cos(theta)-((0.5*weight*np.cos(theta)+m*X[2]*(h_cm/l)+weight*np.sin(theta)*(h_cm/l)))*0.5*l*np.cos(theta)
                
        out["F"] = [f1, f2] # 2 objective functions
        out["G"] = [g1, g2, g3] # 3 equations, use n_ieq_constr, test n_eq_constr
        
problem = TorqueDistribution()

algorithm = NSGA2(
    pop_size=50,
    n_offsprings=10,
    sampling=FloatRandomSampling(),
    crossover=SBX(prob=0.8, eta=15),
    mutation=PM(eta=30),
    eliminate_duplicates=True
)

res = minimize(problem,
               algorithm,
               ('n_gen', 400),
               seed=1,
               save_history=True,
               verbose=True,
               return_least_infeasible=True)

X = res.X
F = res.F
G = res.G
CV = res.CV

print(f"X: {X}\n")
print(f"F: {F}\n")
print(f"G: {G}\n")
print(f"CV: {CV}\n")

plt.figure(figsize=(7, 5))
plt.scatter(F[:, 0], F[:, 1], s=30, facecolors='none', edgecolors='blue')
plt.title("Objective Space")
plt.show()

# plot results as in the problem_sample
# more in https://pymoo.org/getting_started/part_2.html & https://pymoo.org/interface/result.html




