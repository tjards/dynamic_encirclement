#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This project implements an autonomous, decentralized dynamic encirclement strategy for swarms of vehicles. 
The strategy requires no human invervention once the target is selected and all vehicles rely on local knowledge only. 
Each vehicle makes its own decisions about where to go based on its relative position to other vehicles, 
but the protocol results in a globally stable, evenly-spaced swarm. 

Adapted from the approach in:
    
    Ahmed T. Hafez; Anthony J. Marasco; Sidney N. Givigi; Mohamad Iskandarani; Shahram Yousefi; 
    and Camille Alain Rabbath, "Solving Multi-UAV Dynamic Encirclement via Model Predictive Control", 
    IEEE Transactions on Control Systems Technology, Vol. 23 (6), Nov 2015

but reformulated to be compatible with the Reynolds Rules canon

Created on Tue Dec 22 11:48:18 2020

@author: tjards

"""

#%% Import stuff
# --------------

#from scipy.integrate import ode
import numpy as np
import animation 
import dynamics_node as node
import tools as tools
import encirclement_tools as encircle_tools
import ctrl_tactic as tactic 
import pickle 
import quaternions as quat
import random 

#%% Setup Simulation
# ------------------
Ti = 0         # initial time
Tf = 40         # final time 
Ts = 0.02      # sample time
nVeh = 15       # number of vehicles
iSpread = 40   # initial spread of vehicles

tactic_type = 2     # [2 = circle]

# Vehicles states
# ---------------
state = np.zeros((6,nVeh))
state[0,:] = iSpread*(np.random.rand(1,nVeh)-0.5)                   # position (x)
state[1,:] = iSpread*(np.random.rand(1,nVeh)-0.5)                   # position (y)
state[2,:] = np.maximum((iSpread*np.random.rand(1,nVeh)-0.5),2)+20  # position (z)
state[3,:] = 0                                                  # velocity (vx)
state[4,:] = 0                                                  # velocity (vy)
state[5,:] = 0                                                  # velocity (vz)
centroid = encircle_tools.centroid(state[0:3,:].transpose())

# Commands
# --------
cmd = np.zeros((3,nVeh))
cmd[0] = np.random.rand(1,nVeh)-0.5      # command (x)
cmd[1] = np.random.rand(1,nVeh)-0.5      # command (y)
cmd[2] = np.random.rand(1,nVeh)-0.5      # command (z)

# Targets
# -------
targets = 4*(np.random.rand(6,nVeh)-0.5)
targets[0,:] = -13 #5*(np.random.rand(1,nVeh)-0.5)
targets[1,:] = -15 #5*(np.random.rand(1,nVeh)-0.5)
targets[2,:] = 14
targets[3,:] = 0
targets[4,:] = 0
targets[5,:] = 0
targets_encircle = targets.copy()
error = state[0:3,:] - targets[0:3,:]

#%% Define obstacles
# ------------------
nObs = 1    # number of obstacles
obstacles = np.zeros((4,nObs))
oSpread = iSpread*2

# manual (comment out if random)
# obstacles[0,:] = 0    # position (x)
# obstacles[1,:] = 0    # position (y)
# obstacles[2,:] = 0    # position (z)
# obstacles[3,:] = 0

# #random (comment this out if manual)
# obstacles[0,:] = oSpread*(np.random.rand(1,nObs)-0.5)                   # position (x)
# obstacles[1,:] = oSpread*(np.random.rand(1,nObs)-0.5)                   # position (y)
# obstacles[2,:] = np.maximum(oSpread*(np.random.rand(1,nObs)-0.5),2)     # position (z)
# obstacles[3,:] = np.random.rand(1,nObs)+0.5                             # radii of obstacle(s)

# manual - make the target an obstacle
obstacles[0,0] = targets[0,0]     # position (x)
obstacles[1,0] = targets[1,0]     # position (y)
obstacles[2,0] = targets[2,0]     # position (z)
obstacles[3,0] = 1                # radii of obstacle(s)

# Walls/Floors 
# - these are defined manually as planes
# --------------------------------------   
nWalls = 1
walls = np.zeros((6,nWalls)) 
walls_plots = np.zeros((4,nWalls))

# add the ground at z = 0:
newWall0, newWall_plots0 = tools.buildWall('horizontal', -2) 

# load the ground into constraints   
walls[:,0] = newWall0[:,0]
walls_plots[:,0] = newWall_plots0[:,0]

# add other planes (comment out by default)

# newWall1, newWall_plots1 = flock_tools.buildWall('diagonal1a', 3) 
# newWall2, newWall_plots2 = flock_tools.buildWall('diagonal1b', -3) 
# newWall3, newWall_plots3 = flock_tools.buildWall('diagonal2a', -3) 
# newWall4, newWall_plots4 = flock_tools.buildWall('diagonal2b', 3)

# load other planes (comment out by default)

# walls[:,1] = newWall1[:,0]
# walls_plots[:,1] = newWall_plots1[:,0]
# walls[:,2] = newWall2[:,0]
# walls_plots[:,2] = newWall_plots2[:,0]
# walls[:,3] = newWall3[:,0]
# walls_plots[:,3] = newWall_plots3[:,0]
# walls[:,4] = newWall4[:,0]
# walls_plots[:,4] = newWall_plots4[:,0]

#%% Run Simulation
# ----------------------
t = Ti
i = 1
f = 0         # parameter for future use

nSteps = int(Tf/Ts+1)
t_all          = np.zeros(nSteps)
states_all     = np.zeros([nSteps, len(state), nVeh])
cmds_all       = np.zeros([nSteps, len(cmd), nVeh])
targets_all    = np.zeros([nSteps, len(targets), nVeh])
obstacles_all  = np.zeros([nSteps, len(obstacles), nObs])
centroid_all   = np.zeros([nSteps, len(centroid), 1])
f_all          = np.ones(nSteps)

t_all[0]                = Ti
states_all[0,:,:]       = state
cmds_all[0,:,:]         = cmd
targets_all[0,:,:]      = targets
obstacles_all[0,:,:]    = obstacles
centroid_all[0,:,:]     = centroid
f_all[0]                = f

while round(t,3) < Tf:
  
    # Evolve the target
    # -----------------
    tSpeed = 1
    targets[0,:] = targets[0,:] + tSpeed*0.002
    targets[1,:] = targets[1,:] + tSpeed*0.005
    targets[2,:] = targets[2,:] + tSpeed*0.0005
    
    # Update the obstacle
    # manual - make the target an obstacle
    obstacles[0,0] = targets[0,0]     # position (x)
    obstacles[1,0] = targets[1,0]     # position (y)
    obstacles[2,0] = targets[2,0]     # position (z)

    # Evolve the states
    # -----------------
    state = node.evolve(Ts, state, cmd)
    
    # Store results
    # -------------
    t_all[i]                = t
    states_all[i,:,:]       = state
    cmds_all[i,:,:]         = cmd
    targets_all[i,:,:]      = targets
    obstacles_all[i,:,:]    = obstacles
    centroid_all[i,:,:]     = centroid
    f_all[i]                = f
    
    # Increment 
    # ---------
    t += Ts
    i += 1
        
    # Compute encirclement trajectory (next step)
    # ------------------------------------------
    
    # parameters 
    r_desired = 10              # desired radius of encirclement [m]
    ref_plane = 'horizontal'    # define reference plane (default horizontal)    
  
    # update encirclement trajectories
    centroid = encircle_tools.centroid(state[0:3,:].transpose())
    swarm_prox = tactic.sigma_norm(centroid.ravel()-targets[0:3,0])
    if tactic_type == 2:    
        # define the orientation of the plan (quaterions)
        if t < 20:
            quat_0 = quat.e2q(np.array([np.pi/2,0,0]))
        if t >= 30:
            quat_0 = quat.e2q(np.array([0,0,0]))
        if t >= 40:
            quat_0 = quat.e2q(np.array([0,-np.pi/3,0]))
               
        targets_encircle = encircle_tools.encircle_target(targets, state, r_desired, ref_plane, quat_0)
        
    # Prep to compute commands (next step)
    # ----------------------------
    states_q = state[0:3,:]     # positions
    states_p = state[3:6,:]     # velocities 
    d_prime = 1 #0.6*d          # distance between a- and b-agents
    r_prime = 2*d_prime         # interaction range of a- and b-agents
    
    # Add other vehicles as obstacles (optional, default = 0)
    # -------------------------------------------------------
    vehObs = 0     # include other vehicles as obstacles [0 = no, 1 = yes]  
    if vehObs == 0: 
        obstacles_plus = obstacles
    elif vehObs == 1:
        states_plus = np.vstack((state[0:3,:], d_prime*np.ones((1,state.shape[1])))) 
        obstacles_plus = np.hstack((obstacles, states_plus))
            
    # Compute the commads (next step)
    # --------------------------------       
    cmd = tactic.commands(states_q, states_p, obstacles_plus, walls, r_prime, d_prime, targets[0:3,:], targets[3:6,:], targets_encircle[0:3,:], targets_encircle[3:6,:], swarm_prox)
       
    
#%% Produce animation of simulation
# ---------------------------------
showObs = 1 # (0 = don't show obstacles, 1 = show obstacles, 2 = show obstacles + floors/walls)
ani = animation.animateMe(Ts, t_all, states_all, cmds_all, targets_all[:,0:3,:], obstacles_all, r_prime, d_prime, walls_plots, showObs, centroid_all, f_all, r_desired, tactic_type)
#plt.show()    

#%% Save stuff

pickle_out = open("Data/t_all.pickle","wb")
pickle.dump(t_all, pickle_out)
pickle_out.close()
pickle_out = open("Data/cmds_all.pickle","wb")
pickle.dump(cmds_all, pickle_out)
pickle_out.close()
pickle_out = open("Data/states_all.pickle","wb")
pickle.dump(states_all, pickle_out)
pickle_out.close()
pickle_out = open("Data/targets_all.pickle","wb")
pickle.dump(targets_all, pickle_out)
pickle_out.close()
pickle_out = open("Data/obstacles_all.pickle","wb")
pickle.dump(obstacles_all, pickle_out)
pickle_out.close()
pickle_out = open("Data/centroid_all.pickle","wb")
pickle.dump(centroid_all, pickle_out)
pickle_out.close()

#%% Old stuff
    #targets_encircle = encircle_tools.encircle_target(targets, state, r_desired, enc_plane)


