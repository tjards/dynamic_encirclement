#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 22 20:23:23 2020

@author: tjards
"""
import numpy as np

def evolve(Ts, state, cmd):
    
    # constraints
    vmax = 250
    
    #discretized doubple integrator 
    state[0:3,:] = state[0:3,:] + state[3:6,:]*Ts
    #state[3:6,:] = state[3:6,:] + cmd[:,:]*Ts
    state[3:6,:] = np.minimum(np.maximum(state[3:6,:] + cmd[:,:]*Ts, -vmax), vmax)
    
    return state








# %% Old

# # Define dynamics
# # ---------------
# def state_dot(t, state, cmd):
    
#     dynDot = np.array([
#         [state[3]],
#         [state[4]],
#         [state[5]],
#         [cmd[0]],
#         [cmd[1]],
#         [cmd[2]]])
    
#     dstate = np.zeros(6)
#     dstate[0] = dynDot[0]
#     dstate[1] = dynDot[1]
#     dstate[2] = dynDot[2]
#     dstate[3] = dynDot[3]
#     dstate[4] = dynDot[4]
#     dstate[5] = dynDot[5]
    
#     return dstate

# # Set integrator
# # -------------
# integrator = ode(state_dot).set_integrator('dopri5', first_step='0.00005', atol='10e-6', rtol='10e-6')
# integrator.set_initial_value(state, Ti)