#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

Compute the control inputs for dynamic encirclement tactic  

Created on Mon Jan  4 12:45:55 2021

@author: tjards

"""

import numpy as np

#%% Setup hyperparameters
# ================================
a = 0.5
b = 0.5
c = np.divide(np.abs(a-b),np.sqrt(4*a*b)) 
eps = 0.1
#eps = 0.5
h = 0.9
pi = 3.141592653589793
c1_b = 1                # obstacle avoidance
c2_b = 2*np.sqrt(1)
c1_d = 4                # encirclement 
c2_d = 2*np.sqrt(2)


# Some function that are used often
# ---------------------------------

def sigma_norm(z):    
    norm_sig = (1/eps)*(np.sqrt(1+eps*np.linalg.norm(z)**2)-1)
    return norm_sig
    
def n_ij(q_i, q_j):
    n_ij = np.divide(q_j-q_i,np.sqrt(1+eps*np.linalg.norm(q_j-q_i)**2))    
    return n_ij

def sigma_1(z):    
    sigma_1 = np.divide(z,np.sqrt(1+z**2))    
    return sigma_1

def rho_h(z):    
    if 0 <= z < h:
        rho_h = 1        
    elif h <= z < 1:
        rho_h = 0.5*(1+np.cos(pi*np.divide(z-h,1-h)))    
    else:
        rho_h = 0  
    return rho_h
 
def phi_a(q_i, q_j, r_a, d_a): 
    z = sigma_norm(q_j-q_i)        
    phi_a = rho_h(z/r_a) * phi(z-d_a)    
    return phi_a
    
def phi(z):    
    phi = 0.5*((a+b)*sigma_1(z+c)+(a-b))    
    return phi 
        
def a_ij(q_i, q_j, r_a):        
    a_ij = rho_h(sigma_norm(q_j-q_i)/r_a)
    return a_ij

def b_ik(q_i, q_ik, d_b):        
    b_ik = rho_h(sigma_norm(q_ik-q_i)/d_b)
    return b_ik

def phi_b(q_i, q_ik, d_b): 
    z = sigma_norm(q_ik-q_i)        
    phi_b = rho_h(z/d_b) * (sigma_1(z-d_b)-1)    
    return phi_b

    
# Tactic Command Equations 
# ------------------------
def commands(states_q, states_p, obstacles, walls, r_prime, d_prime, targets, targets_v, targets_enc, targets_v_enc, swarm_prox):   
    
    # initialize 
    r_b = sigma_norm(r_prime)                   # obstacle separation (sensor range)
    d_b = sigma_norm(d_prime)                   # obstacle separation (goal range)
    u_obs = np.zeros((3,states_q.shape[1]))     # obstacles 
    u_enc = np.zeros((3,states_q.shape[1]))     # encirclement 
    cmd_i = np.zeros((3,states_q.shape[1]))     # store the commands
    
    # for each vehicle/node in the network
    for k_node in range(states_q.shape[1]): 
                                          
        # Obstacle Avoidance term (phi_beta)
        # ---------------------------------   
        # search through each obstacle 
        for k_obstacle in range(obstacles.shape[1]):

            # compute norm between this node and this obstacle
            normo = np.linalg.norm(states_q[:,k_node]-obstacles[0:3,k_obstacle])
            
            # ignore if overlapping
            #if normo == 0:
            if normo < 0.2:
                continue 
            
            # compute mu
            mu = np.divide(obstacles[3, k_obstacle],normo)
            # compute bold_a_k (for the projection matrix)
            bold_a_k = np.divide(states_q[:,k_node]-obstacles[0:3,k_obstacle],normo)
            bold_a_k = np.array(bold_a_k, ndmin = 2)
            # compute projection matrix
            P = np.identity(states_p.shape[0]) - np.dot(bold_a_k,bold_a_k.transpose())
            # compute beta-agent position and velocity
            q_ik = mu*states_q[:,k_node]+(1-mu)*obstacles[0:3,k_obstacle]
            # compute distance to beta-agent
            dist_b = np.linalg.norm(q_ik-states_q[:,k_node])
            # if it is with the beta range
            if dist_b < r_prime:
                # compute the beta command
                p_ik = mu*np.dot(P,states_p[:,k_node])    
                u_obs[:,k_node] += c1_b*phi_b(states_q[:,k_node], q_ik, d_b)*n_ij(states_q[:,k_node], q_ik) + c2_b*b_ik(states_q[:,k_node], q_ik, d_b)*(p_ik - states_p[:,k_node])
               
        # search through each wall (a planar obstacle)
        for k_wall in range(walls.shape[1]):
            
            # define the wall
            bold_a_k = np.array(np.divide(walls[0:3,k_wall],np.linalg.norm(walls[0:3,k_wall])), ndmin=2).transpose()    # normal vector
            y_k = walls[3:6,k_wall]         # point on plane
            # compute the projection matrix
            P = np.identity(y_k.shape[0]) - np.dot(bold_a_k,bold_a_k.transpose())
            # compute the beta_agent 
            q_ik = np.dot(P,states_q[:,k_node]) + np.dot((np.identity(y_k.shape[0])-P),y_k)
            # compute distance to beta-agent
            dist_b = np.linalg.norm(q_ik-states_q[:,k_node])
            # if it is with the beta range
            maxAlt = 10 # TRAVIS: maxAlt is for testing, only enforces walls below this altitude
            if dist_b < r_prime and states_q[2,k_node] < maxAlt:
                p_ik = np.dot(P,states_p[:,k_node])
                u_obs[:,k_node] += c1_b*phi_b(states_q[:,k_node], q_ik, d_b)*n_ij(states_q[:,k_node], q_ik) + c2_b*b_ik(states_q[:,k_node], q_ik, d_b)*(p_ik - states_p[:,k_node])
          
        # Encirclement term (phi_delta)
        # ----------------------------    
        u_enc[:,k_node] = - c1_d*sigma_1(states_q[:,k_node]-targets_enc[:,k_node])-c2_d*(states_p[:,k_node] - targets_v_enc[:,k_node])    
        
        #safety (if venture too far, just navigate)
        # if prox_i > transition*10:
        #     cmd_i[:,k_node] = - c1_g*sigma_1(states_q[:,k_node]-targets[:,k_node])
        #     print('! Safety engaged on agent:',k_node)
        #     print('-- Agent prox: ',prox_i, 'f: ', f_i)
        #     print('-- Swarm prox: ', swarm_prox)
        #     continue
    
        # Modulate the signals for local bimodal control
        # ----------------------------------------------
        cmd_i[:,k_node] = u_obs[:,k_node] + u_enc[:,k_node] 
        
    cmd = cmd_i    
    
    return cmd