#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan  1 21:26:07 2021

some useful tools for implement dynamic encirclement 

@author: tjards
"""

import numpy as np
import quaternions as quat

# delta_phi_desired = 2Pi/N

def polar2cart(r, theta):
    #note: accepts and return radians
    x = r*np.cos(theta)
    y = r*np.sin(theta)
    return x, y

def cart2polar(x, y):
    #note: return radians
    # [-pi, pi]
    #r = np.linalg.norm(x,y)
    r = np.sqrt(x**2 + y**2)
    theta = np.arctan2(y,x)
    #convert to 0 to 2Pi
    theta = np.mod(theta, 2*np.pi) 
    
    return r, theta 

# def phi_dot_ik(xi,yi,xk,yk,xi_dot,yi_dot,xk_dot,yk_dot):
#     x_ik = xi-xk
#     y_ik = yi-yk
#     x_ik_dot = xi_dot-xk_dot
#     y_ik_dot = yi_dot-yk_dot
#     phi_dot_ik = np.divide(x_ik*y_ik_dot - x_ik_dot*y_ik, (x_ik**2 + y_ik**2))
#    return phi_dot_ik
    
def phi_dot_i_desired(phi_i, phi_j, phi_k, phi_dot_desired):
    gamma = 0.5 # tunable
    phi_ki = np.mod(phi_i - phi_k, 2*np.pi) # make sure between 0 and 2pi
    phi_ij = np.mod(phi_j - phi_i, 2*np.pi) # make sure between 0 and 2pi
    phi_dot_i_desired = np.divide(3*phi_dot_desired + gamma*(phi_ki-phi_ij),3)
    return phi_dot_i_desired
    
def directToCircle(A,B,r):
    # A = target center
    # B = vechicle position 
    # C = closest point between 
    C = A+r*np.divide(B-A,np.linalg.norm(B-A))
    return C

def centroid(points):
    length = points.shape[0]
    sum_x = np.sum(points[:, 0])
    sum_y = np.sum(points[:, 1])
    sum_z = np.sum(points[:, 2])
    centroid = np.array((sum_x/length, sum_y/length, sum_z/length), ndmin = 2)
    return centroid.transpose() 


#%% Encirclement calculations
# ---------------------------
    
def encircle_target(targets, state, r_desired, enc_plane, quatern):
        
    # desired rate of encirclement [rad/s]
    # -----------------------------------
    phi_dot_desired = 0.15                  
    
    # initialise global stuff
    # -----------------------
    targets_encircle = targets.copy() 
    points_i = np.zeros((3,state.shape[1]))
    temp = np.zeros((3,1))
    quatern_ = quat.quatjugate(quatern)
    
    # Regulation of Radius (position control)
    # ------------------------------   
    new_pos_desired_i = np.zeros((3,state.shape[1]))
    
    # iterate through each agent
    for ii in range(0,state.shape[1]):
   
        # to rotate with reference to horizontal
        if enc_plane == 'horizontal':
            # rotate down to the reference plane
            points_i[:,ii] = quat.rotate(quatern_,state[0:3,ii]-targets[0:3,ii])+targets[0:3,ii]
            # now find the desired position projected on the plane
            temp[0:2,0] = directToCircle(targets[0:2,ii],points_i[0:2,ii],r_desired)
            temp[2,0] = targets[2,ii] # at altitude
            # now rotate back
            new_pos_desired_i[:,ii] = quat.rotate(quatern,temp.ravel()-targets[0:3,ii])+targets[0:3,ii]            
        
    # Regulation of Angular speed (velocity control)
    # ----------------------------------------------   
    # express state with reference to target
    state_shifted = state - targets
        
    # to rotate with reference to horizontal
    if enc_plane == 'horizontal':
        # define a new unit vector, which is perp to plane 
        unit_v = np.array([0,0,1]).reshape((3,1))
        # initialize a new state vector
        state_shifted_new = np.zeros((3,state.shape[1]))
        # rotate each agent into the reference plane
        for ij in range(0,state.shape[1]):
            state_shifted_new[:,ij] = quat.rotate(quatern_,state_shifted[0:3,ij])
 
        # convert to polar coordinates
        polar_r, polar_phi = cart2polar(state_shifted_new[0,:], state_shifted_new[1,:])
  
    # sort by phi and save the indicies so we can reassemble
    polar_phi_sorted = np.sort(polar_phi, axis=0)
    polar_phi_argsort = np.argsort(polar_phi, axis=0) 
    
    # for each vehicle, define a desired angular speed 
    phi_dot_desired_i = np.zeros((1,state_shifted.shape[1]))
    #phi_desired_i = np.zeros((1,state_shifted.shape[1])) # for desired separation
    #separation_desired = 2*np.pi/state_shifted.shape[1]   
    
    # identify leading and lagging 
    for ii in range(0,state_shifted.shape[1]):
        # define leading and lagging vehicles (based on angles)
        if ii == 0:
            ij = state_shifted.shape[1]-1    
        else:
            ij = ii-1 # lagging vehicle
        
        if ii == state_shifted.shape[1]-1:
            ik = 0
        else:
            ik = ii+1 # leading vehicle 
        
        # compute the desired phi-dot       
        phi_dot_desired_i[0,ii] = phi_dot_i_desired(polar_phi_sorted[ii], polar_phi_sorted[ij], polar_phi_sorted[ik], phi_dot_desired)
        
    # convert the angular speeds back to cartesian (in the correct order)
    # ----------------------------------------------
    xy_dot_desired_i = np.zeros((3,state.shape[1]))
    
    index_proper = 0
    for ii in polar_phi_argsort:
        
        # get angular speed 
        w_vector = quat.rotate(quatern,phi_dot_desired_i[0,index_proper]*unit_v)
        # find the corresponding velo vector
        v_vector = np.cross(w_vector.ravel(),state_shifted[0:3,ii])
        # break out into components
        xy_dot_desired_i[0,ii] = v_vector[0] 
        xy_dot_desired_i[1,ii] = v_vector[1] 
        xy_dot_desired_i[2,ii] = v_vector[2] 
        index_proper += 1
 
    # define new targets for encirclement
    # ----------------------------------
    # if we're rotating wrt horizontal 
    if enc_plane == 'horizontal':
        targets_encircle[0:3,:] = new_pos_desired_i[:,:]
        targets_encircle[3:6,:] = -xy_dot_desired_i[:,:] 

    return targets_encircle




