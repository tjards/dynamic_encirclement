#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 22 13:55:02 2020

This file plots stuff in 3D

@author: tjards
"""

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
import numpy as np
plt.rcParams['animation.ffmpeg_path'] = '/usr/local/bin/ffmpeg' #my add - this path needs to be added
Writer = animation.writers['ffmpeg']
writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)

numFrames = 10 # frame rate (bigger = slower)
tail = 300
zoom = 1    # do you want to adjust frames with motion? [0 = no, 1 = yes]



def animateMe(Ts, t_all, states_all, cmds_all, targets_all, obstacles_all, r, d, walls_plots, showObs, centroid_all, f, r_desired, tactic_type):
    
    # pull out positions
    # ------------------
    nVeh = states_all.shape[2]
    nObs = obstacles_all.shape[2]
    r_copy = r
    
    # intermediate variables
    # ----------------------
    x = states_all[:,0,:]
    y = states_all[:,1,:]
    z = states_all[:,2,:]
    x_from0 = x
    y_from0 = y
    z_from0 = z
    x_v = states_all[:,3,:]
    y_v = states_all[:,4,:]
    z_v = states_all[:,5,:]
    head = 0.2
    x_head = states_all[:,0,:] + head*x_v
    y_head = states_all[:,1,:] + head*x_v
    z_head = states_all[:,2,:] + head*x_v
    x_t = targets_all[:,0,:]
    y_t = targets_all[:,1,:]
    z_t = targets_all[:,2,:]
    x_o = obstacles_all[:,0,:]
    y_o = obstacles_all[:,1,:]
    z_o = obstacles_all[:,2,:]
    r_o = obstacles_all[:,3,:]
    cx = centroid_all[:,0,:]
    cy = centroid_all[:,1,:]
    cz = centroid_all[:,2,:]
    cd = round(np.linalg.norm(centroid_all[0,:,0].ravel()-targets_all[0,0:3,0]),1)
            
    # initialize plot
    # ---------------
    fig = plt.figure()
    ax = p3.Axes3D(fig)
    
    # axis properties
    # ---------------
    margins = 0.5
    maxRange = 0.5*np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() + margins
    mid_x = 0.5*(x.max()+x.min())
    mid_y = 0.5*(y.max()+y.min())
    mid_z = 0.5*(z.max()+z.min())
    ax.set_xlim3d([mid_x-maxRange, mid_x+maxRange])
    ax.set_xlabel('x-direction')
    ax.set_ylim3d([mid_y-maxRange, mid_y+maxRange])
    ax.set_ylabel('y-direction')
    ax.set_zlim3d([mid_z-maxRange, mid_z+maxRange])
    ax.set_zlabel('Altitude')
    
    # labels
    # ------
    titleTime = ax.text2D(0.05, 0.95, "", transform=ax.transAxes)
    #titleType1 = ax.text2D(0.95, 0.95, '%s : %s' % ("Lattice separation", d), transform=ax.transAxes, horizontalalignment='right')
    titleType1 = ax.text2D(0.95, 0.95, "Mode: Dynamic Encirclement", transform=ax.transAxes, horizontalalignment='right')
    #titleType2 = ax.text2D(0.95, 0.91, 'Title2', transform=ax.transAxes, horizontalalignment='right')
    titleType2 = ax.text2D(0.95, 0.91, '%s : %s' % ("Centroid distance", cd), transform=ax.transAxes, horizontalalignment='right')
    titleType3 = ax.text2D(0.95, 0.87, '%s : %s' % ("Encircle radius", r_desired), transform=ax.transAxes, horizontalalignment='right')
    
    
    # plot things that never move (targets, for now)
    #ax.scatter(targets[0,:], targets[1,:], targets[2,:], color='red', alpha=1, marker = 'o', s = 25)
    
    # initialize lines
    # -----------------
    lines_dots = []
    lines_tails = []
    lines_heads = []
    lines_targets = []
    lines_obstacles = []
    #lattice = ax.plot([], [], [], '-', lw=1, color='cyan')
    lattices = []
    centroids = ax.plot([], [], [], 'kx')
    centroids_line = ax.plot([], [], [], '--', lw=1, color='black')
    
    
    # draw planes (stationary)
    # -----------------------    
    if showObs == 2:
        for i in range(0, walls_plots.shape[1]):
            xx, yy = np.meshgrid(np.linspace(mid_x-maxRange, mid_x+maxRange, 20), np.linspace(mid_y-maxRange, mid_y+maxRange, 20))
            if walls_plots[2,i] == 0:
                walls_plots[2,i] = 0.001 # avoid divide by zero           
            zz = (-walls_plots[0,i] * xx - walls_plots[1,i] * yy + walls_plots[3,i] * 1.) / walls_plots[2,i]
            ax.plot_wireframe(xx, yy, zz, color='m', rcount=20, ccount=20)
  
    # initialize moving stuff
    # -----------------------
    for i in range (0, nVeh):
        
        line_dot = ax.plot([], [], [], 'bo')
        lines_dots.extend(line_dot)
        line_tail = ax.plot([], [], [], ':', lw=1, color='blue')
        lines_tails.extend(line_tail)
        line_head = ax.plot([], [], [], '-', lw=1, color='black')
        lines_heads.extend(line_head)
        line_target = ax.plot([], [], [], 'go')
        lines_targets.extend(line_target)       
        lattice = ax.plot([], [], [], ':', lw=1, color=[0.5,0.5,0.5])
        lattices.extend(lattice)

    # initialize obstacles (if config'd)
    # ---------------------------------
    if showObs >= 1:
        for j in range (0, nObs):    
            line_obstacle = ax.plot([], [], [], 'ro', ms = 10*r_o[0,j] )
            lines_obstacles.extend(line_obstacle)
    
    # update the lines
    # ----------------
    def update(i):
             
        time = t_all[i*numFrames]
        x = states_all[i*numFrames,0,:]
        y = states_all[i*numFrames,1,:]
        z = states_all[i*numFrames,2,:]
        #x_from0 = states_all[0:i*numFrames,0]
        #y_from0 = states_all[0:i*numFrames,1]
        #z_from0 = states_all[0:i*numFrames,2]
        x_from0 = states_all[i*numFrames-tail:i*numFrames,0,:]
        y_from0 = states_all[i*numFrames-tail:i*numFrames,1,:]
        z_from0 = states_all[i*numFrames-tail:i*numFrames,2,:]
        x_v = states_all[i*numFrames,3,:]
        y_v = states_all[i*numFrames,4,:]
        z_v = states_all[i*numFrames,5,:]
        #norma = np.maximum(np.linalg.norm([x+x_v,y+y_v,z+z_v]),0.001)
        norma = np.maximum(np.sqrt(x_v**2 + y_v**2 + z_v**2),0.0001)
        x_head = x + head*x_v/norma
        y_head = y + head*y_v/norma
        z_head = z + head*z_v/norma
        x_point = np.vstack((x,x_head))
        y_point = np.vstack((y,y_head))
        z_point = np.vstack((z,z_head))
        x_t = targets_all[i*numFrames,0,:]
        y_t = targets_all[i*numFrames,1,:]
        z_t = targets_all[i*numFrames,2,:]
        x_o = obstacles_all[i*numFrames,0,:]
        y_o = obstacles_all[i*numFrames,1,:]
        z_o = obstacles_all[i*numFrames,2,:]
        r_o = obstacles_all[i*numFrames,3,:]        
        pos = states_all[i*numFrames,0:3,:]
        x_lat = np.zeros((nVeh,nVeh))
        y_lat = np.zeros((nVeh,nVeh))
        z_lat = np.zeros((nVeh,nVeh))
        cx = centroid_all[i*numFrames,0,:]
        cy = centroid_all[i*numFrames,1,:]
        cz = centroid_all[i*numFrames,2,:]
        cd = round(np.linalg.norm(centroid_all[i*numFrames,:,0].ravel()-targets_all[i*numFrames,0:3,0]),1)
                   
        
        # reset axis limits
        # -----------------       
        if zoom == 1:
            margins = 0.5
            maxRange = 0.5*np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() + margins
            mid_x = 0.5*(x.max()+x.min())
            mid_y = 0.5*(y.max()+y.min())
            mid_z = 0.5*(z.max()+z.min())
            ax.set_xlim3d([mid_x-maxRange, mid_x+maxRange])
            ax.set_ylim3d([mid_y-maxRange, mid_y+maxRange])
            ax.set_zlim3d([mid_z-maxRange, mid_z+maxRange])

        
        
        # build lattice
        # -------------
        
        if f[i*numFrames] < 1:
            r_ = 0*r_copy
        else: 
            r_ = r_copy        # just to help visualize vehicle interactions
        
        for j in range (0, nVeh):
        
            temp_lat = lattices[j]    
        
            # search through each neighbour
            for k_neigh in range(pos.shape[1]):
                # except for itself (duh):
                if j != k_neigh:
                    # compute the euc distance between them
                    dist = np.linalg.norm(pos[:,j]-pos[:,k_neigh])
                    # if it is within the interaction range
                    if dist <= r_: 
                        x_lat[k_neigh,j] = pos[0,k_neigh]
                        y_lat[k_neigh,j] = pos[1,k_neigh]
                        z_lat[k_neigh,j] = pos[2,k_neigh]
                    else:
                        x_lat[k_neigh,j] = pos[0,j]
                        y_lat[k_neigh,j] = pos[1,j]
                        z_lat[k_neigh,j] = pos[2,j]
                else:
                    x_lat[k_neigh,j] = pos[0,j]
                    y_lat[k_neigh,j] = pos[1,j]
                    z_lat[k_neigh,j] = pos[2,j]                     
            
            temp_lat.set_data(x_lat[:,j], y_lat[:,j])
            temp_lat.set_3d_properties(z_lat[:,j])   
  
        # plot states... etc
        # ------------------
        for j in range (0, nVeh):
            
            # create a temporary holder
            # -------------------------
            temp1 = lines_dots[j]
            temp2 = lines_tails[j]
            temp3 = lines_heads[j]
            temp4 = lines_targets[j] 
            #temp_lat = lattices[j]
            temp1.set_data(x[j], y[j])
            temp1.set_3d_properties(z[j])
    
            # set variables
            # -------------
            temp2.set_data(x_from0[:,j], y_from0[:,j])
            temp2.set_3d_properties(z_from0[:,j])
            #temp3.set_data(x_point[:,j],y_point[:,j])
            #temp3.set_3d_properties(z_point[:,j])
            temp4.set_data(x_t[j], y_t[j])
            temp4.set_3d_properties(z_t[j]) 
            temp3.set_data(x_point[:,j],y_point[:,j])
            temp3.set_3d_properties(z_point[:,j])

            # set colors
            if tactic_type >= 3:
                #print(f[i*numFrames-1])
                if f[i*numFrames] < 0.5:    # if f dros below 0.5, we've transitioned to new tactic
                    if (j % 2) == 0:        # even vehicles go tactic 1, odd vehicles go tactic 2 
                        temp1.set_color('b')
                        temp2.set_color('b')
                    else:
                        temp1.set_color('c')
                        temp2.set_color('c')
                else:
                    temp1.set_color('b')
                    temp2.set_color('b')
                 
        # build obstacles
        # ---------------
        if showObs >= 1:
            for k in range (0, nObs):
                
                temp5 = lines_obstacles[k]
                
                temp5.set_data(x_o[k], y_o[k])
                temp5.set_3d_properties(z_o[k])

        # set others
        # ----------
        #line2.set_data(x, y)
        #line2.set_3d_properties(z)
        #line3.set_data(x_from0, y_from0)
        #line3.set_3d_properties(z_from0)
        titleTime.set_text(u"Time = {:.2f} s".format(time))
        titleType2.set_text('%s : %s' % ("Centroid Distance", cd))
          
        centroids[0].set_data(cx,cy)
        centroids[0].set_3d_properties(cz)
        
        cx_line=np.vstack((cx,x_t[0])).ravel()
        cy_line=np.vstack((cy,y_t[0])).ravel()
        cz_line=np.vstack((cz,z_t[0])).ravel()

        centroids_line[0].set_data(cx_line,cy_line)
        centroids_line[0].set_3d_properties(cz_line)
        
        return lines_dots, lines_tails, titleTime, lines_targets, lines_obstacles, centroids
    
    # make a GIF
    # ----------
    line_ani = animation.FuncAnimation(fig, update, blit=False, frames=len(t_all[0:-2:numFrames]), interval=(Ts*1000*numFrames))
    line_ani.save('Figs/animation.gif', writer=writer)
    plt.show()
    return line_ani
    
    #ax.scatter(states_all[:,0], states_all[:,1], states_all[:,2], color='blue', alpha=1, marker = 'o', s = 25) 
    
    print('animated')