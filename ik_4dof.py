#!/usr/bin/env python
import rospy
import tf
import numpy as np

l1 = 0.079
l2 = 0.0935
l3 = 0.0935
l4 = 0.112



def ik_solver(x, y, z, degree):
    ik_candidate = []
    ik_solution = []
    angle = np.radians(degree)
    theta_1 = np.arctan2(y, x)
    xq = x - l4 * np.cos(theta_1) * np.cos(angle)
    yq = y - l4 * np.sin(theta_1) * np.cos(angle)
    zq = z - l4 * np.sin(angle) - l1

    A = (xq)**2 + yq**2 + (zq)**2 

    # print "xq, zq, A: ",xq, zq, A
    theta_3_1 = np.pi - np.arccos( (l2**2 + l3**2 - A) / (2*l2*l3) )
    theta_3_2 = - theta_3_1
    # print theta_3_1

    if not np.isnan(theta_3_1):
        find_ik_candidate(xq, yq, zq, theta_1, theta_3_1, angle, z, ik_candidate)

    if not np.isnan(theta_3_2):
        find_ik_candidate(xq, yq, zq, theta_1, theta_3_2, angle, z, ik_candidate)

    for theta_1, theta_2, theta_3, theta_4 in ik_candidate:
        # print fk_solver(theta_1, np.pi/2 - theta_2, -theta_3, -theta_4)
        truth = np.round([x,y,z],6) == fk_solver(theta_1, np.pi/2 - theta_2, -theta_3, -theta_4)
        # print truth
        if truth.all():
          ik_solution.append([theta_1, theta_2, theta_3, theta_4])

    return np.round(ik_solution,6)
    # return (ik_candidate)

def fk_solver(theta_1, theta_2, theta_3, theta_4):
    x = np.cos(theta_1)*(l2*np.cos(theta_2) + l3*np.cos(theta_2+theta_3)) + l4*np.cos(theta_1)*np.cos(theta_2+theta_3+theta_4)
    y = np.sin(theta_1)*(l2*np.cos(theta_2) + l3*np.cos(theta_2+theta_3)) + l4*np.sin(theta_1)*np.cos(theta_2+theta_3+theta_4)
    z = l1 + l2*np.sin(theta_2) + l3*np.sin(theta_2+theta_3) + l4 * np.sin(theta_2+theta_3+theta_4)
    return np.round((x,y,z),6)

def find_ik_candidate(xq, yq, zq, theta_1, theta_3, angle, z, ik_candidate):

    a = l3*np.sin(theta_3)
    b = l2 + l3*np.cos(theta_3)
    c = z - l1 - l4*np.sin(angle)
    r = np.sqrt(a**2 + b**2)
    # print "a b c r: ", a, b, c,r
    # theta_2_ac = np.arctan2(c, np.sqrt(abs(r**2 - c**2)))

    # if (theta_1 < (np.pi/2)) and (theta_1 > -(np.pi/2)):
    #   if xq < 0:
    #     theta_2_ac = np.pi - theta_2_ac

    # elif (theta_1 <= np.pi) and (theta_1 >= -np.pi) and (theta_1 > (np.pi/2)) and (theta_1 < -(np.pi/2)):
    #   if xq > 0:
    #     theta_2_ac = np.pi - theta_2_ac

    # elif theta_1 == (np.pi/2):
    #   if yq < 0:
    #     theta_2_ac = np.pi - theta_2_ac

    # elif theta_1 == -(np.pi/2):
    #   if yq > 0:
    #     theta_2_ac = np.pi - theta_2_ac



    
    theta_2_1 = np.arctan2(c, np.sqrt(abs(r**2 - c**2))) - np.arctan2(a,b) 

    if not np.isnan(theta_2_1):
        ik_candidate.append([theta_1, np.pi/2 - theta_2_1, -theta_3, -(angle - theta_2_1 - theta_3)])
        # ik_candidate.append([theta_1, theta_2_1, theta_3, (angle - theta_2_1 - theta_3)])

    theta_2_2 = np.arctan2(c, -np.sqrt(abs(r**2 - c**2))) - np.arctan2(a,b)


    if not np.isnan(theta_2_2):
        ik_candidate.append([theta_1, np.pi/2 - theta_2_2, -theta_3, -(angle - theta_2_2 - theta_3)])
        # ik_candidate.append([theta_1, theta_2_2, theta_3, (angle - theta_2_2 - theta_3)])

    return ik_candidate
