#!/usr/bin/env python
"""
Copyright 2020 Daehyung Park

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import copy
import random
import numpy as np
import math

from geometry_msgs.msg import Quaternion

# NOTE: tf's quaternion order is [x,y,z,w]


def quat_angle(quat1, quat2):
    """ angle between two quaternions (as lists) """

    if type(quat1) == Quaternion:
        quat1 = [quat1.x, quat1.y, quat1.z, quat1.w]
    if type(quat2) == Quaternion:
        quat2 = [quat2.x, quat2.y, quat2.z, quat2.w]

    dot = math.fabs(sum([x*y for (x,y) in zip(quat1, quat2)]))
    if dot > 1.0: dot = 1.0
    angle = 2*math.acos( dot )
    return angle     


# t=0, then qm=qa
def slerp(qa, qb, t):
    """ 
    A function that generates a orientation trajectory using SLERP.

    Parameters
    ----------
    qa : list
        a start quaternion
    qb : list
        a goal quaternion
    t : float
        progress between 0 and 1
    """        

    if type(qa) == np.ndarray or type(qa) == list:
        q1 = Quaternion()
        q1.x = qa[0]
        q1.y = qa[1]
        q1.z = qa[2]
        q1.w = qa[3]
    else: q1 = qa
    if type(qb) == np.ndarray or type(qb) == list:
        q2 = Quaternion()
        q2.x = qb[0]
        q2.y = qb[1]
        q2.z = qb[2]
        q2.w = qb[3]
    else: q2 = qb
    
	# quaternion to return
    qm = Quaternion()

	# Calculate angle between them.
    cosHalfTheta = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z
	# if q1=q2 or q1=-q2 then theta = 0 and we can return q1
    if abs(cosHalfTheta) >= 1.0:
        qm.w = q1.w;qm.x = q1.x;qm.y = q1.y;qm.z = q1.z
        if type(qa) == np.ndarray or type(qa) == list: return qa
        else: return qm

    # shortest path
    if cosHalfTheta < 0.0:
        q2.w *= -1.0
        q2.x *= -1.0
        q2.y *= -1.0
        q2.z *= -1.0
        
        cosHalfTheta *= -1.0

    # Calculate temporary values.
    halfTheta = np.arccos(cosHalfTheta)
    sinHalfTheta = np.sqrt(1.0 - np.cos(halfTheta)*np.cos(halfTheta))
    
    # if theta = 180 degrees then result is not fully defined
    # we could rotate around any axis normal to q1 or q2
    if abs(sinHalfTheta) < 0.001: # fabs is floating point absolute
        qm.w = (q1.w * 0.5 + q2.w * 0.5)
        qm.x = (q1.x * 0.5 + q2.x * 0.5)
        qm.y = (q1.y * 0.5 + q2.y * 0.5)
        qm.z = (q1.z * 0.5 + q2.z * 0.5)
        if type(qa) == np.ndarray or type(qa) == list:
            return np.array([qm.x, qm.y, qm.z, qm.w])
        else: return qm

    ratioA = np.sin((1 - t) * halfTheta) / sinHalfTheta
    ratioB = np.sin(t * halfTheta) / sinHalfTheta

    #calculate Quaternion.
    qm.w = (q1.w * ratioA + q2.w * ratioB)
    qm.x = (q1.x * ratioA + q2.x * ratioB)
    qm.y = (q1.y * ratioA + q2.y * ratioB)
    qm.z = (q1.z * ratioA + q2.z * ratioB)
    
    #mag = np.sqrt(qm.w**2+qm.x**2+qm.y**2+qm.z**2)
    #print mag
    ## qm.w /= mag
    ## qm.x /= mag
    ## qm.y /= mag
    ## qm.z /= mag    
    if (type(qa) == np.ndarray and type(qb) == np.ndarray) or \
        (type(qa) == list and type(qb) == list):
        return np.array([qm.x, qm.y, qm.z, qm.w])
    else:
        return qm

    

def euler2quat(z=0, y=0, x=0, w_first=False):
    ''' Return quaternion corresponding to these Euler angles

    Uses the z, then y, then x convention above

    Parameters
    ----------
    z : scalar
       Rotation angle in radians around z-axis (performed first)
    y : scalar
       Rotation angle in radians around y-axis
    x : scalar
       Rotation angle in radians around x-axis (performed last)

    Returns
    -------
    quat : array shape (4,)
       Quaternion in w, x, y z (real, then vector) format

    Notes
    -----
    We can derive this formula in Sympy using:

    1. Formula giving quaternion corresponding to rotation of theta radians
       about arbitrary axis:
       http://mathworld.wolfram.com/EulerParameters.html
    2. Generated formulae from 1.) for quaternions corresponding to
       theta radians rotations about ``x, y, z`` axes
    3. Apply quaternion multiplication formula -
       http://en.wikipedia.org/wiki/Quaternions#Hamilton_product - to
       formulae from 2.) to give formula for combined rotations.
    '''
    z = z/2.0
    y = y/2.0
    x = x/2.0
    cz = np.cos(z)
    sz = np.sin(z)
    cy = np.cos(y)
    sy = np.sin(y)
    cx = np.cos(x)
    sx = np.sin(x)

    if w_first:
        return np.array([cx*sy*sz + cy*cz*sx,
                         cx*cz*sy - sx*cy*sz,
                         cx*cy*sz + sx*cz*sy,
                         cx*cy*cz - sx*sy*sz])
    else:
        return np.array([cx*cz*sy - sx*cy*sz,
                         cx*cy*sz + sx*cz*sy,
                         cx*cy*cz - sx*sy*sz,
                         cx*sy*sz + cy*cz*sx])



# Conversion -----------------------------------
def array2quat(array):
    return Quaternion(x=array[0],y=array[1],z=array[2],w=array[3])


# Return a quaternion array from a quaternion
def quat2array(quat):
    return np.array([quat.x, quat.y, quat.z, quat.w])
    
