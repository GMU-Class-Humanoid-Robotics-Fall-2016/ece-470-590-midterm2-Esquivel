#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2014, Daniel M. Lofaro <dan (at) danLofaro (dot) com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */
import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np
import math

dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW_R   = 'robot-vid-chan-r'
ROBOT_CHAN_VIEW_L   = 'robot-vid-chan-l'
ROBOT_TIME_CHAN  = 'robot-time'
# CV setup 
cv.NamedWindow("wctrl_L", cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow("wctrl_R", cv.CV_WINDOW_AUTOSIZE)
#capture = cv.CaptureFromCAM(0)
#capture = cv2.VideoCapture(0)

# added
##sock.connect((MCAST_GRP, MCAST_PORT))
newx = 320
newy = 240

nx = 320
ny = 240

r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
vl = ach.Channel(ROBOT_CHAN_VIEW_L)
vl.flush()
vr = ach.Channel(ROBOT_CHAN_VIEW_R)
vr.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0

GREEN_R_LIMIT = np.array([0, 30])
GREEN_G_LIMIT = np.array([50, 255])
GREEN_B_LIMIT = np.array([0, 30])

prevTime = 0
prevError = 0
integral = 0
Kp = 3.0
Ki = 1.5
Kd = 1.
firstLoop = True

def findColor(img, rLimit, gLimit, bLimit):
	
	found = False
	totalCount = 0.0
	X = 0.0
	Y = 0.0
	newimg = np.copy(img)
	redmask   = np.logical_and(newimg[:,:,2] >= rLimit[0], newimg[:,:,2] <= rLimit[1])
	greenmask = np.logical_and(newimg[:,:,1] >= gLimit[0], newimg[:,:,1] <= gLimit[1])
	bluemask  = np.logical_and(newimg[:,:,0] >= bLimit[0], newimg[:,:,0] <= bLimit[1])
	
	finalMask = np.logical_and(redmask, greenmask)
	finalMask = np.logical_and(finalMask,bluemask)
	
	
	for j in range(ny):
		for i in range(nx):
			if(finalMask[j,i]):
	#			totalCount = totalCount + 1					
				X = X + i
				Y = Y + j
	totalCount = np.sum(finalMask) 
					
	if(totalCount > 0):	
		X = X / totalCount
		Y = Y / totalCount
		found = True	
	return  found, X, Y, totalCount 

print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'
while True:
    # Get Frame
	imgL = np.zeros((newx,newy,3), np.uint8)
	imgR = np.zeros((newx,newy,3), np.uint8)
	c_image = imgL.copy()
	c_image = imgR.copy()
	vidL = cv2.resize(c_image,(newx,newy))
	vidR = cv2.resize(c_image,(newx,newy))
	[status, framesize] = vl.get(vidL, wait=False, last=True)
	if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
		vid2 = cv2.resize(vidL,(nx,ny))
		imgL = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
		Y = 120
		X = 160
		
		imgL[Y-1,X,0] = imgL[Y-1,X,1] = imgL[Y-1,X,2] = 255
		
		imgL[Y,X-1,0] = imgL[Y,X-1,1] = imgL[Y,X-1,2] = 255	
		imgL[Y,X  ,0] = imgL[Y,X  ,1] = imgL[Y,X  ,2] = 255	
		imgL[Y,X+1,0] = imgL[Y,X+1,1] = imgL[Y,X+1,2] = 255
	
		imgL[Y+1,X,0] = imgL[Y+1,X,1] = imgL[Y+1,X,2] = 255	
		
		cv2.imshow("wctrl_L", imgL)
		cv2.waitKey(10)
	else:
		raise ach.AchException( v.result_string(status) )
	[status, framesize] = vr.get(vidR, wait=False, last=True)
	if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
		vid2 = cv2.resize(vidR,(nx,ny))
		imgR = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
		cv2.imshow("wctrl_R", imgR)
		cv2.waitKey(10)
	else:
		raise ach.AchException( v.result_string(status) )


	[status, framesize] = t.get(tim, wait=False, last=True)
	if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
		pass
        #print 'Sim Time = ', tim.sim[0]
	else:
		raise ach.AchException( v.result_string(status) )

#-----------------------------------------------------
#-----------------------------------------------------
#-----------------------------------------------------
    # Def:
    # ref.ref[0] = Right Wheel Velos
    # ref.ref[1] = Left Wheel Velos
    # tim.sim[0] = Sim Time
    # imgL       = cv image in BGR format (Left Camera)
    # imgR       = cv image in BGR format (Right Camera)
    
	[foundSquare, X, Y, area] = findColor(imgL, GREEN_R_LIMIT, GREEN_G_LIMIT, GREEN_B_LIMIT)
	print foundSquare
	if (foundSquare == False):
		ref.ref[0] =  -1.0 * math.radians(60.0)#-0.5
		ref.ref[1] = math.radians(60.0)#0.5
	if(foundSquare == True):
		
		if(firstLoop == True):
			prevTime = tim.sim[0]
			firstLoop = False
			continue
			
		currTime = tim.sim[0]
		T = currTime - prevTime 
		currError = (160.0 - X) * 1.047/320.0 #in radians from center
		integral = integral + (currError * T)
		derivative = (currError - prevError)/ T
		
		offset = Kp*currError + Ki*integral + Kd*derivative
		
		prevTime = currTime
		prevError = currError
		
		
		ref.ref[0] = offset
		ref.ref[1] = -1. * offset
		print X, offset
	
	# Commands Robot
	r.put(ref)
    # Sleeps
	time.sleep(0.1)   
#-----------------------------------------------------
#-----------------------------------------------------
#-----------------------------------------------------
