from vpython import *
from time import *
import numpy as np
import math

scene.range=5
toRad=2*np.pi/360
toDeg=1/toRad
scene.forward=vector(-1,-1,-1)
 
scene.width=600
scene.height=600
 
xarrow=arrow(lenght=2, shaftwidth=.1, color=color.red,axis=vector(1,0,0))
yarrow=arrow(lenght=2, shaftwidth=.1, color=color.green,axis=vector(0,1,0))
zarrow=arrow(lenght=4, shaftwidth=.1, color=color.blue,axis=vector(0,0,1))
 
frontArrow=arrow(length=4,shaftwidth=.1,color=color.purple,axis=vector(1,0,0))
upArrow=arrow(length=1,shaftwidth=.1,color=color.magenta,axis=vector(0,1,0))
sideArrow=arrow(length=2,shaftwidth=.1,color=color.orange,axis=vector(0,0,1))
 
body=box(length=6,width=1,height=.2,opacity=.8,pos=vector(0,0,0,))
wings=box(length=1,width=5,height=0.1, pos=vector(+.5,.1+.05,0),color=color.blue)
rudder=box(lenght=0.1,width=.1,height=2,pos=vector(-2,.5,0),color=color.green)
plane=compound([body,wings,rudder])

def update(roll,pitch,yaw,debug=True):
    roll=roll*toRad
    pitch=pitch*toRad
    yaw=yaw*toRad+np.pi
    if debug:
        print("Roll=",roll*toDeg," Pitch=",pitch*toDeg,"Yaw=",yaw*toDeg)
    rate(50)
    k=vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
    y=vector(0,1,0)
    s=cross(k,y)
    v=cross(s,k)
    vrot=v*cos(roll)+cross(k,v)*sin(roll)
 
    frontArrow.axis=k
    sideArrow.axis=cross(k,vrot)
    upArrow.axis=vrot
    plane.axis=k
    plane.up=vrot
    sideArrow.length=2
    frontArrow.length=4
    upArrow.length=1