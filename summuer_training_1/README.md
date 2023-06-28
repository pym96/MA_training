# SLAM system: Feature points marching 

## Intro: One Slam system inclues two subsystem, front-end and back-end

### Front-end: Visual odometer 

Estimating roughly the camera's motino and pose by adjacent images

Major algorithm: 
                1. Feature point method
                2. Straightforward method
                
#### 1. Feature point method one: ORB (Oritented Fast)
Based using second derivative mathmetic trick and setting threshold num,
we'll find feature points to allow us fit in the kinematic motion prediction
