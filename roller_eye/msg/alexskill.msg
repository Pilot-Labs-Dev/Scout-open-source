#alex skill command
#0-turn left 1-turn right 2-move left 3-move right 4-move forward 
#5-move bacward 6-clockwise rotate 7-counterclockwise rotate
#error and ok
int8 ALEX_S_TURNLEFT                              = 0
int8 ALEX_S_TURNRIGHT                           = 1
int8 ALEX_S_MOVELEFT                              = 2
int8 ALEX_S_MOVERIGHT                           = 3
int8 ALEX_S_MOVEFORWARD                   = 4
int8 ALEX_S_MOVEBACKWARD                = 5
int8 ALEX_S_CLOCKWISEROTATE           = 6
int8 ALEX_S_COUNTERCLOCKROTATE = 7

uint32 cmd

#motion distance
float32 dist

#x velocity
float32 vx

#y velocity
float32 vy

#Rotationl Speed
float32 w



