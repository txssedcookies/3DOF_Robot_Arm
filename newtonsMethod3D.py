import numpy as np

#Getters

# Assumptions
# 1) Arms are 25 cm like in the CAD
# 2) Distance between tip of EE and 3rd joint is 16.5 cm in x-z plane only
# 3) Marker tip extends 3 cm past the tip of the EE in the x-z plane only
# 4) Max radius on suface is 45 cm (Lmax is rounded down to 65 cm)

def getArm1Length():
    l1 = 0.25
    return l1

def getArm2Length():
    l2 = 0.25
    return l2

# added length due to EE and marker
def getExtraLength():
    le = 0.195
    return le

# height of base (from floor to first joint)
def getBaseHeight():
    h = 0.11
    return h

def NewtonsMethod(coordinates):
    ''' 
    (list) -> list

    Preconditions: 
    x is between 0.4 m and 0.45m
    y is between 0.0 m and 0.325 m
    z is between -0.325 m and 0.325 m

    Description: Takes in a list of coordinates [x,y,z] of the tip of the marker and returns a list of angles [t1,t2,t3].
    '''

    # initial guesses of angles
    guesses = [0.0, 1.226174801538449, -2.046259168569968]

    l1 = getArm1Length()
    l2 = getArm2Length()
    le = getExtraLength()

    # coordinates of the end-effector
    x = coordinates[0]
    y = coordinates[1]
    z = coordinates[2]

    # Height from ground to first joint 
    h = getBaseHeight() 

    # angles {t1, t2, t3: theta1, theta2, theta3}
    t1 = guesses[0]
    t2 = guesses[1]
    t3 = guesses[2]

    # 3 by 3 jacobian matix
    jacobian = [np.zeros(3),np.zeros(3),np.zeros(3)]

    for i in range(10):

        # empty list of size 2
        functions = np.zeros(3)

        # the negative of the rearranged equations for the coordinates x, y, and z
        functions[0] = -((le + l1*np.cos(t2) + l2*np.cos(t2 + t3))*np.cos(t1) - x)
        functions[1] = -(l1*np.sin(t2) + l2*np.sin(t2 + t3) + h - y)
        functions[2] = -((le + l1*np.cos(t2) + l2*np.cos(t2 + t3))*np.sin(t1) - z)

        
        # first index is row: function
        # second index is column: variable
        jacobian[0][0] = -(le + l1*np.cos(t2) + l2*np.cos(t2+ t3))*np.sin(t1)
        jacobian[0][1] = -(l1*np.sin(t2) + l2*np.sin(t2 + t3))*np.cos(t1)
        jacobian[0][2] = -l2*np.sin(t2 + t3)*np.cos(t1)

        jacobian[1][0] = 0
        jacobian[1][1] = l1*np.cos(t2) + l2*np.cos(t2 + t3)
        jacobian[1][2] = l2*np.cos(t2 + t3)

        jacobian[2][0] = (le + l1*np.cos(t2) + l2*np.cos(t2 + t3))*np.cos(t1)
        jacobian[2][1] = -(l1*np.sin(t2) + l2*np.sin(t2 + t3))*np.sin(t1)
        jacobian[2][2] = -l2*np.sin(t2 + t3)*np.sin(t1)

        # solve for vector of delta xs
        delta = np.linalg.solve(jacobian,functions)

        t1 += delta[0]
        t2 += delta[1]
        t3 += delta[2]
        
    return [t1,t2,t3]