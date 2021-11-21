from typing import List
from controller import Robot, Motor, DistanceSensor


# --- constantes ---
TIMESTEP = 64
MAX_SPEED = 6.28
SLOW_SPEED = 3
HIGH_SPEED = 4
FIXED_SPEED = 3.0

D0_SENSOR_VALUE = 150
DMAX_SENSOR_VALUE = 4000
MIN_ANGLE = -40.0
MAX_ANGLE = 40.0

# --- Trapezes des regles d'inferences ---

# ANGLE
TabDistRight = [(0,0),(0,80), (1,500),(1,DMAX_SENSOR_VALUE)] # (Degre appartenance, Valeur)
TabAngleRight = [(1,MIN_ANGLE), (1,MIN_ANGLE+5), (0,0.0), (0,MAX_ANGLE)]

# TabDistLeft = [(0,0),(0,80),(1,500),(1,DMAX_SENSOR_VALUE)]
# TabAngleLeft = [(0,MIN_ANGLE), (0,0.0), (1,25.0), (1,MAX_ANGLE) ]

TabDistGlobal = [(0,0),(0,80),(1,140),(1,160),(0,220),(0,DMAX_SENSOR_VALUE)]
TabAngleGlobal = [(0,MIN_ANGLE),(0,MIN_ANGLE+10),(1,MIN_ANGLE+25),(1,MAX_ANGLE-25),(0,MAX_ANGLE-20), (0,MAX_ANGLE)]

TabFindRightWall = [(1,0),(1,80), (0,500),(0,DMAX_SENSOR_VALUE)]
TabAngleRightWall = [(0,MIN_ANGLE-10), (0,-5), (1, 90), (1,120) ]


# VITESSE

TabIsNotRotating = [(1,0),(1,5), (0,MAX_ANGLE),(0,100)]
TabIsNotRotatingSpeed = [(0,SLOW_SPEED), (0,SLOW_SPEED+0.25),(1, HIGH_SPEED-0.25), (1,HIGH_SPEED)]

TabIsRotating = [(0,0),(0,5), (1,MAX_ANGLE),(1,100)]
TabIsRotatingSpeed = [(1,SLOW_SPEED), (1,SLOW_SPEED+0.25),(0, HIGH_SPEED-0.25), (0,HIGH_SPEED)]

# --- Fonctions utilitaires ---

# recuperer le degre d'appartenance 
def getMembershipDegree(sensorvalue, tabDistance):
    count = 0
    degree = 0
    for i in range(1, len(tabDistance)):
        cond = tabDistance[i-1][1] <= sensorvalue < tabDistance[i][1]
        if(cond):
            inter = proport(sensorvalue, tabDistance[i], tabDistance[i-1])
            return inter
    #return degree/count    

# recuperer l'angle 
def getAngle(membershipDegree, tabAngle):
    for i in range(1, len(tabAngle)):
        cond = tabAngle[i-1][0] <= membershipDegree <= tabAngle[i][0]
        if(tabAngle[i-1][0] == 1):
            cond = tabAngle[i-1][0] >= membershipDegree >= tabAngle[i][0]
        if(cond):
            a = (tabAngle[i-1][0] - tabAngle[i][0])/(tabAngle[i-1][1] - tabAngle[i][1])
            if(a == 0 ):
                return 0
            return membershipDegree/a
    return 0

#recuperer la vitesse
def getSpeed(membershipDegree, tabSpeed):
    for i in range(1, len(tabSpeed)):
        cond = tabSpeed[i-1][0] >= membershipDegree >= tabSpeed[i][0]
        if(cond):
            a = (tabSpeed[i-1][0] - tabSpeed[i][0])/(tabSpeed[i-1][1] - tabSpeed[i][1])
            print(f"b : {a}")
            if(a == 0 ):
                return tabSpeed[i][1]
            return membershipDegree/abs(a) + tabSpeed[i-1][1]
    return 4

# recuperer la synthese des regles -- ~ Barycentre
def getBarycentre(tabValues):
    numerator = 0
    denominator = 0
    for i in range (len(tabValues)):
        numerator+= tabValues[i][0]*tabValues[i][1]
        denominator+=tabValues[i][0]
    if(denominator == 0):
        return numerator
    return numerator/denominator

# Fonction de calcul de toute les regles
def calculateFinalAngle(leftSensor, rightSensor):
    print("--- left ---")
    # leftMembershipDegree = getMembershipDegree(leftSensor, TabDistLeft)
    print("--- right ---")
    rightMembershipDegree = getMembershipDegree(rightSensor, TabDistRight)
    print("--- global ---")
    globalMembershipDegree = getMembershipDegree(max(leftsensor, rightsensor), TabDistGlobal)
    print("--- fwall ---")
    findWallMembershipDegree = getMembershipDegree(rightsensor, TabFindRightWall)

    print(f" -- rightMV = {rightMembershipDegree} -- globalMV = {globalMembershipDegree} -- findWallMV = {findWallMembershipDegree}")

    # leftAngle = getAngle(leftMembershipDegree, TabAngleLeft)
    rightAngle = getAngle(rightMembershipDegree, TabAngleRight)
    globalAngle = getAngle(globalMembershipDegree, TabAngleGlobal)
    findWallAngle = getAngle(findWallMembershipDegree, TabAngleRightWall)

    print(f" -- rightAngle = {rightAngle} -- globalAngle = {globalAngle} -- findWallAngle = {findWallAngle}")
    

    # tabValues = [(leftMembershipDegree, leftAngle),(rightMembershipDegree, rightAngle),(globalMembershipDegree, globalAngle),(findWallMembershipDegree, findWallAngle) ]
    tabValues = [(rightMembershipDegree, rightAngle),(globalMembershipDegree, globalAngle),(findWallMembershipDegree, findWallAngle)]

    angle = getBarycentre(tabValues)
    return angle

# calculer la vitesse par rapport à l'angle
def calculateFinalSpeed(angle):
    print("--- fIsNotRotating ---")
    isNotRotatingMembershipDegree = getMembershipDegree(angle, TabIsNotRotating)
    print("--- fIsRotating ---")
    isRotatingMembershipDegree = getMembershipDegree(angle, TabIsRotating)

    isNotRotatingValue = getSpeed(isNotRotatingMembershipDegree, TabIsNotRotatingSpeed)
    isRotatingValue = getSpeed(isRotatingMembershipDegree, TabIsRotatingSpeed)

    tabValues = [(isNotRotatingMembershipDegree, isNotRotatingValue),(isRotatingMembershipDegree, isRotatingValue)]
    print(f"speed tab : {tabValues}")
    speed = getBarycentre(tabValues)
    return speed

def proport(sensorValue, tabElement, tabElementPrec):
    if(tabElement[0] == tabElementPrec[0]):
        return tabElement[0]
    if(tabElement[1] == 0):
        return 0
    return (sensorValue / (tabElement[1] - tabElementPrec[1]))*tabElement[0]

# convertir un angle en vitesse moteur
def convertToMotorVelocity(angle, speed):
    if(angle == 0):
        leftMotor.setVelocity( speed )
        rightMotor.setVelocity( speed )
    elif(angle > 0):
        leftMotor.setVelocity( speed - (abs(angle)/40))
        rightMotor.setVelocity( speed + (abs(angle)/40) )
    else:
        leftMotor.setVelocity( speed + (abs(angle)/40))
        rightMotor.setVelocity( speed - (abs(angle)/40) )
    # leftMotor.setVelocity( speed - (angle/40))
    # rightMotor.setVelocity( speed + (angle/40))

# create the Robot instance.
robot = Robot()

ps = []
psNames = [
    'ps1',
    'ps2',
    'ps5',
    'ps6'
]

for i in range(4):
    ps.append(robot.getDistanceSensor(psNames[i]))
    ps[i].enable(TIMESTEP)
    
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# --- Boucle ---

while robot.step(TIMESTEP) != -1:
    # on recupere les valeurs des sensors de gauche et droite 
    print("\n----------------------------------\n")
    leftsensor = max(ps[0].getValue(), ps[1].getValue())
    rightsensor = max(ps[2].getValue(), ps[3].getValue())

    print(f" leftsensor : {leftsensor} -- rightsensor : {rightsensor}")

    angle = calculateFinalAngle(leftsensor, rightsensor)
    speed = calculateFinalSpeed(abs(angle))
    print(f"Angle : {angle} -- Speed : {speed}")
    convertToMotorVelocity(angle, speed)
    pass