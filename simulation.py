# Everything in SI Unit and in degrees

import time
import turtle
import math



# PID Gains (Haven't yet tuned)
Kp = 0.01
Ki = 0.00005
Kd = 0.01

# Setpoint & Sensor Measurement
SETPOINT = 0
sensorTiltAngle = 10  # positive mean point to the right  # negative mean left

# PID Error Parameters
pastError1 = 0
integralError1 = 0
output1 = 0
output2 = 0

# Time
timeInterval = 0.1

# Rocket Parameters
thrust = 10  
lengthOfMomentArm = 100  # Distance from the rocket tail to the centre of mass of the rocket
lengthofHeadToCOM = 50
momentOfInertia = 15  
initialTailPosition_Y = -lengthOfMomentArm
initialHeadPosition_Y = lengthofHeadToCOM
initialAngularPosition = sensorTiltAngle  
initialAngularVelocity = 0 

# PID Control Variable
thrustAngle = 0



class Rocket():
    def __init__(self, tiltAngle):

        # Center of mass of rocket (i.e. center of rotation)
        self.COM = turtle.Turtle()
        self.COM.pendown()
        self.COM.shape("circle")
        self.COM.color("red")
        self.COM.goto(SETPOINT, 0)

        # Rocket Tail
        self.rocketTail = turtle.Turtle()
        self.rocketTail.penup()
        self.rocketTail.shape("square")
        self.rocketTail.color("black")

        # Rocket Head
        self.rocketHead = turtle.Turtle()
        self.rocketHead.penup()
        self.rocketHead.shape("square")
        self.rocketHead.color("black")

        self.updateRocketPosition(tiltAngle)


    def updateRocketPosition(self, tiltAngle):

        tiltAngle_InRadians = math.radians(tiltAngle)

        # Rocket Tail
        rocketTailPosition_X = lengthOfMomentArm * math.sin(tiltAngle_InRadians)
        # print(rocketTailPosition_X)
        rocketTailPosition_Y = lengthOfMomentArm * math.cos(tiltAngle_InRadians)
        self.rocketTail.goto(rocketTailPosition_X, -rocketTailPosition_Y)

        # Rocket Head
        rocketHeadPosition_X = lengthofHeadToCOM * math.sin(tiltAngle_InRadians)
        rocketHeadPosition_Y = lengthofHeadToCOM * math.cos(tiltAngle_InRadians)
        self.rocketHead.goto(-rocketHeadPosition_X, rocketHeadPosition_Y)



def thrustCalculations(thrustAngle, initialAngularPosition, initialAngularVelocity):

    # Torque Calculation
    TVC_Angle_InRadians = math.radians(thrustAngle)
    xComponentOfThrust = thrust * math.sin(TVC_Angle_InRadians)
    torque = xComponentOfThrust * lengthOfMomentArm

    # Angular Kinematic Calculation
    angularAcceleration = torque / momentOfInertia
    angularDisplacement = initialAngularVelocity * timeInterval + (angularAcceleration * (timeInterval ** 2)) / 2
    angularDisplacement = angularDisplacement * -1  # times -1 so that positive thurst angle means thrust points to the right side
    finalAngularPosition = initialAngularPosition + angularDisplacement
    finalAngularVelocity = initialAngularVelocity + angularAcceleration * timeInterval

    return finalAngularPosition, finalAngularVelocity



def pid(set_point, current_point, time_change, past_error, integral_error):
    error = set_point - current_point
    derivative_error = (error - past_error) / time_change

    if (sensorTiltAngle <= 4 and sensorTiltAngle >= -4):
        integral_error += error
    else:
        integral_error = 0

    output_value = - (Kp * error + Ki * integral_error + Kd * derivative_error)

    return output_value, error, integral_error



def main():

    global sensorTiltAngle, thrustAngle, initialAngularPosition, initialAngularVelocity, pastError1, integralError1, output1, output2

    # Set Up Turtle Screen
    screen = turtle.Screen()
    screen.setup(width=600, height=600)
    screen.title("TVC Simulation")

    # Set Up Rocket
    rocket = Rocket(sensorTiltAngle)


    # Start PID Control
    pastTime = time.time()
    isRecordSteadyTime = True
    pastTime_forSteadyState = time.time()
    timeInterval_forSteadyState = 10
    startSimTime = time.time()

    while True:
        currentTime = time.time()
        timeElapsed = currentTime - pastTime

        currentTime_forSteadyState = time.time()
        timeElapsed_forSteadyState = currentTime_forSteadyState - pastTime_forSteadyState

        if timeElapsed >= timeInterval:
            output1, pastError1, integralError1 = pid(SETPOINT, sensorTiltAngle, timeElapsed, pastError1, integralError1)

            # print(f'ThrustAngle: {output1}')

            # Update pastTime
            pastTime = currentTime

        thrustAngle = output1
        
        # Thrust Take Effect
        finalAngularPosition, finalAngularVelocity = thrustCalculations(thrustAngle, initialAngularPosition, initialAngularVelocity)

        rocket.updateRocketPosition(finalAngularPosition)
        sensorTiltAngle = finalAngularPosition

        # Find Time To Reach Steady State
        if (isRecordSteadyTime):
            if sensorTiltAngle > 1:
                pastTime_forSteadyState = time.time()
            elif timeElapsed_forSteadyState >= timeInterval_forSteadyState:
                TimeTillSteadyState = time.time() - startSimTime - timeInterval_forSteadyState
                print(f'Time To Reach Steady State: {TimeTillSteadyState}')
                isRecordSteadyTime = False

        initialAngularPosition, initialAngularVelocity = finalAngularPosition, finalAngularVelocity


    # Rocket go from 0 to 90
    def rocketSwing1():
        global sensorTiltAngle
        isSwingUp = True
        while True:

            if isSwingUp and sensorTiltAngle < 90:
                sensorTiltAngle += 1

            if not isSwingUp and sensorTiltAngle > 0:
                sensorTiltAngle -= 1

            if sensorTiltAngle >= 90:
                isSwingUp = False
                sensorTiltAngle -= 1
            
            if sensorTiltAngle <= 0:
                isSwingUp = True
                sensorTiltAngle += 1

            rocket.updateRocketPosition(sensorTiltAngle)

    # Rocket go from -90 to 90
    def rocketSwing2():
        global sensorTiltAngle
        isSwingRight = True
        while True:

            if isSwingRight and sensorTiltAngle < 30:
                sensorTiltAngle += 1

            if not isSwingRight and sensorTiltAngle > -30:
                sensorTiltAngle -= 1

            if sensorTiltAngle >= 30:
                isSwingRight = False
                sensorTiltAngle -= 1
            
            if sensorTiltAngle <= -30:
                isSwingRight = True
                sensorTiltAngle += 1

            rocket.updateRocketPosition(sensorTiltAngle)
    rocketSwing2()

    # Allow the rocket to experience thrust
    while True:

        finalAngularPosition, finalAngularVelocity = thrustCalculations(thrustAngle, initialAngularPosition, initialAngularVelocity)

        rocket.updateRocketPosition(finalAngularPosition)

        initialAngularPosition, initialAngularVelocity = finalAngularPosition, finalAngularVelocity

    turtle.done()



if __name__ == '__main__':
    main()