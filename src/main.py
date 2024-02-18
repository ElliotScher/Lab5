# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Elliot Scher                                                 #
# 	Created:      2/16/2024, 3:35:04 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

brain.screen.print("Hello V5")





class DevicePorts:
    FL_DRIVE = Ports.PORT19
    FR_DRIVE = Ports.PORT10
    BL_DRIVE = Ports.PORT20
    BR_DRIVE = Ports.PORT9

    GYRO = Ports.PORT8

    LEFT_LIFT = Ports.PORT18
    RIGHT_LIFT = Ports.PORT7

    GATE = Ports.PORT6





class Constants:
        WHEEL_DIAMETER_IN = 4
        WHEEL_CIRCUMFERENCE_IN = WHEEL_DIAMETER_IN * math.pi

        DRIVE_BASE_RADIUS_IN = 7.75 # TODO
        DRIVE_BASE_CIRCUMFERENCE_IN = DRIVE_BASE_RADIUS_IN * 2 * math.pi

        MOTOR_MAX_SPEED_RPM = 200

        LOOP_PERIOD_MSECS = 20

        # where phi is the angle between the x/y axis and the wheel vectors, which is always some multiple of pi/4
        SEC_PHI = 2 / math.sqrt(2)

        DRIVE_TRAIN_IDLE_MODE = BRAKE

        DRIVE_TRANSLATION_KP = 0.04

        DRIVE_ROTATION_KP = 0.8

        DRIVE_MAX_SPEED_METERS_PER_SEC = 0.2

        ODOM_X_POSITIVE_DRIFT = 194 / 200
        ODOM_X_NEGATIVE_DRIFT = 207 / 200
        ODOM_Y_POSITIVE_DRIFT = 211 / 200
        ODOM_Y_NEGATIVE_DRIFT = 180 / 200

        ODOM_Y_DRIFT_PER_POSITIVE_X_TRANSLATION = -12 / 200
        ODOM_Y_DRIFT_PER_NEGATIVE_X_TRANSLATION = 18 / 200

        LIFT_SPEED = 150





class Odometry:
    def __init__(self, x: float, y: float, theta: float):
        self.xMeters = x
        self.yMeters = y
        self.thetaRad = theta
        # self.loopPeriodSecs = loopPeriodMSecs / 1000
        # self.timer = Timer()

    # def update(self, xVelocityMetersPerSec: float, yVelocityMetersPerSec: float, headingRad: float):
    #     self.xMeters += (xVelocityMetersPerSec * self.timer.value())
    #     self.yMeters += (yVelocityMetersPerSec * self.timer.value())
    #     self.rotationRad = headingRad
    #     self.timer.reset()
    
    prevTranslationMeters = 0.0
    def update(self, fieldOrientedTranslationRad: float, translationMeters: float, headingRad: float):
        translationDeltaMeters = abs(translationMeters - self.prevTranslationMeters)
        self.prevTranslationMeters = translationMeters

        # As the wheels approach parallel/orthoginal to the drive direction, we don't need to compensate for carpet drift
        robotRelativeTranslationRad = fieldOrientedTranslationRad + headingRad
        driftCompPercentage = abs(math.cos((2 * robotRelativeTranslationRad) % math.pi))

        xDelta = translationDeltaMeters * math.cos(fieldOrientedTranslationRad)
        yDelta = translationDeltaMeters * math.sin(fieldOrientedTranslationRad)

        if(xDelta > 0):
            xDelta *= (Constants.ODOM_X_POSITIVE_DRIFT ** driftCompPercentage)
        else:
            xDelta *= (Constants.ODOM_X_NEGATIVE_DRIFT ** driftCompPercentage)

        if(yDelta > 0):
            yDelta *= ((Constants.ODOM_Y_POSITIVE_DRIFT) ** driftCompPercentage)
            yDelta += ((abs(xDelta) * Constants.ODOM_Y_DRIFT_PER_POSITIVE_X_TRANSLATION) * driftCompPercentage)
        else:
            yDelta *= ((Constants.ODOM_Y_NEGATIVE_DRIFT + (xDelta * Constants.ODOM_Y_DRIFT_PER_NEGATIVE_X_TRANSLATION)) ** driftCompPercentage)
            yDelta += ((xDelta * Constants.ODOM_Y_DRIFT_PER_NEGATIVE_X_TRANSLATION) * driftCompPercentage)

        self.xMeters += xDelta
        self.yMeters += yDelta
        self.thetaRad = headingRad
    
    def getXMeters(self):
        return self.xMeters
    
    def getYMeters(self):
        return self.yMeters
    
    def getThetaRad(self):
        return self.thetaRad





class Drive:
    flDrive = Motor(DevicePorts.FL_DRIVE)
    frDrive = Motor(DevicePorts.FR_DRIVE)
    blDrive = Motor(DevicePorts.BL_DRIVE)
    brDrive = Motor(DevicePorts.BR_DRIVE)

    flDrive.set_stopping(Constants.DRIVE_TRAIN_IDLE_MODE)
    frDrive.set_stopping(Constants.DRIVE_TRAIN_IDLE_MODE)
    blDrive.set_stopping(Constants.DRIVE_TRAIN_IDLE_MODE)
    brDrive.set_stopping(Constants.DRIVE_TRAIN_IDLE_MODE)

    gyro = Inertial(DevicePorts.GYRO)

    odometry = Odometry(0, 0, 0)

    def __init__(self, heading = 0, calibrateGyro = False):
        if calibrateGyro:
            self.gyro.calibrate()
            while self.gyro.is_calibrating():
                pass
            print("GYRO CALIBRATED")
            self.gyro.set_heading(heading)

    @staticmethod
    def __revolutionsToMeters(revolutions):
        return (revolutions * Constants.WHEEL_CIRCUMFERENCE_IN) / (39.3701)

    @staticmethod
    def __metersPerSecToRPM(speedMetersPerSec):
        return (speedMetersPerSec * 39.3701 * 60) / (Constants.WHEEL_CIRCUMFERENCE_IN)
    
    @staticmethod
    def __rpmToMetersPerSecond(speedMetersPerSec):
        return (speedMetersPerSec * Constants.WHEEL_CIRCUMFERENCE_IN) / (39.3701 * 60)
    
    @staticmethod
    def __radPerSecToRPM(speedRadPerSec):
        return (speedRadPerSec * Constants.WHEEL_CIRCUMFERENCE_IN * 60) / (2 * math.pi * Constants.WHEEL_CIRCUMFERENCE_IN)

    def periodic(self):
        self.odometry.update(
            self.getActualDirectionOfTravelRad(),
            self.getDistanceTraveledMeters(),
            self.gyro.heading(RotationUnits.REV) * 2 * math.pi
        )

    def applyDesaturated(self, flSpeedRPM, frSpeedRPM, blSpeedRPM, brSpeedRPM):
        fastestSpeedRPM = max(abs(flSpeedRPM), abs(frSpeedRPM), abs(blSpeedRPM), abs(brSpeedRPM))
        if(fastestSpeedRPM > Constants.MOTOR_MAX_SPEED_RPM):
            ratio = Constants.MOTOR_MAX_SPEED_RPM / fastestSpeedRPM

            flSpeedRPM *= ratio
            frSpeedRPM *= ratio
            blSpeedRPM *= ratio
            brSpeedRPM *= ratio
        
        self.flDrive.spin(FORWARD, round(flSpeedRPM, 4))
        self.frDrive.spin(FORWARD, round(frSpeedRPM, 4))
        self.blDrive.spin(FORWARD, round(blSpeedRPM, 4))
        self.brDrive.spin(FORWARD, round(brSpeedRPM, 4))

    def stop(self):
        self.flDrive.stop()
        self.frDrive.stop()
        self.blDrive.stop()
        self.brDrive.stop()

    def applySpeeds(self, directionRad: float, translationSpeedMetersPerSec: float, rotationSpeedRadPerSec: float, fieldOriented: bool):
        translationRPM = self.__metersPerSecToRPM(translationSpeedMetersPerSec)
        rotationRPM = -self.__radPerSecToRPM(rotationSpeedRadPerSec)

        # offset lateral direction by gyro heading for field-oriented control
        if fieldOriented:
            directionRad += self.gyro.heading(RotationUnits.REV) * 2 * math.pi

        # find the x and y components of the direction vector mapped to a wheel vector
        coeffRPM = Constants.SEC_PHI * translationRPM
        xProjectionRPM = coeffRPM * math.sin(directionRad)
        yProjectionRPM = coeffRPM * math.cos(directionRad)

        self.applyDesaturated(
            rotationRPM - (xProjectionRPM - yProjectionRPM),
            rotationRPM - (xProjectionRPM + yProjectionRPM),
            rotationRPM + (xProjectionRPM + yProjectionRPM),
            rotationRPM + (xProjectionRPM - yProjectionRPM)
        )

    def getActualDirectionOfTravelRad(self, fieldOriented = True):
        xFL = self.flDrive.velocity() * math.cos(7 * math.pi / 4)
        yFL = self.flDrive.velocity() * math.sin(7 * math.pi / 4)

        xFR = self.blDrive.velocity() * math.cos(math.pi / 4)
        yFR = self.blDrive.velocity() * math.sin(math.pi / 4)
        
        xBL = self.frDrive.velocity() * math.cos(5 * math.pi / 4)
        yBL = self.frDrive.velocity() * math.sin(5 * math.pi / 4)

        xBR = self.brDrive.velocity() * math.cos(3 * math.pi / 4)
        yBR = self.brDrive.velocity() * math.sin(3 * math.pi / 4)

        xSumVectors = xFL + xFR + xBL + xBR
        ySumVectors = yFL + yFR + yBL + yBR

        direction = math.atan2(ySumVectors, xSumVectors)

        if fieldOriented:
            direction -= self.gyro.heading(RotationUnits.REV) * 2 * math.pi

        return direction
    
    def getDistanceTraveledMeters(self):
        xFL = self.__revolutionsToMeters(self.flDrive.position()) * math.cos(7 * math.pi / 4)
        yFL = self.__revolutionsToMeters(self.flDrive.position()) * math.sin(7 * math.pi / 4)

        xFR = self.__revolutionsToMeters(self.blDrive.position()) * math.cos(math.pi / 4)
        yFR = self.__revolutionsToMeters(self.blDrive.position()) * math.sin(math.pi / 4)
        
        xBL = self.__revolutionsToMeters(self.frDrive.position()) * math.cos(5 * math.pi / 4)
        yBL = self.__revolutionsToMeters(self.frDrive.position()) * math.sin(5 * math.pi / 4)

        xBR = self.__revolutionsToMeters(self.brDrive.position()) * math.cos(3 * math.pi / 4)
        yBR = self.__revolutionsToMeters(self.brDrive.position()) * math.sin(3 * math.pi / 4)

        xSumVectors = xFL + xFR + xBL + xBR
        ySumVectors = yFL + yFR + yBL + yBR

        magnitude = magnitude = self.__rpmToMetersPerSecond(math.sqrt(xSumVectors**2 + ySumVectors**2) / 4)

        return magnitude
    
    def getActualSpeedMetersPerSec(self):
        xFL = self.flDrive.velocity() * math.cos(7 * math.pi / 4)
        yFL = self.flDrive.velocity() * math.sin(7 * math.pi / 4)

        xFR = self.blDrive.velocity() * math.cos(math.pi / 4)
        yFR = self.blDrive.velocity() * math.sin(math.pi / 4)
        
        xBL = self.frDrive.velocity() * math.cos(5 * math.pi / 4)
        yBL = self.frDrive.velocity() * math.sin(5 * math.pi / 4)

        xBR = self.brDrive.velocity() * math.cos(3 * math.pi / 4)
        yBR = self.brDrive.velocity() * math.sin(3 * math.pi / 4)

        xSumVectors = xFL + xFR + xBL + xBR
        ySumVectors = yFL + yFR + yBL + yBR

        magnitude = self.__rpmToMetersPerSecond(math.sqrt(xSumVectors**2 + ySumVectors**2) / 4)

        return magnitude
    
    def driveToPosition(self, xMeters: float, yMeters: float, headingRad: float):
        xError = xMeters - self.odometry.getXMeters()
        yError = yMeters - self.odometry.getYMeters()

        xEffort = xError * Constants.DRIVE_TRANSLATION_KP
        yEffort = yError * Constants.DRIVE_TRANSLATION_KP

        direction = math.atan2(yEffort, xEffort)
        magnitude = math.sqrt(xError**2 + yError**2)

        thetaError = headingRad - self.odometry.getThetaRad()

        if(thetaError > math.pi):
            thetaError -= 2 * math.pi
        elif(thetaError < -math.pi):
            thetaError += 2 * math.pi

        thetaEffort = thetaError * Constants.DRIVE_ROTATION_KP

        if(abs(magnitude) > Constants.DRIVE_MAX_SPEED_METERS_PER_SEC):
            magnitude = math.copysign(Constants.DRIVE_MAX_SPEED_METERS_PER_SEC, magnitude)

        self.applySpeeds(direction, magnitude, thetaEffort, True)





class Lift:
    leftLift = Motor(DevicePorts.LEFT_LIFT, False)
    rightLift = Motor(DevicePorts.RIGHT_LIFT, True)

    PINION_CIRCUMFERENCE_IN = 0.5

    STOW_POSITION = 0.025
    LOW_POSITION = 0.15
    MID_POSITION = 3.0
    HIGH_POSITION = 5.5

    leftLift.set_velocity(Constants.LIFT_SPEED, RPM)
    rightLift.set_velocity(Constants.LIFT_SPEED, RPM)

    Kp = 0

    def __init__(self):
        pass

    def periodic(self):
        pass

    def setStowPosition(self):
        self.leftLift.spin_to_position(self.STOW_POSITION, TURNS, False)
        self.rightLift.spin_to_position(self.STOW_POSITION, TURNS, True)

    def setLowPosition(self):
        self.leftLift.spin_to_position(self.LOW_POSITION, TURNS, False)
        self.rightLift.spin_to_position(self.LOW_POSITION, TURNS, True)

    def setMidPosition(self):
        self.leftLift.spin_to_position(self.MID_POSITION, TURNS, False)
        self.rightLift.spin_to_position(self.MID_POSITION, TURNS, True)

    def setHighPosition(self):
        self.leftLift.spin_to_position(self.HIGH_POSITION, TURNS, False)
        self.rightLift.spin_to_position(self.HIGH_POSITION, TURNS, True)

    def stop(self):
        self.leftLift.stop()




class Gate:
    leftGate = Motor(DevicePorts.GATE, False)


    # DEGREES
    LOCKED_POSITION = 0.0
    UNLOCKED_POSITION = 90.0

    def __init__(self) -> None:
        pass

    def periodic(self):
        pass

    def setLockedPosition(self):
        self.leftGate.spin_to_position(self.LOCKED_POSITION, DEGREES)

    def setUnlockedPosition(self):
        self.leftGate.spin_to_position(self.UNLOCKED_POSITION, DEGREES)














drive = Drive()
lift = Lift()
gate = Gate()

timer = Timer()

state = 0

def driveState():
    global state
    if (drive.odometry.getXMeters() < 0.3048):
        drive.applySpeeds(0, 0.15, 0, True)
    else:
        drive.stop()
        state = 1

def liftState():
    global state
    lift.setMidPosition()
    lift.stop()
    state = 2

def approachState():
    global state
    if (drive.odometry.getXMeters() < 0.5):
        drive.applySpeeds(0, 0.05, 0, True)
    else:
        drive.stop()
        state = 3

def collectState():
    global state
    lift.setStowPosition()
    lift.stop()
    state = 4

def returnState():
    global state
    if (drive.odometry.getXMeters() > 0.01):
        drive.applySpeeds(math.pi, 0.2, 0, True)
    else:
        drive.stop()
        state = 5

def depositState():
    global state
    if (gate.leftGate.position(DEGREES) < gate.UNLOCKED_POSITION):
        gate.setUnlockedPosition()
    else:
        state = 6

def resetState():
    global state
    if (gate.leftGate.position(DEGREES) > gate.LOCKED_POSITION):
        gate.setLockedPosition()
    else:
        state = 7

def robotPeriodic():
    drive.periodic()
    lift.periodic()
    gate.periodic()
    if (state == 0):
        driveState()
    if (state == 1):
        liftState()
    if (state == 2):
        approachState()
    if (state == 3):
        collectState()
    if (state == 4):
        returnState()
    if (state == 5):
        depositState()
    if (state == 6):
        resetState()
    
    timer.event(robotPeriodic, Constants.LOOP_PERIOD_MSECS)







# execute main loop
drive.gyro.calibrate()
while (drive.gyro.is_calibrating()):
    pass

robotPeriodic()