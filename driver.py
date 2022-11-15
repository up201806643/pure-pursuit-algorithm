'''
Created on Apr 4, 2012

@author: lanquarden
'''

import msgParser
import carState
import carControl
import math
import uteis
from simple_pid import PID


class Driver(object):

    def __init__(self, stage):
        '''Constructor'''
        self.PI_HALF = math.pi / 2.0 # 90 deg
        self.PI_FOURTHS = math.pi / 4.0 # 90 deg
        self.RAD_PER_DEG = math.pi / 180.0
        self.DEG_PER_RAD = 1.0 / self.RAD_PER_DEG 

        self.DEFAULT_MIN_SPEED = 50
        self.DEFAULT_MAX_SPEED = 250

        self.GEAR_MAX = 6      
        self.RPM_MAX = 7000    
        self.ACCEL_MAX = 1.0    
        self.ACCEL_DELTA = 0.5             # maximum rate of change in acceleration signal, avoid spinning out

        self.BRAKING_MAX = -1           # braking signal <= BRAKING_MAX 
        self.BRAKING_DELTA = 0.1          # dampen braking to avoid lockup, max rate of chage in braking
        self.WHEELSPIN_ACCEL_DELTA = 0.025
        self.WHEELSPIN_MAX = 5.0           # greater than this value --> loss of control

        self.PURE_PURSUIT_K = 0.35         #  bias - increase to reduces steering sensitivity 
        self.PURE_PURSUIT_L = 2.4          #   approx vehicle wheelbase
        self.PURE_PURSUIT_2L = 2 * self.PURE_PURSUIT_L
        self.MAX_STEERING_ANGLE_DEG = 21   #  steering lock
        self.USE_STEERING_FILTER = False  
        self.STEERING_FILTER_SIZE = 5  

        self.EDGE_AVOIDANCE_ENABLED = True
        self.EDGE_MAX_TRACK_POS = 0.85     #  track edge limit
        self.EDGE_STEERING_INPUT = 0.0075  #  slightly steer away from edge

        self.STALLED_TIMEOUT = 5           # seconds of no significant movement

        #TODO: implement mode for when vehicle is off track, e.g., a spin or missed turn

        self.LEFT_SIDE = 1
        self.RIGHT_SIDE = -1
        self.MIDDLE = 0
        self.Q1 = 1
        self.Q2 = 2
        self.Q3 = 3
        self.Q4 = 4
        self.OFF_TRACK_TARGET_SPEED = 20

        #Implementar um PI
        self.PI_speed_gain = 0.2

        #FUNÇAO DO PUSH
        self.steeringCmdFilter = uteis.tool()
        self.speedMonitor = uteis.tool()

        #Controlador de velocidade
        self.pid = PID(0.5,0,1, setpoint = 0.0)


        self.WARM_UP = 0
        self.QUALIFYING = 1
        self.RACE = 2
        self.UNKNOWN = 3
        self.stage = stage
        
        self.parser = msgParser.MsgParser()
        
        self.state = carState.CarState()
        
        self.control = carControl.CarControl()
        self.past_accel = 0
    def init(self):
        '''Return init string with rangefinder angles'''
        
        
        self.angles = [ -90.0, -75.0, -60.0, -45.0, -30.0, -20.0, -15.0, -10.0, -5.0, 
                        0.0,
                        5.0,  10.0, 15.0, 20.0, 30.0, 45.0, 60.0, 75.0, 90.0]

        
        return self.parser.stringify({'init': self.angles})
    
    def drive(self, msg):
        self.state.setFromMsg(msg)
        
        #TODO:colocar aqui uma função para ver se o carro esta estavel se quiser

        self.computeSteering()

        self.computeSpeed()


        
        return self.control.toMsg()
    
    def gear(self):
        rpm = self.state.getRpm()
        gear = self.state.getGear()
        
        gear = 1
        if self.state.getSpeed() >40 :
            gear=2
        if self.state.getSpeed()>60 :
            gear=3
        if self.state.getSpeed()>80 :
            gear=4
        if self.state.getSpeed()>130 :
            gear=5
        if self.state.getSpeed()>300 :
            gear=6
        
        self.control.setGear(gear)
    
    def speed(self):
        speed = self.state.getSpeedX()
        accel = self.control.getAccel()
        
        if speed < 150:
            accel += 0.1
            if accel > 1:
                accel = 1.0
        else:
            accel -= 0.1
            if accel < 0:
                accel = 0.0
        
        self.control.setAccel(accel)
            
        


    #A partir daqui algoritmo e sua implementação

    #Steering and TargetAngle

    def computeSteering(self):

        targetAngle = self.computeTargetAngle()             #Obter o angle alvo para o qual queremos ir
        #alpha (a) = angle of longest sensor (... -20, -10, 0, 10, 20, ...)

        speed = self.state.getSpeed()
        if speed == 0:
            self.rawSteeringAngleRad = 0
        else:
            self.rawSteeringAngleRad = -math.atan( (self.PURE_PURSUIT_2L * math.sin(targetAngle)) / (self.PURE_PURSUIT_K * speed))
        self.rawSteeringAngleDeg = self.rawSteeringAngleRad * self.DEG_PER_RAD

        #normalize between[-1,1]
        self.normalizedSteeringAngle = self.state.clamp(self.rawSteeringAngleDeg / self.MAX_STEERING_ANGLE_DEG, -1.0, 1.0)

        

        if self.USE_STEERING_FILTER:
            self.steeringCmdFilter.push(self.normalizedSteeringAngle)
            self.control.setSteer(self.steeringCmdFilter.get())
        else:
            self.control.setSteer(self.normalizedSteeringAngle) # APENSAS ISTO
        
        
        #On straight segments, correct for vehicle drift near edge of track 
        edgeSteeringCorrection = 0
        if self.EDGE_AVOIDANCE_ENABLED and self.state.isOnTrack():
            edgeSteeringCorrection = 0

        if self.state.getTrackPos() > self.EDGE_MAX_TRACK_POS and self.state.getAngle() < 0.005:     #  too far left
            edgeSteeringCorrection = -self.EDGE_STEERING_INPUT
        elif self.state.getTrackPos() < -self.EDGE_MAX_TRACK_POS and self.state.getAngle() > -0.005: #  too far right
            edgeSteeringCorrection = self.EDGE_STEERING_INPUT


        self.control.setSteer(self.control.getSteer() + edgeSteeringCorrection)

    def computeTargetAngle(self):

        targetAngle = self.state.getMaxDistanceAngle() * self.RAD_PER_DEG

        #Fazer isto para o caso do carro sair da pista

        if self.state.isOffTrack():
            targetAngle = self.computeRecoveryTargetAngle()

        return targetAngle

    def computeRecoveryTargetAngle(self):
        
        targetAngle = 0 
        trackPos = self.state.getTrackPos()
        angle = self.state.getAngle()


        quadrant = 0
        if angle >= 0.0 and angle < self.PI_HALF:
            quadrant = self.Q1
        elif angle >= self.PI_HALF:
            quadrant = self.Q2
        elif angle <= -self.PI_HALF:
            quadrant = self.Q3
        else:
            quadrant = self.Q4


        trackSide = self.MIDDLE
        if trackPos > 1.0 :
            trackSide = self.LEFT_SIDE
        elif trackPos < -1.0 :
            trackSide = self.RIGHT_SIDE


        #SWITCH CASE
        if quadrant == self.Q1:
            if trackSide == self.LEFT_SIDE:
                targetAngle = self.PI_FOURTHS - angle
            elif trackSide == self.RIGHT_SIDE:
                targetAngle = -self.PI_FOURTHS
        elif quadrant == self.Q2:
            if trackSide == self.RIGHT_SIDE:
                targetAngle = self.PI_FOURTHS
            else:
                targetAngle = -self.PI_FOURTHS
        elif quadrant == self.Q3:
            if trackSide == self.LEFT_SIDE:
                targetAngle = -self.PI_FOURTHS
            else:
                targetAngle = self.PI_FOURTHS
        elif quadrant == self.Q4:
            if trackSide == self.LEFT_SIDE:
                targetAngle = self.PI_FOURTHS
            elif trackSide == self.RIGHT_SIDE:
                targetAngle = -self.PI_FOURTHS - angle

        return targetAngle


    #Speed target
    def computeSpeed(self):
        accel = 0
        gear = self.state.getGear()
        breakingZone = self.state.getMaxDistance() < self.state.getSpeedX() / 1.5
        targetSpeed = 0
        hasWhellSpin = False

        if self.state.isOnTrack():

            if breakingZone:
                targetSpeed = max(self.DEFAULT_MIN_SPEED, self.state.getMaxDistance()) 
            else:
                targetSpeed = self.DEFAULT_MAX_SPEED

            frontWhellAvgSpeed = (self.state.getWheelSpinVel()[0] + self.state.getWheelSpinVel()[1]) / 2.0
            rearWheelAvgSpeed  = (self.state.getWheelSpinVel()[2] + self.state.getWheelSpinVel()[3]) / 2.0
            if rearWheelAvgSpeed != 0.0:
                slippagePercent    = (frontWhellAvgSpeed/rearWheelAvgSpeed) *100.0
            else:
                slippagePercent = 0.0

            whellSpinDelta = abs(frontWhellAvgSpeed - rearWheelAvgSpeed)

            hasWhellSpin = self.state.getSpeedX() > 5.0 and slippagePercent < 80.0
            if hasWhellSpin:
                accel = self.control.getAccel() - self.WHEELSPIN_ACCEL_DELTA
        else:
            targetSpeed = self.OFF_TRACK_TARGET_SPEED

        if not hasWhellSpin:
            #entrar num PID ou PI
            self.pid.setpoint = targetSpeed
            #accel = ao valor de la
            accel = self.pid(self.state.getSpeed())
            print('accel1', accel)
        if accel > 0 :
            accel = self.state.clamp(accel, 0.0, self.ACCEL_MAX)
            if gear == 0 or self.state.getRpm() > self.RPM_MAX:
                gear = gear + 1
        elif accel < 0:
            accel = self.state.clamp(accel, self.control.getAccel() - self.BRAKING_DELTA, 0.0)
            if self.control.getAccel()==0.0:
               accel = self.past_accel - self.BRAKING_DELTA
               print('fodasswe')
            self.past_accel = accel

            if self.state.getRpm() < self.RPM_MAX * 0.75:
                gear = gear - 1
        #self.control.setBrake(self.state.clamp(gear, 1, self.GEAR_MAX))

        accel = self.state.clamp(accel, self.BRAKING_MAX, self.ACCEL_MAX)
        if accel > 0.0:
            self.control.setAccel(accel)
        else:
            self.control.setAccel(0.0)

        if accel < 0.0:
            self.control.setBrake(abs(accel))
        else:
            self.control.setBrake(0)

        self.control.setGear(gear)

        print('TargerSpeed: ', targetSpeed, 'Acceleration: ', self.control.getAccel(), 'Breaking: ', self.control.getBrake())

























    def onShutDown(self):
        pass
    
    def onRestart(self):
        pass
        