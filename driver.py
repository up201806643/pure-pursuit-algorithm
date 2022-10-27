'''
Created on Apr 4, 2012

@author: lanquarden
'''

import msgParser
import carState
import carControl
import math
import uteis


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
        self.RPM_MAX = 9500    
        self.ACCEL_MAX = 1.0    
        self.ACCEL_DELTA = 0.5             # maximum rate of change in acceleration signal, avoid spinning out

        self.BRAKING_MAX = -0.5            # braking signal <= BRAKING_MAX 
        self.BRAKING_DELTA = 0.05          # dampen braking to avoid lockup, max rate of chage in braking
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



        self.WARM_UP = 0
        self.QUALIFYING = 1
        self.RACE = 2
        self.UNKNOWN = 3
        self.stage = stage
        
        self.parser = msgParser.MsgParser()
        
        self.state = carState.CarState()
        
        self.control = carControl.CarControl()

    
    def init(self):
        '''Return init string with rangefinder angles'''
        
        
        self.angles = [ -90.0, -75.0, -60.0, -45.0, -30.0, -20.0, -15.0, -10.0, -5.0,     0.0,    5.0,  10.0, 15.0, 20.0, 30.0, 45.0, 60.0, 75.0, 90.0]

        
        return self.parser.stringify({'init': self.angles})
    
    def drive(self, msg):
        self.state.setFromMsg(msg)
        
        #TODO:colocar aqui uma função para ver se o carro esta estavel se quiser

        self.computeSteering()

        
        self.gear()
        
        self.speed()
        
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

    def computeSteering(self):

        targetAngle = self.computeTargetAngle()
        

        #alpha (a) = angle of longest sensor (... -20, -10, 0, 10, 20, ...)
        print("speed", self.state.getSpeed())
        sp = (self.PURE_PURSUIT_K * self.state.getSpeed())

        print("target", targetAngle , "sp", sp)
        if sp == 0:
            self.rawSteeringAngleRad = 0
        else:
            self.rawSteeringAngleRad = -math.atan( (self.PURE_PURSUIT_2L * math.sin(targetAngle)) / sp)
        self.rawSteeringAngleDeg = self.rawSteeringAngleRad * self.DEG_PER_RAD
        print("self", self.rawSteeringAngleDeg)
        #normalize between[-1,1]
        self.normalizedSteeringAngle = self.state.clamp(self.rawSteeringAngleDeg / self.MAX_STEERING_ANGLE_DEG, -1.0, 1.0)
        
        self.steeringCmdFilter.push(self.normalizedSteeringAngle)
        print(self.control.getSteer())
        '''

        if self.USE_STEERING_FILTER:
            self.steeringCmdFilter.push(self.normalizedSteeringAngle)
            self.control.setSteer(self.steeringCmdFilter.get())
        else:
            self.control.setSteer(self.normalizedSteeringAngle) # APENSAS ISTO
        
        
        #On straight segments, correct for vehicle drift near edge of track 
        if self.EDGE_AVOIDANCE_ENABLED and self.state.isOnTrack():
            edgeSteeringCorrection = 0

        if self.state.getTrackPos() > self.EDGE_MAX_TRACK_POS and self.state.getAngle() < 0.005:     #  too far left
            edgeSteeringCorrection = -self.EDGE_STEERING_INPUT
        elif self.state.getTrackPos() < -self.EDGE_MAX_TRACK_POS and self.state.getAngle() > -0.005: #  too far right
            edgeSteeringCorrection = self.EDGE_STEERING_INPUT


        self.control.setSteer(self.control.getSteer() + edgeSteeringCorrection)


        print(self.control.getSteer())
        '''


    def computeTargetAngle(self):

        targetAngle = self.state.getMaxDistanceAngle() * self.RAD_PER_DEG

        #Fazer isto para o caso do carro sair da pista

        #if (sensors.isOffTrack()) {
        #    targetAngle = this.computeRecoveryTargetAngle(sensors);

        return targetAngle




































    def onShutDown(self):
        pass
    
    def onRestart(self):
        pass
        