import numpy as np
import matplotlib.pyplot as plt

## Noise generator - do not touch!!!
def awgn_noise_single_element(x, SNR_dB):
    #np.random.seed(1)   # set the random generator seed to same value (for comparison only)
    L = x.size
    SNR = np.power(10, (SNR_dB/10))     # SNR to linear scale
    Esym = np.sum(np.abs(x)**2) / (L)   # Calculate actual symbol energy
    N0 = Esym / SNR                     # Find the noise spectral density
    if np.isreal(x).all():
        noiseSigma = np.sqrt(N0)        # Standard deviation for AWGN Noise when x is real
        n = noiseSigma * np.random.randn(1, L)
    else:
        noiseSigma = np.sqrt(N0/2)      # Standard deviation for AWGN Noise when x is complex
        n = noiseSigma * (np.random.randn(1, L) + complex(0, 1) * np.random.randn(1, L))

    return n[-1, -1]

class FollowSim():
    '''
    Ustvarimo nov simulator.

    AutoDriverClass - razred AutoDriver (ne objekt!), v katerem ste implementirali
    lastno implementacijo pristajanja.

    Parametri:
    AutoDriverClass - razred, ki implemetira zahtevan vmesnik
    timeDuration - čas trajanja simulacije
    timeStep - korak simulacije
    snr_dB - izbrana snr dB vrednost za šum, lahko ekperimentiramo
    objectMass - float, masa objekta v kg
    objectMaxThrust - maksimalni potisk motorjev, v N
    startDistance - začetna oddaljenost od tarče v m
    startVelocity - začetna hitrost v m/s (od 0 dalje)
    startThrust - začetna moč motorja (normalizirano 0-1)
    targetVelocitiTimePoints - časovne značke v sekundah, kdaj naj tarča spremeni hitrost
    targetVelocityThrustPoints - hitrost v m/s premikanje tarče pri ujemajoči časovni znački
    '''
    def __init__(self,
                 AutoDriverClass,
                 animate=False,             # animate the plots
                 timeDuration=10,           # seconds
                 timeStep=0.001,            # seconds
                 snr_dB=30.0,               # power of signal over noise
                 objectMass=0.4,            # kg
                 objectMaxThrust=65.0,      # F (force of engines)
                 objectStaticFriction=0.15, # percentage of objects downwards force that constantly atenuates acceleration
                 objectCOF=0.09,            # percentage of objects forward force that attenuates acceleration)
                 desiredDistance=0.5,       # meters desired distance of object from target
                 startDistance=5,           # meters (initial distance)
                 startVelocity=0,           # meters / seconds (initial velocity)
                 startThrust=0,             # percentage of maxThrust (initial force)
                 targetVelocitiTimePoints=[0, 1, 3, 5, 7, 8],
                 targetVelocityThrustPoints=[0, 0.9, 0.3, 0, 0.5, 0.2]):



        self.AutoDriverClass = AutoDriverClass
        self.autoDriverObj = None
        self.animate = animate

        ## simulation variables
        self.timeDuration = timeDuration    # seconds
        self.timeStep = timeStep            # seconds

        self.g = 9.81           # meters / seconds * seconds (gravitational change of velocity)
        self.snr_dB = snr_dB    # power of signal over noise

        ## object variables

        # properties
        self.objectMass = objectMass
        self.objectMaxThrust = objectMaxThrust
        self.objectNeutralBreakForce = (self.objectMass * self.g) * objectStaticFriction
        self.objectCOF = objectCOF

        # state
        self.startDistance = startDistance  # meters (initial height)
        self.startVelocity = startVelocity  # meters / seconds (initial velocity)
        self.startThrust = startThrust      # percentage of maxThrust (initial force)

        # target
        self.desiredDistance = desiredDistance
        self.targetVelocitiTimePoints = targetVelocitiTimePoints
        self.targetVelocityThrustPoints = targetVelocityThrustPoints

        # history
        self.historyTime = np.arange(self.timeStep,
                                     self.timeDuration+self.timeStep,
                                     self.timeStep)

        self.historyDistance = np.zeros(self.historyTime.shape)
        self.historyDistanceWithNoise = np.zeros(self.historyTime.shape)
        self.historyDistanceWithFilter = np.zeros(self.historyTime.shape)

        self.historyObjectThrust = np.zeros(self.historyTime.shape)
        self.historyObjectForce = np.zeros(self.historyTime.shape)
        self.historyObjectVelocity = np.zeros(self.historyTime.shape)
        self.historyObjectDistance = np.zeros(self.historyTime.shape)

        self.historyTargetVelocity = np.zeros(self.historyTime.shape)
        self.historyTargetDistance = np.zeros(self.historyTime.shape)

        self.historyDesiredDistanceDifference = np.zeros(self.historyTime.shape)

    def run(self):
        '''
        Pozene simulacijo, stopi skozi vse korake 
        in sproti prikazuje rezultate (ce je animate=True).
        '''
        # prepare model
        currentTime = 0
        currentDistance = self.startDistance
        currentDesiredDistanceDifference = self.desiredDistance - currentDistance
        currentObjectVelocity = self.startVelocity
        currentObjectThrust = self.startThrust

        currentObjectDistance = 0
        currentTargetDistance = currentDistance

        currentTargetVelocity = 0

        autoDriverObj = self.AutoDriverClass(self.objectMass,
                                             self.objectMaxThrust,
                                             self.objectNeutralBreakForce,
                                             self.desiredDistance)
        self.autoDriverObj = autoDriverObj

        ts_last_update = 0
        if self.animate:
            self.animation_setup()

        # simulation model loop
        for ts in np.arange(1, self.historyTime.shape[0], 1):
            ## prepare for new loop ...
            currentTime = currentTime + self.timeStep
            previousObjectVelocity = currentObjectVelocity
            #previousTargetVelocity = currentTargetVelocity

            # simulate noise :P
            currentNoise = awgn_noise_single_element(self.historyDistance[0:ts+1], self.snr_dB)
            currentDistance = currentDistance + currentNoise

            autoDriverObj.updateMeasurement(self.historyTime[ts], currentDistance)

            currentDistanceFiltered = autoDriverObj.distanceFiltered
            self.historyDistanceWithFilter[ts] = currentDistanceFiltered
            
            currentObjectThrust = autoDriverObj.thrust
            ## comply with controll restraints
            if currentObjectThrust < 0:
                currentObjectThrust = 0
            if currentObjectThrust > 1:
                currentObjectThrust = 1
            self.historyObjectThrust[ts] = currentObjectThrust

            ## calculate new state ...
            # new force on object = accumulated force - static friction - friction from force + acceleration
            oldForce = (self.objectMass * previousObjectVelocity) / self.timeStep
            currentForce = oldForce \
                            - self.objectNeutralBreakForce \
                            - oldForce * self.objectCOF \
                            + (currentObjectThrust * self.objectMaxThrust)
            self.historyObjectForce[ts] = currentForce
            # new velocity of object =  current force * time / mass
            currentObjectVelocity = (currentForce * self.timeStep) / self.objectMass
            self.historyObjectVelocity[ts] = currentObjectVelocity

            # new displacement of objects

            dPO = currentObjectVelocity * self.timeStep
            currentObjectDistance = currentObjectDistance + dPO

            currentTargetVelocity = self.targetVelocityThrustPoints[len([i for i in self.targetVelocitiTimePoints if i <= currentTime])-1]
            self.historyTargetVelocity[ts] = currentTargetVelocity

            dPT = currentTargetVelocity * self.timeStep
            currentTargetDistance = currentTargetDistance + dPT
            self.historyTargetDistance[ts] = currentTargetDistance
            

            currentDistance = currentTargetDistance - currentObjectDistance
            
            self.historyDistance[ts] = currentDistance
            self.historyDistanceWithNoise[ts] = currentDistance + currentNoise


            currentDesiredDistanceDifference = self.desiredDistance - currentDistance
            self.historyDesiredDistanceDifference[ts] = currentDesiredDistanceDifference


            if self.historyTime[ts]-self.historyTime[ts_last_update] >= 1/30:
                if self.animate:
                    self.animation_update()
                    ts_last_update = ts
    
    def getSimulationResult(self):
        if (np.abs(np.average(self.historyDesiredDistanceDifference)) <= self.desiredDistance * 0.1):
            return True
        else:
            return False


    def printResult(self):
        '''
        Izpise koncne rezultate.
        '''

        print('Simulation results:')
        print('Desired distance:', self.desiredDistance, 'm')
        print('Average distance to target:', np.average(self.historyDistance), 'm')
        print('Average distance error:', np.average(self.historyDesiredDistanceDifference), 's')
        # Simulation is sucessfull if average error is less than 10% of desired distance
        if (np.abs(np.average(self.historyDesiredDistanceDifference)) <= self.desiredDistance * 0.1):
            print('Simulation sucessful :)')
        else:
            print('Simulation failed :(')


    def animation_setup(self):
        '''
        setup animation frame
        '''
        plt.figure()

        self.ax1 = plt.subplot(4, 1, 1)
        plt.title('Distance to target')
        self.heightWithNoiseLine, = plt.plot(self.historyTime,
                                             self.historyDistanceWithNoise,
                                             'r',
                                             label='noise')
        self.heightWithFilterLine, = plt.plot(self.historyTime,
                                              self.historyDistanceWithFilter,
                                              'g',
                                              label='noise - filter')
        self.historyDistanceLine, = plt.plot(self.historyTime,
                                           self.historyDistance,
                                           'b',
                                           label='real')
        plt.xlabel('time (s)'); plt.ylabel('distance (m)')
        plt.legend()

        self.ax2 = plt.subplot(4, 1, 2)
        plt.title('Desired distance difference')
        self.historyDesiredDistanceDifferenceLine, = plt.plot(self.historyTime,
                                                        self.historyDesiredDistanceDifference,
                                                        label='difference')
        plt.xlabel('time (s)'); plt.ylabel('difference (m)')
        plt.legend()

        self.ax3 = plt.subplot(4, 1, 3)
        plt.title('Thrust of object')
        self.historyObjectThrustLine, = plt.plot(self.historyTime,
                                           self.historyObjectThrust,
                                           label='thrust')
        plt.xlabel('time (s)'); plt.ylabel('thrust (%)')
        plt.legend()

        self.ax4 = plt.subplot(4, 1, 4)
        plt.title('Velocity of objects')
        self.historyObjectVelocityLine, = plt.plot(self.historyTime,
                                             self.historyObjectVelocity,
                                             label='object')
        self.historyTargetVelocityLine, = plt.plot(self.historyTime,
                                             self.historyTargetVelocity,
                                             label='target')                                    
        plt.xlabel('time (s)'); plt.ylabel('velocity (m/s)')
        plt.legend()

        plt.subplots_adjust(hspace=1.4)
        plt.subplots_adjust(wspace=1.4)

        plt.show(block=False)
        plt.waitforbuttonpress(0.001)

    def animation_update(self):
        '''
        update animation frame
        '''
        self.heightWithNoiseLine.set_ydata(self.historyDistanceWithNoise)
        self.heightWithFilterLine.set_ydata(self.historyDistanceWithFilter)
        self.historyDistanceLine.set_ydata(self.historyDistance)
        self.ax1.relim()
        self.ax1.autoscale_view()
        self.historyDesiredDistanceDifferenceLine.set_ydata(self.historyDesiredDistanceDifference)
        self.ax2.relim()
        self.ax2.autoscale_view()
        self.historyObjectThrustLine.set_ydata(self.historyObjectThrust)
        self.ax3.relim()
        self.ax3.autoscale_view()
        self.historyObjectVelocityLine.set_ydata(self.historyObjectVelocity)
        self.historyTargetVelocityLine.set_ydata(self.historyTargetVelocity)
        self.ax4.relim()
        self.ax4.autoscale_view()
        plt.draw()
        plt.waitforbuttonpress(0.001)


    def plot(self):
        '''
        plot progress
        '''
        plt.figure()

        plt.subplot(4, 1, 1)
        plt.title('Distance to target')
        plt.plot(self.historyTime[1:-1],
                    self.historyDistanceWithNoise[1:-1],
                    'r',
                    label='noise')
        plt.plot(self.historyTime[1:-1],
                    self.historyDistanceWithFilter[1:-1],
                    'g',
                    label='noise - filter')
        plt.plot(self.historyTime[1:-1],
                    self.historyDistance[1:-1],
                    'b',
                    label='real')
        plt.xlabel('time (s)'); plt.ylabel('distance (m)')
        plt.legend()

        plt.subplot(4, 1, 2)
        plt.title('Desired distance difference')
        plt.plot(self.historyTime[1:-1],
                    self.historyDesiredDistanceDifference[1:-1],
                    label='difference')
        plt.xlabel('time (s)'); plt.ylabel('difference (m)')
        plt.legend()

        plt.subplot(4, 1, 3)
        plt.title('Thrust of object')
        plt.plot(self.historyTime[1:-1],
                    self.historyObjectThrust[1:-1],
                    label='thrust')
        plt.xlabel('time (s)'); plt.ylabel('thrust (%)')
        plt.legend()

        plt.subplot(4, 1, 4)
        plt.title('Velocity of objects')
        plt.plot(self.historyTime[1:-1],
                    self.historyObjectVelocity[1:-1],
                    label='object')
        plt.plot(self.historyTime[1:-1],
                    self.historyTargetVelocity[1:-1],
                    label='target')                                    
        plt.xlabel('time (s)'); plt.ylabel('velocity (m/s)')
        plt.legend()

        plt.subplots_adjust(hspace=1.4)
        plt.subplots_adjust(wspace=1.4)

        plt.show()

from autodriver import AutoDriver

if __name__ == '__main__':
    follow_sim = FollowSim(AutoDriver,
                          animate=False,
                          snr_dB=30.,
                          startDistance=1)

    follow_sim.run()
    follow_sim.printResult()
    follow_sim.plot()
