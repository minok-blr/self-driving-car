class AutoDriver:
    def __init__(self, desiredDistance):
        '''
        Inicializacija AutoLander implementacije.
        Tu prejmete informacije o letalniku.

        objectMass - float
            masa vozila v kg

        objectMaxThrustForce - float
            maksimalna sila potiska motorjev v N

        objectNeutralBreakForce - float
            sila v N, s katero vozilo zavira če je moč motorja 0N
        
        desired distance - float
            željena oddaljenost objekta od tarče
        '''
        self.kalman = KalmanFilter(0.02, 0.5)
        self.distanceFilt = 0
        self.desiredDistance = desiredDistance

        self.currentThrust = 0

        # PID variables
        self.Kp = -9.5
        self.Ki = -2.0
        self.Kd = -0.001
        self.proportional = 0
        self.integral = 0
        self.derivative = 0
        self.lastTime = None
        self.lastError = 0.

    def updateMeasurement(self, time, distance):
        '''
        Receipt of individual measurement.
         time - float
             time covers, in s

         distance - float
             measured distance to the object in front of the vehicle in m
        '''

        self.distanceFilt = self.kalman(distance)

        # PID controller
        error = self.desiredDistance - self.distanceFilt
        if self.lastTime == None:
            dt = 1e-16
        else:
            dt = time - self.lastTime  # time difference
        deltaInput = error - self.lastError
        self.proportional = self.Kp * error  # proportional part
        #print(dt)
        self.integral += error * dt  # integral part
        self.derivative = self.Kd * deltaInput / dt  # derivative part
        outputError = self.proportional + self.Ki * self.integral + self.derivative

        self.currentThrust = self.currentThrust * 0.5 + outputError * 0.5
        # self.currentThrust = outputError
        # print(outputError)
        if self.currentThrust > 1:
            self.currentThrust = 1
        elif self.currentThrust < 0:
            self.currentThrust = 0

        self.lastTime = time
        self.lastError = error

        # object properties - do not touch!!!

    @property
    def distanceFiltered(self):
        '''
         Here you return the current distance estimated by the filters.
         This allows later rendering, returning your internal variable.
        '''
        return self.distanceFilt

    @property
    def thrust(self):
        '''
        Here you return the relative amount of engine power, a value between 0 and 1.
        '''
        return self.currentThrust


class KalmanFilter(object):
    def __init__(self, process_variance, estimated_measurement_variance):
        self.process_variance = process_variance
        self.estimated_measurement_variance = estimated_measurement_variance
        self.posteri_estimate = 0.
        self.posteri_error_estimate = 1.0

    def __call__(self, measurement):
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.posteri_error_estimate + self.process_variance

        blending_factor = priori_error_estimate / (priori_error_estimate + self.estimated_measurement_variance)
        self.posteri_estimate = priori_estimate + blending_factor * (measurement - priori_estimate)
        self.posteri_error_estimate = (1 - blending_factor) * priori_error_estimate

        return self.posteri_estimate
