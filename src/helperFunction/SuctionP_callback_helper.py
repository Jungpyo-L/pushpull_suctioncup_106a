#!/usr/bin/env python
import numpy as np
import rospy
from suction_cup.msg import SensorPacket
from suction_cup.msg import cmdPacket
from scipy import signal
import threading

class P_CallbackHelp(object):
    def __init__(self):        
        rospy.Subscriber("SensorPacket", SensorPacket, self.callback_P)
        
        self.START_CMD = 2
        self.IDLE_CMD = 3
        self.RECORD_CMD = 10
        self.msg2Sensor = cmdPacket()
        self.P_vac = -10000.0
        # self.P_vac = -20000.0

        self.sensorCMD_Pub = rospy.Publisher('cmdPacket', cmdPacket, queue_size=10)

        # callback delay test
        self.callback_Pub = rospy.Publisher('SensorCallback', SensorPacket, queue_size=10)
        self.callback_Pressure = SensorPacket()
        
        ## For pressure feedback
        self.Psensor_Num = 4
        self.BufferLen = 7

        # self.PressureBuffer = [[0.0]*self.Psensor_Num]*self.BufferLen # JP: This may cause a problem
        self.PressureBuffer = [[0.0] * self.Psensor_Num for _ in range(self.BufferLen)] # JP: This is the correct way to initialize a 2D list. This way, each inner list will be independent of each other.
        self.P_idx = 0
        self.startPresAvg = False
        self.four_pressure = [0.0]*self.Psensor_Num
        self.thisPres = 0

        # For FFT
        self.samplingF= 166
        self.FFTbuffer_size = int(self.samplingF/2)    # 166 is 1 second
        self.PressurePWMBuffer = np.array([[0]*self.Psensor_Num]*self.FFTbuffer_size)
        self.PressureOffsetBuffer = np.array([[0]*self.Psensor_Num]*51)
        self.PWM_idx = 0
        self.offset_idx = 0
        self.startPresPWMAvg = False
        self.offsetMissing = True
        self.four_pressurePWM = np.array([0.0]*4)
        self.power = 0
        self.PressureOffset = np.array([0.0]*4)

        # Initize a lock
        self.lock = threading.Lock()
    
    def startSampling(self):
        self.msg2Sensor.cmdInput = self.START_CMD
        self.sensorCMD_Pub.publish(self.msg2Sensor)
    
    def stopSampling(self):
        self.msg2Sensor.cmdInput = self.IDLE_CMD
        self.sensorCMD_Pub.publish(self.msg2Sensor)

    def setNowAsOffset(self):
        self.PressureOffset *= 0
        rospy.sleep(0.5)
        # print("self.PressureBuffer: ", self.PressureBuffer)
        buffer_copy = np.copy(self.PressureBuffer)
        self.PressureOffset = np.mean(buffer_copy, axis=0)

        # # JP: The below code for acquiring lock, just in case the code keep throwing error
        # # Acquire the lock before modifying self.PressureBuffer
        # with self.lock:
        #     buffer_copy = np.copy(self.PressureBuffer)
        #     self.PressureOffset = np.mean(buffer_copy, axis=0)
        

    def callback_P(self, data):
        fs = self.samplingF
        N = self.FFTbuffer_size
        fPWM = 30
        
        # print("self.four_pressurePWM:", np.floor(self.four_pressurePWM))
        # print("self.PressureOffset: ", self.PressureOffset)
        # print("self.PressureOffsetBuffer: ", self.PressureOffsetBuffer)

        # fill in the pressure data ring buffer
        self.thisPres = np.array(data.data)


        self.PressureBuffer[self.P_idx] = self.thisPres - self.PressureOffset 
        self.P_idx += 1

        # # JP: The below code for acquiring lock, just in case the code keep throwing error
        # # Acquire the lock before modifying self.PressureBuffer
        # with self.lock:
        #     self.PressureBuffer[self.P_idx] = self.thisPres - self.PressureOffset 
        #     self.P_idx += 1

        self.PressurePWMBuffer[self.PWM_idx] = self.thisPres - self.PressureOffset 
        self.PWM_idx += 1

        # if buffer is filled, then set average flag to true and reset idx
        if self.P_idx == len(self.PressureBuffer):
            # averagin flag is always true now, i.e. ring buffer
            self.startPresAvg = True
            self.P_idx = 0
        
        # if buffer is filled, then set average flag to true and reset idx
        if self.PWM_idx == len(self.PressurePWMBuffer):
            # averagin flag is always true now, i.e. ring buffer
            self.startPresPWMAvg = True
            self.PWM_idx = 0

        # if averaging flag is True
        if self.startPresAvg:
            averagePres_dummy = [0]*4

            # let each row contribute to its column average
            for pressure in self.PressureBuffer:
                first = averagePres_dummy
                second = [x / len(self.PressureBuffer) for x in pressure]
                final_list = [sum(value) for value in zip(first, second)]
                averagePres_dummy = final_list

            self.four_pressure = averagePres_dummy
            # callback delay check
            self.callback_Pressure.data = averagePres_dummy
            self.callback_Pub.publish(self.callback_Pressure)
        
        # if averaging flag is True
        if self.startPresPWMAvg:
            averagePresPWM_dummy = [0.0]*4

            # run stft to each pressure sensor
            for i in range(4):
                f, t, Zxx = signal.stft(self.PressurePWMBuffer[:,i], fs, nperseg=self.FFTbuffer_size)
                delta_f = f[1]-f[0]
                idx = int(fPWM/delta_f)
                self.power = abs(Zxx[idx])
                mean_power = np.mean(self.power)
                averagePresPWM_dummy[i] = mean_power

            self.four_pressurePWM = averagePresPWM_dummy
        # print("freq: ", f[idx])
        # print("P4PWM: ", [abs(Zxx[idx-1]) , abs(Zxx[idx]), abs(Zxx[idx+1])] )
        # print("abs(Zxx): ", np.mean(abs(Zxx), axis=1))

    def get_P_WENS(self):
            # absolute pressures - P_atm for each sensor
            P0, P1, P2, P3 = self.four_pressure    

            # absolute cardinal direction pressures
            PW = (P3 + P2)/2
            PE = (P1 + P0)/2
            PN = (P1 + P2)/2
            PS = (P0 + P3)/2   

            return PW, PE, PN, PS
            # P0pwm = four_pressurePWM[0]
            # P1pwm = four_pressurePWM[1]
            # P2pwm = four_pressurePWM[2]
            # P3pwm = four_pressurePWM[3]