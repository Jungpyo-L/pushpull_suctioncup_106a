#!/usr/bin/env python
import numpy as np
import rospy
from pushpull_suctioncup_106a.msg import SensorPacket, cmdPacket
from scipy import signal
import threading

class P_CallbackHelp(object):
    def __init__(self):
        self.sub = rospy.Subscriber("SensorPacket", SensorPacket, self.callback_P)
        self.START_CMD  = 2
        self.IDLE_CMD   = 3
        self.RECORD_CMD = 10
        self.msg2Sensor = cmdPacket()
        self.P_vac      = -10000.0
        self.sensorCMD_Pub = rospy.Publisher('cmdPacket', cmdPacket, queue_size=10)
        self.callback_Pub  = rospy.Publisher('SensorCallback', SensorPacket, queue_size=10)
        self.callback_Pressure = SensorPacket()
        self.publish_enabled = True
        self.Psensor_Num = 4
        self.BufferLen   = 7
        self.PressureBuffer       = [[0.0]*self.Psensor_Num for _ in range(self.BufferLen)]
        self.PressurePWMBuffer    = np.zeros((int(166/2), self.Psensor_Num))
        self.PressureOffsetBuffer = np.zeros((51, self.Psensor_Num))
        self.P_idx      = 0
        self.PWM_idx    = 0
        self.offset_idx = 0
        self.startPresAvg    = False
        self.startPresPWMAvg = False
        self.offsetMissing   = True
        self.thisPres        = np.zeros(self.Psensor_Num)
        self.four_pressure   = [0.0]*self.Psensor_Num
        self.four_pressurePWM= [0.0]*self.Psensor_Num
        self.PressureOffset  = np.zeros(self.Psensor_Num)
        self.power           = 0.0
        self.samplingF      = 166
        self.FFTbuffer_size = int(self.samplingF / 2)
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
        with self.lock:
            buffer_copy = np.copy(self.PressureBuffer)
        self.PressureOffset = np.mean(buffer_copy, axis=0)

    def callback_P(self, data):
        if not self.publish_enabled or rospy.is_shutdown():
            return
        self.thisPres = np.array(data.data, dtype=float)
        with self.lock:
            self.PressureBuffer[self.P_idx] = self.thisPres - self.PressureOffset
            self.P_idx += 1
            if self.P_idx == len(self.PressureBuffer):
                self.startPresAvg = True
                self.P_idx = 0
        self.PressurePWMBuffer[self.PWM_idx] = self.thisPres - self.PressureOffset
        self.PWM_idx += 1
        if self.PWM_idx == len(self.PressurePWMBuffer):
            self.startPresPWMAvg = True
            self.PWM_idx = 0
        if self.startPresAvg:
            buffer_np = np.array(self.PressureBuffer)
            averagePres = np.mean(buffer_np, axis=0)
            self.four_pressure = averagePres.tolist()
            if self.publish_enabled:
                self.callback_Pressure.ch   = self.Psensor_Num
                self.callback_Pressure.data = self.four_pressure
                self.callback_Pub.publish(self.callback_Pressure)
        if self.startPresPWMAvg:
            averagePresPWM = np.zeros(self.Psensor_Num, dtype=float)
            fs   = self.samplingF
            N    = self.FFTbuffer_size
            fPWM = 30
            for i in range(self.Psensor_Num):
                f, t, Zxx = signal.stft(self.PressurePWMBuffer[:, i], fs, nperseg=N)
                if len(f) > 1:
                    delta_f = f[1] - f[0]
                    idx = int(fPWM / delta_f) if delta_f != 0 else 0
                    idx = min(idx, len(f)-1)
                    power_spectrum = abs(Zxx[idx])
                    mean_power = np.mean(power_spectrum)
                    averagePresPWM[i] = mean_power
            self.four_pressurePWM = averagePresPWM

def shutdown_hook(p_helper):
    p_helper.publish_enabled = False
    p_helper.callback_Pub.unregister()
    p_helper.sensorCMD_Pub.unregister()
    p_helper.sub.unregister()

def main():
    rospy.init_node("pressure_callback_node", anonymous=True)
    p_helper = P_CallbackHelp()
    rospy.on_shutdown(lambda: shutdown_hook(p_helper))
    p_helper.startSampling()
    rospy.spin()

if __name__ == "__main__":
    main()
