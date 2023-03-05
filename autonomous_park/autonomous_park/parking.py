import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
import math
import sys
import numpy as np
import threading
import time
import joblib

def predict_action(X):
    model = joblib.load('/knn_modelV1.joblib')
    y_pred = model.predict(X)
    return y_pred


def parkControl(array): #Park yapmak icin uygun alan bulursa 1, bulamazsa 0 dondurur.
    result = 1
    refArray = np.array([3.9391923,3.6008472,3.5089402,3.4278915,3.3676655,3.316912,3.295899,3.2808099,3.26835,3.2576642,3.2515213,3.2564604,3.261841,3.2678766,3.275327,3.2809753,3.2902405,3.3008604,3.3119783,3.3222463,3.3342087,3.34664,3.3581226,3.3715882,3.3867,3.4032164,3.4203718,3.4379115,3.456512,3.4765756,3.4982924,3.5204663,3.5440037,3.5682435,3.5950413,3.6243062,3.65451,3.6851537,3.7150943,3.7475715,3.7847028,3.8233879,3.8656418,3.910041,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9874141,3.9652004,3.9436321,3.9228027,3.902638,3.8831332,3.8643076,3.846099,3.8285232,3.8115757,3.7952049,3.7794647,3.7642624,3.7496514,3.7355883,3.7220688,3.7091024,3.6966548,3.6847334,3.673334,3.6624293,3.652044,3.6421332,3.63272,3.6237824,3.6153183,3.6073296,3.5998018,3.5927346,3.5861273,3.5799668,3.57426,3.5689924,3.5641685,3.559782,3.555831,3.5523138,3.5492272,3.54657,3.544341,3.542538,3.5411606,3.5402067,3.5396771,3.5395715,3.5398889,3.5406299,3.541795,3.5433853,3.5454004,3.5478432,3.550714,3.5540156,3.5577497,3.5619168,3.566523,3.571564,3.5770538,3.58299,3.5893683,3.5962083,3.6035006,3.6112602,3.6194873,3.6281812,3.637362,3.6470098,3.6571677,3.667823,3.678959,3.6906264,3.7027993,3.7155097,3.7287555,3.742533,3.756882,3.771759,3.7872503,3.8033295,3.8199556,3.8372319,3.8551028,3.8736267,3.8927977,3.912608,3.933126,3.9542742,3.9762008,3.998857,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.9,3.8717902,3.8016691,3.6748025,3.6408772,3.6010303,3.5641,3.5282097,3.496596,3.4710298,3.4452395,3.4201927,3.3967516,3.3731039,3.3527517,3.3349102,3.3185427,3.3018794,3.286686,3.271059,3.256772,3.2456973,3.2354188,3.228316,3.219058,3.2103832,3.2071764,3.193593,3.190017,3.2205622,3.2342503,3.2334821,3.2341564,3.2385175,3.250991,3.2582612,3.2764897,3.3087418,3.3595674,3.395867,3.426786,3.524675,3.5446754,3.9] )
    for i in range(len(array)):
        if(refArray[i]*0.9>array[i]):
            result = 0
            break
    return result


def carAlignment(array):
    result = 0
    if (array[236] + array[235] + array[234])/3 < 3.0:
        result = 1
    return result


class ackermannControl(Node):
    def __init__(self):
        super().__init__('Ackermann_Control')
        self.subscription = self.create_subscription(PointStamped,'vehicle/gps',self.gps_callback,10)
        self.publisher = self.create_publisher(AckermannDrive,'cmd_ackermann',10)
        self.subscription = self.create_subscription(LaserScan,'vehicle/lidar_on',self.lidar_on,10)
        self.subscription = self.create_subscription(LaserScan,'vehicle/lidar_sag',self.lidar_sag,10)
        self.flag = 0
        print("[BILGI] Uygun Park Alanı Aranıyor.")
        threading.Thread(target=self.data).start()
    
    def data(self):
        while True:
            if not hasattr(self,'ranges_on') or not hasattr(self,'location') or not hasattr(self,'ranges_sag'):
                time.sleep(0.1)
                continue
            if(self.flag == 0):
                if(parkControl(self.rangesC)==0): #Park Alanı Buluyor.
                    v = 3.0
                    q = 0.0
                else:
                    print("[BILGI] Uygun Park Alanı Bulundu.")
                    print("[BILGI] Arac Park Icin Hizalaniyor.")
                    self.flag = 1
            if(self.flag == 1):
                if(carAlignment(self.rangesC)==0): #Park Alani İcin Hizalaniyor.
                    v = 2.5
                    q = 0.0
                else:
                    print("[BILGI] Arac Hizalandi. Park Etmeye Basliyor.")
                    self.flag = 2
            if(self.flag == 2): #Park etme islemi yapiliyor.
                data = np.column_stack((self.ranges_on, self.ranges_sag, self.location))
                data[data == np.inf] = 4.0
                sonuc = predict_action(data)
                v = sonuc[0][1]*1.2
                q = sonuc[0][0]
                if(v == 0.0 and q == 0.0):
                    print("[BILGI] Arac Park Etmeyi Tamamladi.")
                    sys.exit(0)
            msg = AckermannDrive()
            msg.steering_angle = q
            msg.speed = v
            msg.steering_angle_velocity = math.pi/72
            msg.acceleration = 0.5
            self.publisher.publish(msg)
            time.sleep(0.3)

    def lidar_on(self, msg):
        ranges = msg.ranges
        self.ranges_on = np.array(ranges)
        self.ranges_on = self.ranges_on[192:320:4]
        self.ranges_on = self.ranges_on.reshape((1,32))
        self.ranges_on = self.ranges_on*1.4

    def lidar_sag(self, msg):
        ranges = msg.ranges
        self.ranges_sag = np.array(ranges)
        self.ranges_sag = self.ranges_sag[128:384:4]
        self.ranges_sag = self.ranges_sag.reshape((1,64))
        self.rangesC = np.array(ranges)
        self.rangesC = self.rangesC[0:512:2]
        self.rangesC[self.rangesC == np.inf] = 4.0

    def gps_callback(self, msg):
        err = 0.4
        self.location = [msg.point.x - 0.1 , msg.point.y - err , msg.point.z]
        self.location = np.array(self.location)
        self.location = self.location.reshape((1,3))

def main(args=None):
    rclpy.init(args=args)
    ackermann_control = ackermannControl()
    rclpy.spin(ackermann_control)
    ackermann_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
