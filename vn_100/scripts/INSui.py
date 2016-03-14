#!/usr/bin/env python
from INSdatasource import Ui_MainWindow
import signal
import sys
from PyQt4 import QtGui,QtCore
import rospy
from vn_100.msg import *
class INSui(QtGui.QMainWindow):
    inscalback=QtCore.pyqtSignal(list,list)
    sensorcalback=QtCore.pyqtSignal(list,list,list,float,float)
    def __init__(self,parent=None):
        QtGui.QWidget.__init__(self,parent)
        self.ui=Ui_MainWindow()
        self.ui.setupUi(self)
        self.pubsub()
        self.signalnslots()
    def pubsub(self):
        self.insdatasub=rospy.Subscriber("/vn_100/ins_data",ins_data,self.insdatacallback)
        self.sensordatapub=rospy.Subscriber("/vn_100/sensor_data",sensor_data,self.sensordatacallback)
    def signalnslots(self):
        self.inscalback.connect(self.inscalbackprint)
        self.sensorcalback.connect(self.sensorcalbackprint)
    def insdatacallback(self,msg):
        #rospy.loginfo(msg.YPR)
        self.list1=[msg.YPR.x,msg.YPR.y,msg.YPR.z]
        #rospy.loginfo(self.list1)
        self.list2=[msg.quat_data[0],msg.quat_data[1],msg.quat_data[2],msg.quat_data[3]]
        self.inscalback.emit(self.list1,self.list2)
    def sensordatacallback(self,msg):
        self.list3=[msg.Mag.x,msg.Mag.y,msg.Mag.z]
        self.list4=[msg.Accel.x,msg.Accel.y,msg.Accel.z]
        self.list5=[msg.Gyro.x,msg.Gyro.y,msg.Gyro.z]
        self.sensorcalback.emit(self.list3,self.list4,self.list5,msg.Temp,msg.Pressure)
    @QtCore.pyqtSlot(list,list,list,float,float)
    def sensorcalbackprint(self,mag,accel,gyro,temp,pressure):
        self.ui.Pressure.setText(str(pressure))
        self.ui.Temperature.setText(str(temp))
        self.ui.Accelerationdata.setText("Accel.x:"+str(accel[0]))
        self.ui.Accelerationdata.append("Accel.y:"+str(accel[1]))
        self.ui.Accelerationdata.append("Accel.z:"+str(accel[2]))
        self.ui.Gyrodata.setText("angular rate.x:"+str(gyro[0]))
        self.ui.Gyrodata.append("angular rate.y:"+str(gyro[1]))
        self.ui.Gyrodata.append("angular rate.z:"+str(gyro[2]))
        self.ui.Magnetometerdata.setText("Magnetometer.x:"+str(mag[0]))
        self.ui.Magnetometerdata.append("Magnetometer.y:"+str(mag[1]))
        self.ui.Magnetometerdata.append("Magnetometer.z:"+str(mag[2]))
    @QtCore.pyqtSlot(list,list)
    def inscalbackprint(self,ypr,quat_data):
        self.ui.Yaw.setText(str(ypr[0]))
        self.ui.Pitch.setText(str(ypr[1]))
        self.ui.Roll.setText(str(ypr[2]))
        self.ui.Quaterniondata.setText("quaternion.x:"+str(quat_data[0]))
        self.ui.Quaterniondata.append("quaternion.y"+str(quat_data[1]))
        self.ui.Quaterniondata.append("quaternion.z"+str(quat_data[2]))
        self.ui.Quaterniondata.append("quaternion.a"+str(quat_data[3]))
def main():
    rospy.init_node("INSui")
    signal.signal(signal.SIGINT,signal.SIG_DFL)
    app=QtGui.QApplication(sys.argv)
    insuii=INSui()
    insuii.show()
    sys.exit(app.exec_())

if __name__=="__main__":
    main()
