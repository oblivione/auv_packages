#!/usr/bin/env python
from thrustercontrols import Ui_MainWindow
import sys
from PyQt4 import QtGui,QtCore
import rospy
import signal
#from gui.msg import *
from std_msgs.msg import String,UInt16
class auvthrusterpanel(QtGui.QMainWindow):
    callbackarrived=QtCore.pyqtSignal(str)
    def __init__(self,parent=None):
        QtGui.QWidget.__init__(self,parent)
        self.ui=Ui_MainWindow()
        self.a=1
        self.b=1
        self.ui.setupUi(self)
        self.pubsNsubsrbs()
        self.connections()
        self.setslidervalues()
        self.signalandslots()
    def pubsNsubsrbs(self):
        self.thrusterstatepub=rospy.Publisher("thrusterstate",UInt16,queue_size=1)
        self.frontpitchspeedpub=rospy.Publisher("frontpitchspeed",UInt16,queue_size=1)
        self.backpitchspeedpub=rospy.Publisher("backpitchspeed",UInt16,queue_size=1)
        self.sideleftsppedpub=rospy.Publisher("sideleftspeed",UInt16,queue_size=1)
        self.siderightspeedpub=rospy.Publisher("siderightspeed",UInt16,queue_size=1)
        self.motherboardstatepub=rospy.Publisher("motherboardstate",UInt16,queue_size=1)
        self.thrusterreversepub=rospy.Publisher("thrusterreverse",UInt16,queue_size=1)
        self.errorsubscribe=rospy.Subscriber("errorboard",String,self.errorcallback)
    def connections(self):
        self.ui.ThrustersInitialize.clicked.connect(self.thrustersinitialize)
        self.ui.ThrustersOn.clicked.connect(self.thrusterson)
        self.ui.Thrustersoff.clicked.connect(self.thrustersoff)
        self.ui.poweroffOBC.clicked.connect(self.poweroffOBC)
        self.ui.OBCon.clicked.connect(self.OBCon)
        self.ui.rfront_pitch.clicked.connect(self.frontpitchrev)
        self.ui.rback_pitch.clicked.connect(self.backpitchrev)
        self.ui.rside_left.clicked.connect(self.sideleftrev)
        self.ui.rside_right.clicked.connect(self.siderightrev)
        self.connect(self.ui.front_pitch_slider,QtCore.SIGNAL("valueChanged(int)"),self.frontpitchspeed)
        self.connect(self.ui.back_pitch_slider,QtCore.SIGNAL("valueChanged(int)"),self.backpitchspeed)
        self.connect(self.ui.side_left_slider,QtCore.SIGNAL("valueChanged(int)"),self.sideleftspeed)
        self.connect(self.ui.side_right_slider,QtCore.SIGNAL("valueChanged(int)"),self.siderightspeed)
    def setslidervalues(self):
        self.ui.front_pitch_slider.setRange(1000,2000)
        self.ui.back_pitch_slider.setRange(1000,2000)
        self.ui.side_left_slider.setRange(1000,2000)
        self.ui.side_right_slider.setRange(1000,2000)
        self.ui.front_pitch_slider.setValue(1000) 
        self.ui.back_pitch_slider.setValue(1000)
        self.ui.side_left_slider.setValue(1500)
        self.ui.side_right_slider.setValue(1500)
    def signalandslots(self):
        self.callbackarrived.connect(self.errorprint)
    @QtCore.pyqtSlot(str)
    def errorprint(self,data):
        self.ui.errorboard.append(data)
    def errorcallback(self,msg):
        rospy.loginfo(msg.data)
        self.callbackarrived.emit(msg.data)
    def thrustersinitialize(self):
        self.thrusterstatepub.publish(1)
        self.a=2
    def thrusterson(self):
        if self.a==2:
            self.thrusterstatepub.publish(2)
            self.a=2
        else:
            QtGui.QMessageBox.critical(self,'Error!!!!!','Please initialize thrusters before switching them on')
    def thrustersoff(self):
        self.thrusterstatepub.publish(3)
    def poweroffOBC(self):
        self.motherboardstatepub.publish(0)
    def OBCon(self):
        self.motherboardstatepub.publish(2)
    def frontpitchrev(self):
        self.thrusterreversepub.publish(1)
    def backpitchrev(self):
        self.thrusterreversepub.publish(2)
    def sideleftrev(self):
        self.thrusterreversepub.publish(3)
    def siderightrev(self):
        self.thrusterreversepub.publish(4)
    def frontpitchspeed(self):
        self.frontpitchspeedpub.publish(self.ui.front_pitch_slider.value())
        self.ui.front_pitch_val.setText(str(self.ui.front_pitch_slider.value()))
        if self.ui.checkBox.isChecked():
            self.backpitchspeedpub.publish(self.ui.front_pitch_slider.value())
            self.ui.back_pitch_val.setText(str(self.ui.front_pitch_slider.value()))
    def backpitchspeed(self):
        self.backpitchspeedpub.publish(self.ui.back_pitch_slider.value())
        self.ui.back_pitch_val.setText(str(self.ui.back_pitch_slider.value()))
        if self.ui.checkBox.isChecked():
            self.frontpitchspeedpub.publish(self.ui.back_pitch_slider.value())
            self.ui.front_pitch_val.setText(str(self.ui.back_pitch_slider.value()))
    def sideleftspeed(self):
        self.sideleftsppedpub.publish(self.ui.side_left_slider.value())
        self.ui.side_left_val.setText(str(self.ui.side_left_slider.value()))
        if self.ui.checkBox.isChecked():
            self.siderightspeedpub.publish(self.ui.side_left_slider.value())
            self.ui.side_right_val.setText(str(self.ui.side_left_slider.value()))
    def siderightspeed(self):
        self.siderightspeedpub.publish(self.ui.side_right_slider.value())
        self.ui.side_right_val.setText(str(self.ui.side_right_slider.value()))
        if self.ui.checkBox.isChecked():
            self.sideleftsppedpub.publish(self.ui.side_right_slider.value())
            self.ui.side_left_val.setText(str(self.ui.side_right_slider.value()))

def main():
    rospy.init_node("thrustercontrolpanel")
    signal.signal(signal.SIGINT,signal.SIG_DFL)
    app=QtGui.QApplication(sys.argv)
    thrusterpanel=auvthrusterpanel()
    thrusterpanel.show()
    sys.exit(app.exec_())
if __name__=="__main__":
    main()

