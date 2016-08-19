from Tkinter import *
import rospy
from std_msgs.msg import String
import threading


class Client:
    
    def __init__(self):
        
        self.cmd_out = "NONE"
        
        self.tk = Tk()
        self.pub = rospy.Publisher('rover_cmds', String, queue_size=10)
        self.pub_thread = threading.Thread(target=self.pub_thread_main)
        rospy.init_node('RoverClient', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz
        self.frame = Frame(self.tk, width=300, height=300)
        self.tk.bind('<Left>', self.leftKey)
        self.tk.bind('<Right>', self.rightKey)
        self.tk.bind('<Up>', self.upKey)
        self.tk.bind('<Down>', self.downKey)
        self.tk.bind('<KeyRelease-Left>', self.leftKeyRelease)
        self.tk.bind('<KeyRelease-Right>', self.rightKeyRelease)
        self.tk.bind('<KeyRelease-Up>', self.upKeyRelease)
        self.tk.bind('<KeyRelease-Down>', self.downKeyRelease)
        self.tk.bind('<q>', self.qKey)
        self.tk.bind('<w>', self.wKey)
        self.tk.bind('<KeyRelease-q>', self.qKeyRelease)
        self.tk.bind('<KeyRelease-w>', self.wKeyRelease)

        self.frame.pack()
        
    def pub_thread_main(self):
        while not rospy.is_shutdown():
            rospy.loginfo(self.cmd_out)
            self.pub.publish(self.cmd_out)
            self.rate.sleep()

    def start(self):
        self.pub_thread.start()
        self.tk.mainloop()
  
    def leftKey(self, event):
        self.cmd_out = "LEFT"

    def rightKey(self, event):
        self.cmd_out = "RIGHT"

    def upKey(self, event):
        self.cmd_out = "FORWARD"
        
    def downKey(self, event):
        self.cmd_out = "BACKWARD"
        
    def qKey(self, event):
        self.cmd_out = "YAWLEFT"
        
    def wKey(self, event):
        self.cmd_out = "YAWRIGHT"
        
    def leftKeyRelease(self, event):
        self.cmd_out = "NONE"

    def rightKeyRelease(self, event):
        self.cmd_out = "NONE"

    def upKeyRelease(self, event):
        self.cmd_out = "NONE"
        
    def downKeyRelease(self, event):
        self.cmd_out = "NONE"
        
    def qKeyRelease(self, event):
        self.cmd_out = "NONE"
        
    def wKeyRelease(self, event):
        self.cmd_out = "NONE"


if __name__ == "__main__":
    cl = Client()
    cl.start()