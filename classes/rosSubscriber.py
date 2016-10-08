import rospy

class RosSubscriber():

    def __init__(self,node,topic,msgType,callBack):
        print "ros"
        rospy.init_node(node)

        self.topic=topic
        self.msgType=msgType
        self.callBack=callBack

        self.listener()

    def listener(self):
        """ Listens to Kinefly Flystate topic"""
        rospy.Subscriber(self.topic, self.msgType, self.callBack)

