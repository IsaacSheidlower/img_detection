#!/usr/bin/python3

from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
import rospy
import os
from norfair_ros.msg import Detection as DetectionMsg
from norfair_ros.msg import Detections as DetectionsMsg
from norfair_ros.msg import Point
from norfair_ros.msg import BoundingBoxes
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge 
global img_pub

"""norfair detections message:

Header header
Header image_header
Detection[] detections

"""
"""norfair detection message:
int16 id
string label
float64[] scores
Point[] points
"""

""" norfair bounding box message:
float64 probability
int64 xmin
int64 ymin
int64 xmax
int64 ymax
int16 id
string Class
"""
""" Detection2D message:
Header header

ObjectHypothesisWithPose[] results

BoundingBox2D bbox

sensor_msgs/Image source_img

"""

""" ObectHypothesisWithPose message:

int64 id

# The probability or confidence value of the detected object. By convention,
#   this value should lie in the range [0-1].
float64 score

# The 6D pose of the object hypothesis. This pose should be
#   defined as the pose of some fixed reference point on the object, such a
#   the geometric center of the bounding box or the center of mass of the
#   object.
# Note that this pose is not stamped; frame information can be defined by
#   parent messages.
# Also note that different classes predicted for the same input data may have
#   different predicted 6D poses.
geometry_msgs/PoseWithCovariance pose

"""

""" BoundingBox2D message:
geometry_msgs/Pose2D center

# The size (in pixels) of the bounding box surrounding the object relative
#   to the pose of its center.
float64 size_x
float64 size_y
"""

""" pose2D message:
float64 x
float64 y
float64 theta

"""

# define callback function
def callback(detections):
    bridge = CvBridge()
    #print("callback")
    global img_pub
    try:
        img = rospy.wait_for_message("/rgb/image_raw", Image)
    except Exception as e:
        return
    #print(img)
    # convert image to cv2 image
    img = bridge.imgmsg_to_cv2(img_msg=img, desired_encoding="passthrough")

    # print all detections
    # for each detection in detections, draw bounding box on image
    for detection in detections.detections:
        #print(detection)
        # get bounding box
        x_min = int(detection.points[0].point[0])
        y_min = int(detection.points[0].point[1])
        x_max = int(detection.points[1].point[0])
        y_max = int(detection.points[1].point[1])
        #print(x_min, y_min, x_max, y_max)
        # if detection is not a person, skip
        
        # if not "person" in detection.label:
        #     continue
        # draw bounding box on image
        cv2.rectangle(img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        # draw very large ID on image
        cv2.putText(img, str(detection.label) + " " + str(detection.id), (x_min, y_min), cv2.FONT_HERSHEY_SIMPLEX, 5, (0, 0, 255), 3)
    
    # convert image to numpy array
    img = np.asarray(img)
    #print(img)
    # convert image to msg
    img_data = CvBridge().cv2_to_imgmsg(img)
    img_pub.publish(img_data)


# define main function
def main():
    # initialize node
    try: 
        rospy.init_node('norfair_imgs', anonymous=True)
    except rospy.ROSInitException:
        pass

    # subscribe to norfair/output topic
    rospy.Subscriber("/norfair/output", DetectionsMsg, callback)
    global img_pub
    img_pub = rospy.Publisher("/norfair/imgs", Image, queue_size=10)   

    # spin
    rospy.spin()
    #rospy.sleep(.01)

# run main function
if __name__ == '__main__':
    main()
