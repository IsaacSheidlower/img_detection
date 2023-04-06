#!/usr/bin/python3

from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
import rospy
import os
from norfair_ros.msg import Detection as DetectionMsg
from norfair_ros.msg import Detections as DetectionsMsg
from norfair_ros.msg import Point
from norfair_ros.msg import BoundingBoxes

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


# define global nofair_input publisher
global norfair_input_pub 

# define path to object id text file from /home/aabl-guest/catkin_ws/src/yolov7-ros/class_labels/coco.txt
path = "/home/aabl-guest/catkin_ws/src/yolov7-ros/class_labels/coco.txt"

# define a function to read the object id text file
def read_object_id(id):
    file = open(path, "r")
    lines = file.readlines()
    #print(lines[0])
    return lines[id]
def convert_detection2dArray_to_detections_msg(detection2darray):
    # create a norfair detections message
    detections_msg = DetectionsMsg()
    # create an empty list to store the norfair detection messages
    detections = []
    # iterate over the detection2darray message
    for detection2d in detection2darray.detections:
        # get the object id
        id = detection2d.results[0].id
        # get the object label
        label = read_object_id(id)
        # get the object probability
        probability = detection2d.results[0].score
        # get the object bounding box
        bbox = detection2d.bbox
        # get the object bounding box center
        center = bbox.center
        # get the object bounding box size
        size_x = bbox.size_x
        size_y = bbox.size_y
        # get the object bounding box corners
        xmin = center.x - size_x/2
        ymin = center.y - size_y/2
        xmax = center.x + size_x/2
        ymax = center.y + size_y/2
        # create a norfair detection message
        detection = DetectionMsg(
            id=id,
            label=label,
            scores=[probability, probability],
            points=[
                Point([xmin, ymin]),
                Point([xmax, ymax]),
            ],
        )
        # append the norfair detection message to the detections list
        detections.append(detection)
    
    # make detections_msg.detections equal to the detections list
    detections_msg.detections = detections
    # return the norfair detections message
    return detections_msg


# subscribe to the /yolov7/yolov7 topic
# and print the message to the console
def callback(msg):
    global norfair_input_pub
    # print the message to the console
    # print(msg)
    # convert the message to norfair format
    norfair_input = convert_detection2dArray_to_detections_msg(msg)
    # publish the norfair message to the /norfair_input topic
    norfair_input_pub.publish(norfair_input)

# define main function
def main():
    # initialize the node
    try:
        rospy.init_node('yolov7_testing_node', anonymous=True)
    except rospy.ROSInitException:
        pass
    # create a subscriber
    rospy.Subscriber('/yolov7/yolov7', Detection2DArray, callback)
    global norfair_input_pub
    # create a publisher
    norfair_input_pub = rospy.Publisher('/norfair/input', DetectionsMsg, queue_size=1, latch=True)
    print("yolov7_testing_node is running")
    # spin
    rospy.spin()

# call main function
if __name__ == '__main__':
    main()