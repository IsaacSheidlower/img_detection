#!/usr/bin/python3

from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
import rospy
import os
from norfair_ros.msg import Detection as DetectionMsg
from norfair_ros.msg import Detections as DetectionsMsg
from norfair_ros.msg import Point
from norfair_ros.msg import BoundingBoxes
from sensor_msgs.msg import Image
from img_detection.msg import ObjectPositionNorfair, ObjectPositionsNorfair
import cv2
import numpy as np
from cv_bridge import CvBridge 

global object_pub

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

    object_array = ObjectPositionsNorfair()
    object_array.header = detections.header
    for detection in detections.detections:
        if not "person" in detection.label:
            continue
        #print(detection)
        # get bounding box
        x_min = int(detection.points[0].point[0])
        y_min = int(detection.points[0].point[1])
        x_max = int(detection.points[1].point[0])
        y_max = int(detection.points[1].point[1])

        # calculate center of bounding box and draw on image
        center_x = int((x_min + x_max) / 2)
        center_y = int((y_min + y_max) / 2)

        # flip x and y to match image coordinates
        center = [center_y, center_x]

        # get depth image
        depth_image = rospy.wait_for_message("/rgb_to_depth/image_raw", Image)
        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")
        depth_image = np.array(depth_image, dtype=np.float32)
        # depth shape (1024, 1024, 4)

        color_image = rospy.wait_for_message("/rgb/image_raw", Image)
        color_image = bridge.imgmsg_to_cv2(color_image, desired_encoding="passthrough")
        color_image = np.array(color_image, dtype=np.uint8)
        # color shape (1536, 2048, 4)

        # translate center to depth image coordinates
        center_depth = [int(center[0] * 1024 / 1536), int(center[1] * 1024 / 2048)]

        # get depth value at center of bounding box
        depth = depth_image[center_depth[0], center_depth[1], 0]
        print(depth)


# define main function
def main():
    # initialize node
    try: 
        rospy.init_node('object_positions', anonymous=True)
    except rospy.ROSInitException:
        pass

    # subscribe to norfair/output topic
    rospy.Subscriber("/norfair/output", DetectionsMsg, callback)
    global img_pub
    img_pub = rospy.Publisher("/norfair/object_positions", ObjectPositionsNorfair, queue_size=10)   

    # spin
    rospy.spin()
    #rospy.sleep(.01)

# run main function
if __name__ == '__main__':
    main()
