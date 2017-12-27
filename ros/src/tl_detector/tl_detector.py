#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3
QUEUE_SIZE = 1
LARGE = 1.e10
LOOKAHEAD_WPS = 100

def euc_dist(x1,y1,x2,y2):
    x = (x1-x2)
    y = (y1-y2)
    return math.sqrt(x*x+y*y)


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        self.light = None
        self.last_pose = None
        self.last_light_wp = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=QUEUE_SIZE)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=QUEUE_SIZE)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=QUEUE_SIZE)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=QUEUE_SIZE)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()

        self.on_sim = True if rospy.get_param('~on_sim') == 1 else False
        self.light_classifier = TLClassifier(self.on_sim)
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        pos_x = pose.position.x
        pos_y = pose.position.y

        # iterate over waypoints to find the next one (i.e. closest in front)
        min_dist = LARGE
        min_idx = 0
        for idx, wp in enumerate(self.waypoints):
            wp_x = wp.pose.pose.position.x
            wp_y = wp.pose.pose.position.y
            if euc_dist(wp_x, wp_y, pos_x, pos_y) < min_dist:
                min_dist = euc_dist(wp_x, wp_y, pos_x, pos_y)
                min_idx = idx

        return min_idx


    def get_closest_light(self, car_pose, light_waypoints):
        """Identifies the closest position between the given car position and traffic light waypoints
        """


        min_idx = LARGE

        for pos_x, pos_y in light_waypoints:
            light_pose = Pose()
            light_pose.position.x = pos_x
            light_pose.position.y = pos_y
            light_wp = self.get_closest_waypoint(light_pose)
            # Check if the wp (idx) is ahead of car position
            # and the wp is closer than the min_idx
            if (light_wp >= car_pose and light_wp < min_idx):
                min_idx = light_wp

        return min_idx


    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False


        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # img_time = self.camera_image.header.stamp
        # img_age = rospy.Time.now() - img_time
        # rospy.loginfo('(%s), (%s)', rospy.Time.now().secs, self.camera_image.header.stamp.secs)

        #if (img_age.secs > 0.2):
            # pass
            # rospy.loginfo('Image age (%s) is greater than threhsold (%s)', img_age.secs, 0.2)
            # return -1, TrafficLight.UNKNOWN


        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        # Don't update when car is not moving
        if(self.pose and self.waypoints):
            if (self.last_pose and self.pose.pose != self.last_pose.pose):
                car_wp = self.get_closest_waypoint(self.pose.pose)
                light_wp = self.get_closest_light(car_wp, stop_line_positions)

                # rospy.loginfo("Car position: (%s) Closest waypoint idx: (%s) Closest light waypoint idx: (%s)",
                #     self.pose.pose.position, car_wp, light_wp)

                # If the next traffic light is 100 waypoints ahead of current car position
                self.light = light_wp - car_wp <= LOOKAHEAD_WPS
            else:
                light_wp = self.last_light_wp

        self.last_pose = self.pose
        self.last_light_wp = light_wp

        #TODO find the closest visible traffic light (if one exists)
        if self.light:
            state = self.get_light_state(self.light)
            #rospy.loginfo("CLASSIFIER Light state near waypoint (%s) is (%s)", light_wp, state)
            return light_wp, state


        # self.waypoints = None
        return None, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
