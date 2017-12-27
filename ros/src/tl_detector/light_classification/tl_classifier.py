from styx_msgs.msg import TrafficLight

import tensorflow as tf
import numpy as np
import sys
import os
import rospy
from utilities import label_map_util


class TLClassifier(object):
    def __init__(self, on_sim=True):
        self.on_sim = on_sim     # True to run on simulator. False to run on Carla
        self.prob_threshold = 0.65  # TL probability threshold
        self.load_graph()


    def load_graph(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        dir_path = os.path.join(dir_path, 'frozen_models')

        # The trained tf model

        # Accurate
        MODEL_NAME = 'frozen_resnet101'

        # Fast
        # MODEL_NAME = 'frozen_ssd_mobilenet'

        MODEL_NAME += ('_sim' if self.on_sim else '_real')
        MODEL_NAME = os.path.join(dir_path, MODEL_NAME)

        # Path to frozen detection graph. This is the actual model that is used for the object detection.
        PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

        # List of the strings that is used to add correct label for each box.
        PATH_TO_LABELS = MODEL_NAME + '/label_map.pbtxt'

        NUM_CLASSES = 14

        ###
        # Load a (frozen) Tensorflow model into memory.
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        ###
        # Loading label map
        # label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        # categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        # category_index = label_map_util.create_category_index(categories)

        # category_index = {
        # category_index[1] = {'id': 1, 'name': TrafficLight.GREEN, 'description': 'Green'}
        # category_index[2] = {'id': 2, 'name': TrafficLight.RED, 'description': 'Red'}
        # category_index[3] = {'id': 3, 'name': TrafficLight.YELLOW, 'description': 'Yellow'}
        # category_index[4] = {'id': 4, 'name': TrafficLight.UNKNOWN, 'description': 'off'}}


        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,
                                                                    use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)
        rospy.loginfo('%s', self.category_index)

        rospy.loginfo('Loaded model: %s', PATH_TO_CKPT)

        with detection_graph.as_default():
            # Definite input and output Tensors for detection_graph
            self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object was detected.
            self.detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = detection_graph.get_tensor_by_name('num_detections:0')

            self.sess = tf.Session(graph=detection_graph)

        self.detection_graph = detection_graph
        # self.category_index = category_index


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        # rospy.loginfo("Go classify")

        image_np = image
        # the array based representation of the image will be used later in order to prepare the
        # result image with boxes and labels on it.
        # image_np = load_image_into_numpy_array(image)
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)
        # Actual detection.
        (boxes, scores, classes, num) = self.sess.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_np_expanded})

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        min_score_thresh = self.prob_threshold
        for i in range(boxes.shape[0]):
            if scores is None or scores[i] > min_score_thresh:
                class_name = self.category_index[classes[i]]['name']
                rospy.loginfo("%s, %f", class_name, scores[i])

                if class_name == 'Green':
                    return TrafficLight.GREEN
                elif class_name == 'Red':
                    return TrafficLight.RED
                elif class_name == 'Yellow':
                    return TrafficLight.YELLOW

        # uint8 UNKNOWN=4
        # uint8 GREEN=2
        # uint8 YELLOW=1
        # uint8 RED=0
        return TrafficLight.UNKNOWN
