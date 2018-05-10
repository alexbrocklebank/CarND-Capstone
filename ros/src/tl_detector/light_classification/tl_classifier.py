from styx_msgs.msg import TrafficLight
import tensorflow as tf

import os
import sys
import numpy as np
from functools import partial

THRESHOLD = 0.5
# Reduce unnecessary TensorFlow logging to command line
tf.logging.set_verbosity(tf.logging.ERROR)

class TLClassifier(object):
    def __init__(self):

        # Class member variables
        self.tf_session = None
        self.predict = None
        self.clabels = [4, 0, 1, 2, 4]
        self.readsize = 1024

        # Load classifier model and restore weights
        dirname, filename = os.path.split(os.path.abspath(__file__))
        self.model_path = dirname + '/checkpoint/frozen_inference_graph.pb'


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Set up tensorflow and Traffic Light Classifier
        if self.tf_session is None:
            self.config = tf.ConfigProto(log_device_placement=False)
            self.config.gpu_options.per_process_gpu_memory_fraction = 0.5
            # Terminate after 3 seconds without result
            self.config.operation_timeout_in_ms = 3000
            self.tf_graph = tf.Graph()
            with self.tf_graph.as_default():
                od_graph_def = tf.GraphDef()
                with tf.gfile.GFile(self.model_path, 'rb') as fid:
                    serialized_graph = fid.read()
                    od_graph_def.ParseFromString(serialized_graph)
                    tf.import_graph_def(od_graph_def, name='')
                    self.tf_session = tf.Session(graph=self.tf_graph, config=self.config)
                    # Definite input and output Tensors for self.tf_graph
                    self.image_tensor = self.tf_graph.get_tensor_by_name('image_tensor:0')
                    self.detection_scores = self.tf_graph.get_tensor_by_name('detection_scores:0')
                    self.detection_classes = self.tf_graph.get_tensor_by_name('detection_classes:0')
                    self.num_detections = self.tf_graph.get_tensor_by_name('num_detections:0')
                    self.predict = True

        predict = TrafficLight.UNKNOWN
        if self.predict is not None:
            # Expand dimensions from [1, None, None, 3]
            image_np_expanded = np.expand_dims(image, axis=0)

            # Detection output
            (scores, classes, num) = self.tf_session.run(
                [self.detection_scores, self.detection_classes, self.num_detections],
                feed_dict={self.image_tensor: image_np_expanded})

            # Visualization of the results of a detection.
            scores = np.squeeze(scores)
            classes = np.squeeze(classes).astype(np.int32)

            # Is Prediction confident enough?
            predict = self.clabels[classes[0]]
            confidence = scores[0]
            if confidence is None or confidence < THRESHOLD:
                predict = self.clabels[0]
        return predict
