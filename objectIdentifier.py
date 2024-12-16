#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32, Header, Bool
import cv2
import time
import sys
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker, MarkerArray

INPUT_WIDTH = 640
INPUT_HEIGHT = 640
NMS_THRESHOLD = 0.4
SCORE_THRESHOLD = 0.4
CONFIDENCE_THRESHOLD = 0.5

ID_FOTO = 0
capture = Image()
bridge = CvBridge()

colors = [(255, 255, 0), (0, 255, 0), (0, 255, 255), (255, 0, 0)]

is_cuda = len(sys.argv) > 1 and sys.argv[1] == "cuda"


#start = time.time_ns()
frame_count = 0
total_frames = 0
fps = -1

def build_model(is_cuda):
    #print("CUDA COOL 1")
    model_path = '/home/puzzlebot/catkin_ws/src/plaga/plantas.onnx'
    net = cv2.dnn.readNetFromONNX(model_path)
    #print("CUDA COOL 2")
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
    #print("CUDA COOL 3")
    return net

net = build_model(is_cuda)

def detect(image, net):
    blob = cv2.dnn.blobFromImage(image, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
    net.setInput(blob)
    preds = net.forward()
    return preds

def callback(msg):
	global capture
	capture = msg
        
def load_classes():
    class_list = ['Beans_Angular_LeafSpot', 'Beans_Rust', 'Strawberry_Angular_LeafSpot', 'Strawberry_Anthracnose_Fruit_Rot', 'Strawberry_Blossom_Blight', 'Strawberry_Gray_Mold', 'Straberry_Leaf_Spot', 'Straberry_Powdery_Mildew_Fruit', 'Straberry_Powdery_Mildew_Leaf', 'Tomato_Blight', 'Tomato_Leaf_Mold','Tomato_Spider_Mites']
   
    return class_list

class_list = load_classes()

def wrap_detection(input_image, output_data):
    class_ids = []
    confidences = []
    boxes = []

    rows = output_data.shape[0]

    image_width, image_height, _ = input_image.shape

    x_factor = image_width / INPUT_WIDTH
    y_factor =  image_height / INPUT_HEIGHT

    for r in range(rows):
        row = output_data[r]
        confidence = row[4]
        if confidence >= CONFIDENCE_THRESHOLD:

            classes_scores = row[5:]
            _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
            class_id = max_indx[1]
            if (classes_scores[class_id] > SCORE_THRESHOLD):

                confidences.append(confidence)

                class_ids.append(class_id)

                x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                left = int((x - 0.5 * w) * x_factor)
                top = int((y - 0.5 * h) * y_factor)
                width = int(w * x_factor)
                height = int(h * y_factor)
                box = np.array([left, top, width, height])
                boxes.append(box)

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, SCORE_THRESHOLD, CONFIDENCE_THRESHOLD) 

    result_class_ids = []
    result_confidences = []
    result_boxes = []

    for i in indexes:
        result_confidences.append(confidences[i])
        result_class_ids.append(class_ids[i])
        result_boxes.append(boxes[i])

    return result_class_ids, result_confidences, result_boxes

def format_yolov5(frame):

    row, col, _ = frame.shape
    _max = max(col, row)
    result = np.zeros((_max, _max, 3), np.uint8)
    result[0:row, 0:col] = frame
    return result

def odom_callback(msg):
    global pos
    pos_x = msg.pose.pose.position.x
    pos_y = msg.pose.pose.position.y

    # pos_x = np.random.randint(1, 10)
    # pos_y = np.random.randint(1, 10)
    
    pos =  (pos_x, pos_y)

def create_marker(marker_id, x, y):
    marker = Marker()
    marker.header = Header()
    marker.header.frame_id = "map"     #desired frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = "marker_array"
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = -x
    marker.pose.position.y = -y
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.65
    marker.color.b = 0.0

    return marker

def key_callback( msg):
    global obstacleAvoidance_key
    obstacleAvoidance_key = msg.data

if __name__ == '__main__':
    rospy.init_node('objectIdentifier')
    print("Object Identifier Initialized")

    objectIdentifier_key = True

    rospy.Subscriber("/img", Image, callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber('/objectIdentifier_key', Bool, key_callback)

    pub = rospy.Publisher ('/signals', Int32, queue_size=10)
    markerArray_pub = rospy.Publisher('/marker_array_topic', Marker, queue_size=10)


    rate=rospy.Rate(10)

    markerId = 0 
    
    while (not rospy.is_shutdown()) and (objectIdentifier_key): 
        try:       
            class_ids = []
            confidences =[]
            box=[]
            classid=10
        
            frame = bridge.imgmsg_to_cv2(capture, desired_encoding='passthrough')

            inputImage = format_yolov5(frame)
            outs = detect(inputImage, net)
        
            class_ids, confidences, boxes = wrap_detection(inputImage, outs[0])
        
            frame_count += 1
            total_frames += 1
            if len(class_ids)> 0: 
                for (classid, confidence, box) in zip(class_ids, confidences, boxes):
                    color = colors[int(classid) % len(colors)]
                    cv2.rectangle(frame, box, color, 2)
                    cv2.rectangle(frame, (box[0], box[1] - 20), (box[0] + box[2], box[1]), color, -1)
                    cv2.putText(frame, class_list[classid], (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0))
                    ID_FOTO = class_list[classid]
                    print(class_list[classid])# cambie ID_FOTO
                    print
                marker = create_marker(markerId, pos[0], pos[1])
                markerArray_pub.publish(marker)
                markerId += 1
                rospy.sleep(5)
        
            if fps > 0:
                fps_label = "FPS: %.2f" % fps
                cv2.putText(frame, fps_label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # cv2.imshow("output", frame)

            pub.publish(classid)
            rospy.sleep(0.1)
    
            if cv2.waitKey(1) > -1:
                print("finished by user")
                break
        except CvBridgeError as e:
            pass
            # print(e)
