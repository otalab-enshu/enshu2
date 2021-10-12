#!/usr/bin/env python3
import rospy
import rospkg
import numpy as np
import cv2
from std_msgs.msg import String
from yolox.yolox_onnx import YoloxONNX
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from enshu_msgs.msg import Bbox, BboxArray


class YoloxNode:
    def __init__(self):
        # Get package path
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("enshu3")

        # Yolox module from https://github.com/Kazuhito00/YOLOX-ONNX-TFLite-Sample
        rospy.loginfo("Setup Yolox nano model")
        # self.targets = ["stop sign", "traffic light", "person"]
        # rospy.loginfo(f"Target labels: {self.targets}")
        self.show_img = False
        self.score_th = 0.3
        input_shape = (416, 416)
        # input_shape = (320, 320)
        self.module = YoloxONNX(
            model_path=pkg_path + "/scripts/model/yolox_nano.onnx",
            input_shape=(416, 416),
            class_score_th=self.score_th,
            nms_th=0.45,
            nms_score_th=0.1,
            with_p6=False,
        )

        # Load coco class label
        with open(pkg_path + "/scripts/coco_classes.txt", "rt") as f:
            self.labels = f.read().rstrip("\n").split("\n")
        self.label2color = {
            lbl: np.random.choice(range(256), size=3).tolist() for lbl in self.labels
        }

        # Ros subscriber
        rospy.loginfo("Subscribe image: /camera/color/image_raw/compressed")
        rospy.Subscriber(
            "/camera/color/image_raw/compressed",
            CompressedImage,
            self.callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        self.pub = rospy.Publisher("/bbox_results", BboxArray, queue_size=1)
        rospy.loginfo("Publishing bounding box results: /bbox_results")

    def callback(self, img_msg):
        # decode image
        img = cv2.imdecode(np.frombuffer(img_msg.data, np.uint8), cv2.IMREAD_COLOR)

        # Inference
        bboxes, scores, class_ids = self.module.inference(img)

        # Setup message
        msg = BboxArray()
        for bbox, score, class_id in zip(bboxes, scores, class_ids):
            if self.score_th > score:
                continue
            # Class label
            label = str(self.labels[int(class_id)])
            # if not label in self.targets:
            #     # rospy.logdebug(f"{label} is ignored. Not in targets.")
            #     continue

            x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
            # Upper left point
            ul = Point(x=x1, y=y1)
            # Bottom right point
            br = Point(x=x2, y=y2)
            # Message
            msg.bbox.append(Bbox(ul=ul, br=br, label=label, score=score))

        self.pub.publish(msg)

        # Visualize results
        if self.show_img:
            for bbox_msg in msg.bbox:
                color = self.label2color[bbox_msg.label]
                img = cv2.rectangle(
                    img,
                    (bbox_msg.ul.x, bbox_msg.ul.y),
                    (bbox_msg.br.x, bbox_msg.br.y),
                    color,
                    thickness=2,
                )
                img = cv2.putText(
                    img,
                    f"{bbox_msg.label}:{bbox_msg.score:.2f}",
                    (bbox_msg.ul.x, bbox_msg.ul.y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    color,
                    thickness=2,
                )

            cv2.imshow("img", img)
            key = cv2.waitKey(5)

            if key == 27:
                rospy.signal_shutdown("Esc key is pressed")

    def start(self):
        # Main loop
        rospy.spin()

        # End the node
        rospy.loginfo("End the node")
        cv2.destroyAllWindows()


rospy.init_node("yolox_node", anonymous=True)
node = YoloxNode()
node.start()
