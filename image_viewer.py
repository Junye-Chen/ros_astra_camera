# dabai_image_viewer.py
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os

class AstraImageViewer:
    def __init__(self, topic_name="camera/color/image_raw", image_type="rgb"):
        self.bridge = CvBridge()
        self.image_type = image_type.lower()
        self.window_name = f"Astra {image_type.upper()} Viewer"
        
        # 根据类型设置话题前缀
        self.topic_prefix = rospy.get_param("~camera_name", "camera")
        self.topic_map = {
            "rgb": f"{self.topic_prefix}/color/image_raw",
            "depth": f"{self.topic_prefix}/depth/image_raw",
            "ir": f"{self.topic_prefix}/ir/image_raw"
        }
        
        # 初始化窗口
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 640, 480)
        
        # 订阅话题
        rospy.Subscriber(
            self.topic_map[self.image_type],
            Image,
            self.image_callback,
            queue_size=1
        )
        rospy.loginfo(f"Subscribed to: {self.topic_map[self.image_type]}")

    def image_callback(self, msg):
        rospy.loginfo("Image callback triggered.") # [诊断] 确认回调是否被调用
        rospy.loginfo(f"Message encoding: {msg.encoding}, Height: {msg.height}, Width: {msg.width}") # [诊断] 打印消息编码和尺寸

        try:
            cv_image = None # 初始化 cv_image
            if self.image_type == "rgb":
                rospy.loginfo("Attempting to convert to bgr8.")
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            elif self.image_type == "depth":
                try:
                    # 尝试获取16位深度图
                    cv_image_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough") # 或者 "16UC1"
                    if cv_image_depth.dtype == 'uint16':
                        # 示例：归一化到0-255以便显示 (需要根据实际深度范围调整)
                        # 例如，如果最大深度是1米 (10000mm for Kinect-like cameras)
                        max_depth_val = 2000.0 
                        image_max = max(cv_image_depth.flatten())
                        image_min = min(cv_image_depth.flatten())
                        print(f"Depth image range: {image_min} - {image_max}")                     
                        cv_image_display = ((cv_image_depth.astype(float) - image_min) / (image_max - image_min) * 255.0).astype('uint8')
                        # 或者使用OpenCV的normalize函数
                        # cv2.normalize(cv_image_depth, cv_image_depth, 0, 255, cv2.NORM_MINMAX)
                        # cv_image_display = cv_image_depth.astype('uint8')
                        # cv_image_display = (cv_image_depth * 50).astype('uint8') # 简单缩放示例
                    else: # 如果不是预期的深度格式，退回到 mono8
                        rospy.logwarn_once(f"Depth image format is {cv_image_depth.dtype}, falling back to mono8 conversion.")
                        cv_image_display = self.bridge.imgmsg_to_cv2(msg, "mono8")

                    cv_image = cv_image_display # 用于显示的图像
                except CvBridgeError as e:
                    rospy.logerr(f"CV Bridge Error for depth: {e}")
                    return # 出现错误则不继续处理
            else: # depth or ir
                rospy.loginfo(f"Attempting to convert to mono8 for type: {self.image_type}.")
                cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")


            # 判断是否存在self.image_type名称的文件夹，如果不存在则创建
            if not os.path.exists(self.image_type):
                os.makedirs(self.image_type)
            # 保存图像到文件
            cv2.imwrite(f"{self.image_type}/{self.image_type}.png", cv_image) # 保存图像到文件
            
            # if cv_image is not None and cv_image.size > 0:
            #     rospy.loginfo(f"cv_image successfully created. Shape: {cv_image.shape}, dtype: {cv_image.dtype}") # [诊断]
            #     # ! 这里调用窗口显示图像还是没有成功，后续再看看是怎么回事
            #     cv2.imshow(self.window_name, cv_image)
            #     key_pressed = cv2.waitKey(1000) & 0xFF # 等待1毫秒，并获取按键
            #     # if key_pressed == ord('q'): # 如果需要按q退出功能
            #     #     rospy.signal_shutdown("User pressed 'q'")
            # elif cv_image is None:
            #     rospy.logwarn("cv_image is None after conversion.") # [诊断]
            # else: # cv_image.size == 0
            #     rospy.logwarn(f"cv_image is empty (size is 0). Shape: {cv_image.shape}") # [诊断]
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error in image_callback: {e}")
        except Exception as e: # 捕获其他任何可能的异常
            rospy.logerr(f"Unexpected error in image_callback: {e}")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node("astra_image_viewer")
    viewer = AstraImageViewer(
        topic_name=rospy.get_param("~topic", "camera/color/image_raw"),
        # image_type=rospy.get_param("~image_type", "rgb")
        image_type=rospy.get_param("~image_type", "depth")
        # image_type=rospy.get_param("~image_type", "ir")
    )
    viewer.run()
