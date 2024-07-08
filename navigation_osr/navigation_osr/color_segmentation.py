#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from skimage.color import rgb2hsv

class ImageProcessor(Node):
    
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(
            Image, 
            "/camera/color/image_raw", 
            self.image_callback, 
            10
        )
        
        self.bool_pub = self.create_publisher(
            Int8, 
            "/color_dominance", 
            10
        )

        self.repeated_count = 0
        self.color_anterior = None
        self.frame_count = 0


    def auto_white_balance(self, img):
        wb = cv2.xphoto.createSimpleWB()
        corrected_img = wb.balanceWhite(img)
        return corrected_img
    
    def apply_clahe_color(self, img):
        img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)
        l_channel, a_channel, b_channel = cv2.split(img_lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        cl = clahe.apply(l_channel)
        img_clahe = cv2.merge((cl, a_channel, b_channel))
        img_clahe_bgr = cv2.cvtColor(img_clahe, cv2.COLOR_Lab2BGR)
        return img_clahe_bgr
    
    def process_frame(self, frame):
        img_clahe = self.apply_clahe_color(frame)
        adjusted_img = self.auto_white_balance(img_clahe)
        return adjusted_img

    def update_saturation_mask_red(self, image_hsv, saturation_threshold):
        lower_mask1 = (image_hsv[:, :, 0] >= 0) & (image_hsv[:, :, 0] <= 0.1)
        lower_mask2 = (image_hsv[:, :, 0] >= 0.9) & (image_hsv[:, :, 0] <= 1)
        saturation_mask = image_hsv[:, :, 1] > saturation_threshold
        mask = (lower_mask1 | lower_mask2) & saturation_mask
        return mask
    
    def update_saturation_mask_blue(self, image_hsv, saturation_threshold):
        lower_mask1 = (image_hsv[:, :, 0] >= 0.5) & (image_hsv[:, :, 0] <= 0.75)
        saturation_mask = image_hsv[:, :, 1] > saturation_threshold
        mask = lower_mask1 & saturation_mask
        return mask

    def apply_mask(self, image_rgb, mask):
        red = image_rgb[:, :, 0] * mask
        green = image_rgb[:, :, 1] * mask
        blue = image_rgb[:, :, 2] * mask
        return np.dstack((red, green, blue))

    def image_callback(self, msg):
        try:
            if self.repeated_count <= 10:
                self.frame_count += 1
                if self.frame_count % 300 != 0:
                    #start_time = self.get_clock().now()
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

                    # Recortar la imagen para obtener solo la parte central
                    #height, width, _ = cv_image.shape
                    #top = height // 4
                    #bottom = 3 * height // 4
                    #left = width // 4
                    #right = 3 * width // 4
                    #cv_image = cv_image[top:bottom, left:right]

                    # Recortar la imagen solo para banda central
                    width = cv_image.shape[1]
                    left_crop = width // 5
                    right_crop = width - left_crop
                    cv_image = cv_image[:, left_crop:right_crop]
                    
                    processed_frame = self.process_frame(cv_image)
                    image_rgb = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
                    image_hsv = rgb2hsv(image_rgb)
                    
                    mask_red = self.update_saturation_mask_red(image_hsv, 0.3)
                    mask_blue = self.update_saturation_mask_blue(image_hsv, 0.3)
                    
                    # Process red mask
                    image_masked_red = self.apply_mask(image_rgb, mask_red)
                    image_masked_red = cv2.cvtColor(image_masked_red, cv2.COLOR_RGB2BGR)
                    
                    gray_red = cv2.cvtColor(image_masked_red, cv2.COLOR_BGR2GRAY)
                    _, binary_red = cv2.threshold(gray_red, 1, 255, cv2.THRESH_BINARY)
                    
                    mask_final_red = binary_red.astype(bool)
                    
                    # Process blue mask
                    image_masked_blue = self.apply_mask(image_rgb, mask_blue)
                    image_masked_blue = cv2.cvtColor(image_masked_blue, cv2.COLOR_RGB2BGR)
                    
                    gray_blue = cv2.cvtColor(image_masked_blue, cv2.COLOR_BGR2GRAY)
                    _, binary_blue = cv2.threshold(gray_blue, 1, 255, cv2.THRESH_BINARY)
                    
                    mask_final_blue = binary_blue.astype(bool)
                    
                    # Compare the amount of red and blue
                    red_area = np.sum(mask_final_red)
                    blue_area = np.sum(mask_final_blue)
                    total_area = mask_red.size

                    self.get_logger().info(f"Red area{red_area}, blue area {blue_area}, total area {total_area}")

                    
                    color_dominance = Int8()
                    if red_area > blue_area and red_area > 0.15 * total_area:
                        color_dominance.data = 1
                        self.get_logger().info(f"red {self.repeated_count}")
                        if self.color_anterior == color_dominance.data:
                            self.repeated_count += 1
                            self.get_logger().info(f"Dominance rojo {self.repeated_count}")
                            self.color_anterior = color_dominance.data
                        else:
                            self.repeated_count = 1
                            self.color_anterior = color_dominance.data
                    elif blue_area > red_area and blue_area > 0.15 * total_area:
                        color_dominance.data = 2
                        self.get_logger().info(f"blue {self.repeated_count}")
                        if self.color_anterior == color_dominance.data:
                            self.repeated_count += 1
                            self.get_logger().info(f"Dominance azul {self.repeated_count}")
                            self.color_anterior = color_dominance.data
                        else:
                            self.repeated_count = 1
                            self.color_anterior = color_dominance.data

                    if self.repeated_count >= 10:
                        self.bool_pub.publish(color_dominance)
                    
                    #end_time = self.get_clock().now()
                    
                    #delay = (end_time - start_time).nanoseconds / 1e9
                    #self.get_logger().info(f"Processing delay: {delay:.6f} seconds")
                    #self.get_logger().info(f"Dominance: {'red' if color_dominance.data == 1 else 'blue'}")
            else:
                pass
        except CvBridgeError as e:
            self.get_logger().error("CvBridge Error: {0}".format(e))

def main(args=None):
    rclpy.init(args=args)
    processor = ImageProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


"""
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from skimage.color import rgb2hsv

class ImageProcessor(Node):
    
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(
            Image, 
            "/camera/color/image_raw", 
            self.image_callback, 
            10
        )
        
        self.image_pub_red = self.create_publisher(
            Image, 
            "/astra_camera/image_preprocessed_red", 
            10
        )
        
        self.image_pub_blue = self.create_publisher(
            Image, 
            "/astra_camera/image_preprocessed_blue", 
            10
        )

    def auto_white_balance(self, img):
        wb = cv2.xphoto.createSimpleWB()
        corrected_img = wb.balanceWhite(img)
        return corrected_img
    
    def apply_clahe_color(self, img):
        img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)
        l_channel, a_channel, b_channel = cv2.split(img_lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        cl = clahe.apply(l_channel)
        img_clahe = cv2.merge((cl, a_channel, b_channel))
        img_clahe_bgr = cv2.cvtColor(img_clahe, cv2.COLOR_Lab2BGR)
        return img_clahe_bgr
    
    def process_frame(self, frame):
        img_clahe = self.apply_clahe_color(frame)
        adjusted_img = self.auto_white_balance(img_clahe)
        return adjusted_img

    def update_saturation_mask_red(self, image_hsv, saturation_threshold):
        lower_mask1 = (image_hsv[:, :, 0] >= 0) & (image_hsv[:, :, 0] <= 0.1)
        lower_mask2 = (image_hsv[:, :, 0] >= 0.9) & (image_hsv[:, :, 0] <= 1)
        saturation_mask = image_hsv[:, :, 1] > saturation_threshold
        mask = (lower_mask1 | lower_mask2) & saturation_mask
        return mask
    
    def update_saturation_mask_blue(self, image_hsv, saturation_threshold):
        lower_mask1 = (image_hsv[:, :, 0] >= 0.5) & (image_hsv[:, :, 0] <= 0.75)
        saturation_mask = image_hsv[:, :, 1] > saturation_threshold
        mask = lower_mask1 & saturation_mask
        return mask

    def apply_mask(self, image_rgb, mask):
        red = image_rgb[:, :, 0] * mask
        green = image_rgb[:, :, 1] * mask
        blue = image_rgb[:, :, 2] * mask
        return np.dstack((red, green, blue))

    def SE(self, size):
        return np.ones((size, size), dtype='uint8')

    def image_callback(self, msg):
        try:
            start_time = self.get_clock().now()
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            processed_frame = self.process_frame(cv_image)
            image_rgb = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
            image_hsv = rgb2hsv(image_rgb)
            
            mask_red = self.update_saturation_mask_red(image_hsv, 0.3)
            mask_blue = self.update_saturation_mask_blue(image_hsv, 0.3)
            
            # Process and publish red mask
            image_masked_red = self.apply_mask(image_rgb, mask_red)
            image_masked_red = cv2.cvtColor(image_masked_red, cv2.COLOR_RGB2BGR)
            
            gray_red = cv2.cvtColor(image_masked_red, cv2.COLOR_BGR2GRAY)
            _, binary_red = cv2.threshold(gray_red, 1, 255, cv2.THRESH_BINARY)
            
            eroded_binary_red = cv2.erode(binary_red, self.SE(30), iterations=1)
            dilated_binary_red = cv2.dilate(eroded_binary_red, self.SE(50), iterations=1)
            
            mask_final_red = dilated_binary_red.astype(bool)
            resulted_image_red = self.apply_mask(image_rgb, mask_final_red)
            resulted_image_red = cv2.cvtColor(resulted_image_red, cv2.COLOR_RGB2BGR)
            
            self.image_pub_red.publish(self.bridge.cv2_to_imgmsg(resulted_image_red, "bgr8"))
            
            # Process and publish blue mask
            image_masked_blue = self.apply_mask(image_rgb, mask_blue)
            image_masked_blue = cv2.cvtColor(image_masked_blue, cv2.COLOR_RGB2BGR)
            
            gray_blue = cv2.cvtColor(image_masked_blue, cv2.COLOR_BGR2GRAY)
            _, binary_blue = cv2.threshold(gray_blue, 1, 255, cv2.THRESH_BINARY)
            
            eroded_binary_blue = cv2.erode(binary_blue, self.SE(30), iterations=1)
            dilated_binary_blue = cv2.dilate(eroded_binary_blue, self.SE(50), iterations=1)
            
            mask_final_blue = dilated_binary_blue.astype(bool)
            resulted_image_blue = self.apply_mask(image_rgb, mask_final_blue)
            resulted_image_blue = cv2.cvtColor(resulted_image_blue, cv2.COLOR_RGB2BGR)
            
            self.image_pub_blue.publish(self.bridge.cv2_to_imgmsg(resulted_image_blue, "bgr8"))
            
            end_time = self.get_clock().now()
            
            delay = (end_time - start_time).nanoseconds / 1e9
            self.get_logger().info(f"Processing delay: {delay:.6f} seconds")
        except CvBridgeError as e:
            self.get_logger().error("CvBridge Error: {0}".format(e))

def main(args=None):
    rclpy.init(args=args)
    processor = ImageProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""

