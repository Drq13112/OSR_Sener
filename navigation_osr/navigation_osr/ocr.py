#!/usr/bin/env python3
import cv2
import numpy as np
import imutils
from pytesseract import image_to_string
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

class OCRNode(Node):
    def __init__(self):
        super().__init__('ocr_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ocr = self.create_publisher(Image, 'processed_frame_ocr', 10)
        self.publisher_num = self.create_publisher(String, "ocr_num" , 10)
        self.br = CvBridge()
        self.ocr_processor = OCRProcessor()
        self.frame_count = 0 

        self.bandera = True
        self.repeated_count = 0
        self.last_digits = None

    def image_callback(self, msg):
        if self.bandera == True:
          self.frame_count += 1  # Increment frame counter

          # Process only 1 out of every 10 frames
          if self.frame_count % 10 != 0:
              return
          #self.get_logger().info('Imagen recibida')
          frame = self.br.imgmsg_to_cv2(msg)

          # Preprocess the image
          preprocessed_image = self.ocr_processor.preprocess_image(frame)
          
          # Publish the processed image
          processed_image_msg = self.br.cv2_to_imgmsg(preprocessed_image, encoding="bgr8")
          self.publisher_ocr.publish(processed_image_msg)

          # Detect number
          digits = self.ocr_processor.detect_number(preprocessed_image)

          if digits:
              self.get_logger().info(f'Dígitos detectados: {digits}')
              if digits == self.last_digits:
                  self.repeated_count += 1

                  # Publica un número y ya no vuelve a hacer nada
                  if self.repeated_count >= 4:
                      #self.get_logger().info(f'Dígito publicado{digits}')
                      self.bandera = False
                      detect = String()
                      detect.data = digits
                      self.publisher_num.publish(detect)

              else:
                  self.repeated_count = 1
              self.last_digits = digits
          #else:
              #self.get_logger().info('No se detectaron dígitos')
        
        else:
            pass


class OCRProcessor:
    def __init__(self):
        pass

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

    def preprocess_image(self, img):
        # Auto White Balance
        awb_image = self.auto_white_balance(img)
        # Apply CLAHE
        clahe_image = self.apply_clahe_color(awb_image)
        # Resize to 300 pixels in height
        resized_image = imutils.resize(clahe_image, height=300)
        
        # Crop 1/5 from the left and right sides
        width = resized_image.shape[1]
        left_crop = width // 5
        right_crop = width - left_crop
        cropped_image = resized_image[:, left_crop:right_crop]

        # Convert white and light colors to black
        lower_bound = np.array([170, 170, 170])  # Lower bound for light colors
        upper_bound = np.array([255, 255, 255])  # Upper bound for light colors
        mask = cv2.inRange(cropped_image, lower_bound, upper_bound)
        cropped_image[mask != 0] = [0, 0, 0]
        
        # Remove everything that is not black
        non_black_mask = cv2.inRange(cropped_image, np.array([1, 1, 1]), np.array([255, 255, 255]))
        cropped_image[non_black_mask != 0] = [255, 255, 255]

        # Apply Gaussian Blur to reduce noise
        blurred_image = cv2.GaussianBlur(cropped_image, (5, 5), 0)

        # Apply morphological operations to clean up the image
        kernel = np.ones((3, 3), np.uint8)
        cleaned_image = cv2.morphologyEx(blurred_image, cv2.MORPH_OPEN, kernel)
        cleaned_image = cv2.morphologyEx(cleaned_image, cv2.MORPH_CLOSE, kernel)

        
        return cropped_image

    def detect_number(self, processed_frame):
        config = '--psm 13 --oem 3 -c tessedit_char_whitelist=0123456789'
        digits = image_to_string(processed_frame, lang='eng', config=config)
        return digits.strip()

def main(args=None):
    rclpy.init(args=args)
    ocr_node = OCRNode()
    rclpy.spin(ocr_node)
    ocr_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
#!/usr/bin/env python3
import cv2
import numpy as np
import imutils
from pytesseract import image_to_string

class OCRProcessor:
    def __init__(self):
        pass

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

    def preprocess_image(self, img):
        # Auto White Balance
        awb_image = self.auto_white_balance(img)
        # Apply CLAHE
        clahe_image = self.apply_clahe_color(awb_image)
        # Resize to 300 pixels in height
        resized_image = imutils.resize(clahe_image, height=300)
        # Convert white and light colors to black
        lower_bound = np.array([170, 170, 170])  # Lower bound for light colors
        upper_bound = np.array([255, 255, 255])  # Upper bound for light colors
        mask = cv2.inRange(resized_image, lower_bound, upper_bound)
        resized_image[mask != 0] = [0, 0, 0]
        # Remove everything that is not black
        non_black_mask = cv2.inRange(resized_image, np.array([1, 1, 1]), np.array([255, 255, 255]))
        resized_image[non_black_mask != 0] = [255, 255, 255]
        return resized_image

    def detect_number(self, processed_frame):
        # Guardar la imagen procesada en un archivo
        cv2.imwrite('processed_image_6.png', processed_frame)
        print('Imagen procesada guardada en processed_image.png')

        config = '--psm 13 --oem 3 -c tessedit_char_whitelist=0123456789'
        digits = image_to_string(processed_frame, lang='eng', config=config)
        return digits.strip()

    def process_image_file(self, file_path):
        frame = cv2.imread(file_path)
        if frame is None:
            print(f"Error: unable to load image from {file_path}")
            return
        
        # Preprocess the image
        preprocessed_image = self.preprocess_image(frame)
        
        # Detect number
        digits = self.detect_number(preprocessed_image)
        print(f'Detected digits: {digits}')

def main():
    file_path = '6.jpeg'  # Ruta al archivo de imagen
    ocr_processor = OCRProcessor()
    ocr_processor.process_image_file(file_path)

if __name__ == '__main__':
    main()
"""

""""
#!/usr/bin/env python3
import pytesseract
from pytesseract import image_to_string
import cv2
import numpy as np
import imutils
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class OCRNode(Node):
    def __init__(self):
        super().__init__('ocr_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()

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

    def preprocess_image(self, img):
        # Auto White Balance
        awb_image = self.auto_white_balance(img)
        # Apply CLAHE
        clahe_image = self.apply_clahe_color(awb_image)
        # Resize to 300 pixels in height
        resized_image = imutils.resize(clahe_image, height=300)
        # Convert to grayscale
        gray_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
        # Binarize the image using Otsu's thresholding
        _, binary_image = cv2.threshold(gray_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # Apply morphological transformations
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        morphed_image = cv2.morphologyEx(binary_image, cv2.MORPH_CLOSE, kernel)
        return morphed_image

    def detect_number(self, processed_frame):
        config = '--psm 13 --oem 3 -c tessedit_char_whitelist=0123456789'
        digits = image_to_string(processed_frame, lang='eng', config=config)
        return digits.strip()

    def image_callback(self, msg):
        self.get_logger().info('Image received')
        frame = self.br.imgmsg_to_cv2(msg)

        # Preprocess the image
        preprocessed_image = self.preprocess_image(frame)
        
        # Detect number
        digits = self.detect_number(preprocessed_image)
        self.get_logger().info(f'Detected digits: {digits}')

def main(args=None):
    rclpy.init(args=args)
    ocr_node = OCRNode()
    rclpy.spin(ocr_node)
    ocr_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


import numpy as np
import cv2
import imutils
from skimage import exposure
from pytesseract import image_to_string

import PIL
import cv2
import rclpy
import rclpy.logging
import rclpy.node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
from std_msgs.msg import Int8,String
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge

class ocr_node(rclpy.node.Node):
  def __init__(self):
    super().__init__('ocr_node1')
    timer_period = 1.0  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)

    self.frame=None

    self.count0=0
    self.frames_update=30
    self.br= CvBridge()


    self.publisher_num = self.create_publisher(String, "ocr_num" , 10)

    self.subscription = self.create_subscription(
      Image,
      'camera/color/image_raw',
      self.image_sub,
      1
      #qos_profile_sensor_data
      )
    self.subscription
    
  def cnvt_edged_image(self,img_arr, should_save=True):
    # ratio = img_arr.shape[0] / 300.0
    image = imutils.resize(img_arr,height=300)
    gray_image = cv2.bilateralFilter(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),11, 17, 17)
    edged_image = cv2.Canny(gray_image, 30, 200)

    return edged_image
  
  def find_display_contour(self,edge_img_arr):
    display_contour = None
    edge_copy = edge_img_arr.copy()
    contours,hierarchy = cv2.findContours(edge_copy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    top_cntrs = sorted(contours, key = cv2.contourArea, reverse = True)[:10]

    for cntr in top_cntrs:
      peri = cv2.arcLength(cntr,True)
      approx = cv2.approxPolyDP(cntr, 0.02 * peri, True)

      if len(approx) == 4:
        display_contour = approx
        break
    return display_contour

  def crop_display(self,image_arr):
    edge_image = self.cnvt_edged_image(image_arr)
    display_contour = self.find_display_contour(edge_image)

    if display_contour is not None:
      cntr_pts = display_contour.reshape(4,2)
    else:
      cntr_pts=None
    return cntr_pts

  def normalize_contrs(self,img,cntr_pts):
    ratio = img.shape[0] / 300.0
    norm_pts = np.zeros((4,2), dtype="float32")

    if cntr_pts is not None:
      s = cntr_pts.sum(axis=1)
      norm_pts[0] = cntr_pts[np.argmin(s)]
      norm_pts[2] = cntr_pts[np.argmax(s)]

      d = np.diff(cntr_pts,axis=1)
      norm_pts[1] = cntr_pts[np.argmin(d)]
      norm_pts[3] = cntr_pts[np.argmax(d)]
    else:
      norm_pts[0]=0
      norm_pts[1]=0
      norm_pts[2]=0
      norm_pts[3]=0

    norm_pts *= ratio

    (top_left, top_right, bottom_right, bottom_left) = norm_pts

    width1 = np.sqrt(((bottom_right[0] - bottom_left[0]) ** 2) + ((bottom_right[1] - bottom_left[1]) ** 2))
    width2 = np.sqrt(((top_right[0] - top_left[0]) ** 2) + ((top_right[1] - top_left[1]) ** 2))
    height1 = np.sqrt(((top_right[0] - bottom_right[0]) ** 2) + ((top_right[1] - bottom_right[1]) ** 2))
    height2 = np.sqrt(((top_left[0] - bottom_left[0]) ** 2) + ((top_left[1] - bottom_left[1]) ** 2))

    max_width = max(int(width1), int(width2))
    max_height = max(int(height1), int(height2))

    dst = np.array([[0,0], [max_width -1, 0],[max_width -1, max_height -1],[0, max_height-1]], dtype="float32")
    persp_matrix = cv2.getPerspectiveTransform(norm_pts,dst)
    return cv2.warpPerspective(img,persp_matrix,(max_width,max_height))

  def process_image(self,orig_image_arr):
    ratio = orig_image_arr.shape[0] / 300.0
    display_image_arr = self.normalize_contrs(orig_image_arr,self.crop_display(orig_image_arr))
    #display image is now segmented.
    gry_disp_arr = cv2.cvtColor(display_image_arr, cv2.COLOR_BGR2GRAY)
    gry_disp_arr = exposure.rescale_intensity(gry_disp_arr, out_range= (0,255))

    #thresholding
    ret, thresh = cv2.threshold(gry_disp_arr,127,255,cv2.THRESH_BINARY)
    return thresh

  def image_sub(self,msg):
    self.count0 += 1
    if self.count0>self.frames_update:
      self.frame=self.br.imgmsg_to_cv2(msg)
      self.count0=0
  
  def timer_callback(self):

    if self.frame is not None:
      frame3=self.process_image(self.frame)
      rgb = cv2.cvtColor(frame3.astype(np.uint8),cv2.COLOR_GRAY2RGB)
      letra=image_to_string(rgb,lang='eng', config='--psm 10 --oem 3 -c tessedit_char_whitelist=0123456789')
      
      clean = letra.strip()
      if clean:  # Verificar si hay texto detectadop
        print(f"Detected number: {clean}")  # Imprimir el número detectado
        detected = String()
        detected.data = clean
        self.publisher_num.publish(detected)
      detected=String()
      detected.data="3"
      self.publisher_num.publish(detected)

      

def main():
  rclpy.init()
  ocr_node1=ocr_node()
  rclpy.spin(ocr_node1)

if __name__ == '__main__':
    main()

#vid.release() 
# Destroy all the windows 
#cv2.destroyAllWindows()
"""