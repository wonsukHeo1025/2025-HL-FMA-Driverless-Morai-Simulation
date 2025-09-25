#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os
from datetime import datetime
from sensor_msgs.msg import CompressedImage
import threading
import sys
import select
import termios
import tty

class ImageCaptureNode:
    def __init__(self):
        rospy.init_node('image_capture_node', anonymous=True)
        
        # Subscriber
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)
        
        # Current image buffer
        self.current_image = None
        self.image_lock = threading.Lock()
        
        # Save directory
        self.save_dir = os.path.expanduser("~/traffic_light_images")
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            rospy.loginfo(f"Created directory: {self.save_dir}")
        
        # Image counter
        self.image_counter = 0
        
        # Terminal settings for keyboard input
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("Image Capture Node initialized")
        rospy.loginfo(f"Images will be saved to: {self.save_dir}")
        rospy.loginfo("Press 'q' to capture current image")
        rospy.loginfo("Press 'ESC' or 'Ctrl+C' to exit")
        rospy.loginfo("=" * 50)
        
    def image_callback(self, msg):
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if img_bgr is not None:
                with self.image_lock:
                    self.current_image = img_bgr.copy()
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    
    def save_current_image(self):
        with self.image_lock:
            if self.current_image is None:
                rospy.logwarn("No image available to save")
                return
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"traffic_light_{timestamp}_{self.image_counter:04d}.jpg"
            filepath = os.path.join(self.save_dir, filename)
            
            # Save image
            cv2.imwrite(filepath, self.current_image)
            self.image_counter += 1
            
            rospy.loginfo(f"Saved image: {filename}")
            rospy.loginfo(f"Total images captured: {self.image_counter}")
    
    def check_keyboard(self):
        """Non-blocking keyboard check"""
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            return key
        return None
    
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        
        try:
            while not rospy.is_shutdown():
                key = self.check_keyboard()
                
                if key:
                    if key == 'q' or key == 'Q':
                        self.save_current_image()
                    elif key == '\x1b' or key == '\x03':  # ESC or Ctrl+C
                        rospy.loginfo("Exiting...")
                        break
                    elif key == 's' or key == 'S':
                        # Show current status
                        with self.image_lock:
                            if self.current_image is not None:
                                h, w = self.current_image.shape[:2]
                                rospy.loginfo(f"Current image size: {w}x{h}")
                            else:
                                rospy.loginfo("No image received yet")
                        rospy.loginfo(f"Images saved: {self.image_counter}")
                
                rate.sleep()
                
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            rospy.loginfo(f"Total images captured: {self.image_counter}")
            rospy.loginfo(f"Images saved in: {self.save_dir}")

if __name__ == '__main__':
    try:
        node = ImageCaptureNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received")
    finally:
        # Make sure terminal settings are restored
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.old_settings)
        except:
            pass
