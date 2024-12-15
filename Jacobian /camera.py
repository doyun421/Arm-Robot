import cv2
import numpy as np

class CameraSensor:
    def __init__(self, camera_id=0):
        self.cap = cv2.VideoCapture(camera_id)
    
    def capture_image(self):
        # Capture an image from the camera
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame
    
    def process_image(self, image):
        # Simple image processing: Convert to grayscale
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Example usage
camera = CameraSensor()
image = camera.capture_image()  # Capture an image
processed_image = camera.process_image(image)  # Process the image
