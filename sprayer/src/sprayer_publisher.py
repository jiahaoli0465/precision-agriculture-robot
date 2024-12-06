import rospy
import time
from std_msgs.msg import String
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Point
from leaf_detection import LeafDetectionModel
from PIL import Image


class Sprayer:
    def __init__(self):
        rospy.init_node('robot_node', anonymous=True)
        self.model = LeafDetectionModel()

    def test_with_stored_image(self, image_path):
        """Test the model with a stored image."""
        try:
            # Load the image using OpenCV
            cv_image = cv2.imread(image_path)
            if cv_image is None:
                rospy.logerr(f"Failed to load image from {image_path}")
                return

            # Convert OpenCV image to RGB format
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Convert to PIL Image
            pil_image = Image.fromarray(cv_image)

            # Perform leaf detection
            boxes, classes, confidences = self.model.predict(pil_image)
            rospy.loginfo(f"Detections: {len(boxes)}")
            # print(results[0].boxes)
            # render = render_result(model=model, image=image, result=results[0]) 
            # render.show()
            # # Annotate image
            for box, cls, conf in zip(boxes, classes, confidences):
                x1, y1, x2, y2 = map(int, box)
                label = f"Class {cls}: {conf:.2f}"
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            # Display the annotated image
            # cv2.imshow("Test Image", cv_image)
            # cv2.waitKey(0)  # Wait indefinitely until a key is pressed
            # cv2.destroyAllWindows()
            output_path = "annotated_image.jpg"  # Change the filename if needed
            cv2.imwrite(output_path, cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR))
            rospy.loginfo(f"Annotated image saved to {output_path}")
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def led_control_publisher():
        pub = rospy.Publisher('/led_control', String, queue_size=10)
        rospy.init_node('led_control_publisher', anonymous=True)
        rate = rospy.Rate(1)  

        while not rospy.is_shutdown():
            message = ["RELAY_OFF", "RELAY_ON"]
            for m in message:
                pub.publish(m)  
                rospy.loginfo(f"Published: {m}")
                time.sleep(5)


if __name__ == "__main__":
    try:
        sprayer = Sprayer()
        image_path = "onion_leaves.jpeg"  
        robot.test_with_stored_image(image_path)
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")
        cv2.destroyAllWindows()


# Arduino Uno ID
# Bus 001 Device 006: ID 2341:0069 Arduino SA 
# ATTRS{serial}=="30022E1436313536DFA333334B572F3E"
# SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0069", ATTRS{serial}=="30022E1436313536DFA333334B572F3E", SYMLINK+="arduino-uno"