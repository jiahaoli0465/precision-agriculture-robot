from ultralyticsplus import YOLO, render_result
from PIL import Image
import io


model_default = YOLO('foduucom/plant-leaf-detection-and-classification')

class LeafDetectionModel:
    def __init__(self, conf=0.25, iou=0.45, agnostic_nms=False, max_det=1000):
        """
        Initialize the Leaf Detection Model with the given parameters.
        :param model_path: Path or name of the YOLO model.
        :param conf: NMS confidence threshold.
        :param iou: NMS IoU threshold.
        :param agnostic_nms: Whether NMS is class-agnostic.
        :param max_det: Maximum number of detections per image.
        """
        self.model = model_default
        self.model.overrides['conf'] = conf
        self.model.overrides['iou'] = iou
        self.model.overrides['agnostic_nms'] = agnostic_nms
        self.model.overrides['max_det'] = max_det

    def predict(self, image_path):
        """
        Perform prediction on the provided image data.
        :param image_data: Image data in bytes or a PIL Image object.
        :return: Tuple of (boxes, classes, confidences).
        """

        # Convert image data to PIL Image if necessary
        # if isinstance(image_data, bytes):
        #     image = Image.open(io.BytesIO(image_data))
        # elif isinstance(image_data, Image.Image):
        #     image = image_data
        # else:
        #     raise ValueError("Invalid image data format. Provide bytes or PIL Image object.")

        # Perform inference
        results = self.model.predict(image_path)
        render = render_result(model=self.model, image=image_path, result=results[0])
        # render.show()


        # Extract results
        # boxes = results[0].boxes.xyxy.tolist()
        # classes = results[0].boxes.cls.tolist()
        # confidences = results[0].boxes.conf.tolist()
        # print('result', results[0])
        label = self.model.names[results[0].boxes.cls]
        print('afsd', self.model.names[results[0].boxes.cls])
        print('label', label)
        # return boxes, classes, confidences

