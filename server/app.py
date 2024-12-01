from flask import Flask, jsonify, request, send_from_directory
from ultralyticsplus import YOLO, render_result
from PIL import Image
import io
import os

# Create Flask app instance
app = Flask(__name__)

# Load model globally to avoid reloading it for each request
model = YOLO('foduucom/plant-leaf-detection-and-classification')
model.overrides['conf'] = 0.25  # NMS confidence threshold
model.overrides['iou'] = 0.45  # NMS IoU threshold
model.overrides['agnostic_nms'] = False  # NMS class-agnostic
model.overrides['max_det'] = 1000  # Maximum number of detections per image

# Path to the folder containing the HTML file
STATIC_FOLDER = "static"

def generate_result(image):
    # Perform inference
    results = model.predict(image)

    # Render the result
    render = render_result(model=model, image=image, result=results[0])
    render.show()
    return results[0].boxes.xyxy.tolist(), results[0].boxes.cls.tolist(), results[0].boxes.conf.tolist()

# Define a route to serve the HTML file
@app.route('/')
def serve_html():
    return send_from_directory(STATIC_FOLDER, 'index.html')

# Define a predict route
@app.route('/predict', methods=['POST'])
def predict():
    if 'image' not in request.files:
        return jsonify(error="No image file provided"), 400

    # Get the uploaded image
    file = request.files['image']

    if file.filename == '':
        return jsonify(error="No selected file"), 400

    try:
        # Convert the image to a format suitable for the model
        image = Image.open(io.BytesIO(file.read()))
        
        # Generate prediction results
        boxes, classes, confidences = generate_result(image)
        
        # Prepare response data
        response_data = {
            "boxes": boxes,
            "classes": classes,
            "confidences": confidences
        }
        return jsonify(response_data), 200
    except Exception as e:
        return jsonify(error=str(e)), 500

# Run the app
if __name__ == '__main__':
    app.run(debug=True)
