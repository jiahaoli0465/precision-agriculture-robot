from flask import Flask, request, jsonify, send_from_directory
import os
import base64
import time
import threading
from flask_cors import CORS


# Initialize Flask app with static folder for serving React app
app = Flask(__name__, static_folder='build', static_url_path='')
CORS(app)

# Global variables
ALL_INSTRUCTION = ['GO_HOME', 'GO_TO_PLANT_', 'SCAN_ALL', 'NONE']
INSTRUCTION = 'NONE'


def jpg_to_base64(image_path):
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')


available_plants = []





# Serve React app
@app.route('/')
def serve_react_app():
    """Serve the main React app entry point."""
    return send_from_directory(app.static_folder, 'index.html')

@app.route('/hi', methods=['GET'])
def hello():
    return 'hello'

@app.route('/get_plants', methods=['GET'])
def get_available_plants():
    """Get a list of available plants.

    Returns:
        JSON response with available plants and HTTP status 200.
    """

    return jsonify(available_plants), 200

@app.route('/get_instruction', methods=['GET'])
def get_instruction():
    """Get the current instruction.

    Returns:
        JSON response with the current instruction and HTTP status 200.
    """
    global INSTRUCTION
    return jsonify({"instruction": INSTRUCTION}), 200

@app.route('/reset_all', methods=['POST'])
def reset_all():
    global INSTRUCTION
    global available_plants

    INSTRUCTION = 'NONE'
    available_plants = []
    return jsonify({"message": "All data reset successfully"}), 200

@app.route('/update_instruction', methods=['POST'])
def update_instruction():
    """Update the current instruction.

    Expects JSON payload with 'instruction' key.
    Validates and updates the global INSTRUCTION variable.

    Returns:
        Success or error message with appropriate HTTP status.
    """
    global INSTRUCTION
    data = request.get_json()
    if not data:
        return jsonify({"error": "Invalid input"}), 400

    instruction = data.get('instruction')
    if not instruction:
        return jsonify({"error": "Instruction not provided"}), 400
    print('Setting instruction to:', instruction)

    if instruction == 'GO_HOME':
        INSTRUCTION = 'GO_HOME'
    elif instruction == 'SCAN_ALL':
        INSTRUCTION = 'SCAN_ALL'
    elif instruction == 'NONE':
        INSTRUCTION = 'NONE'
    elif instruction.startswith('GO_TO_PLANT_'):
        fiducial_id = instruction.split('_')[-1]
        INSTRUCTION = instruction

    else:
        return jsonify({"error": "Invalid instruction"}), 400

    return jsonify({"message": "Instruction updated successfully"}), 200

@app.route('/update_plants', methods=['POST'])
def update_available_plants():
    
    global available_plants

    data = request.get_json()

    if not data:
        return jsonify({"error": "Invalid input"}), 400
    
    available_plants = data.get('available_plants')
    print('Setting available plants to:', available_plants)
    return jsonify({"message": "Available plants updated successfully"}), 200



if __name__ == '__main__':
    # Set debug to False for production
    # app.run(debug=False)
    app.run(host='0.0.0.0', port=6969, debug=True)

    
