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
# available_plants = {}  # fiducial_id: [plant_type, image]

def jpg_to_base64(image_path):
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')


# testing data
# available_plants = [ 
#     ['108', 'Cactus', jpg_to_base64('images/cactus.jpg')],
#     ['109', 'Basil', jpg_to_base64('images/basil.jpg')],
#     ['110', 'Thyme', jpg_to_base64('images/thyme.jpg')],
#     ['111', 'Cactus', jpg_to_base64('images/cactus_2.jpg')],

# ]
available_plants = []



# # Function to print the current instruction every 5 seconds
# def print_instructions():
#     while True:
#         print(f"Current instruction: {INSTRUCTION}")
#         time.sleep(5)

# threading.Thread(target=print_instructions, daemon=True).start()


# Serve React app
@app.route('/')
def serve_react_app():
    """Serve the main React app entry point."""
    return send_from_directory(app.static_folder, 'index.html')

# # Serve static files for React app
# @app.route('/<path:path>')
# def serve_static_files(path):
#     """Serve static files from the React build folder."""
#     file_path = os.path.join(app.static_folder, path)
#     if os.path.exists(file_path):
#         return send_from_directory(app.static_folder, path)
#     else:
#         return send_from_directory(app.static_folder, 'index.html')
@app.route('/hi', methods=['GET'])
def hello():
    return 'hello'

@app.route('/get_plants', methods=['GET'])
def get_available_plants():
    """Get a list of available plants.

    Returns:
        JSON response with available plants and HTTP status 200.
    """
    # response = {fiducial_id: {"plant_type": plant[0], "image": plant[1]} for fiducial_id, plant in available_plants.items()}

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
        # if any(fiducial_id == str(plant[0]) for plant in available_plants):
        #     INSTRUCTION = instruction
        # else:
        #     return jsonify({"error": "Invalid plant"}), 400
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


    # return jsonify({"message": "Available plants updated successfully"}), 200

if __name__ == '__main__':
    # Set debug to False for production
    # app.run(debug=False)
    app.run(host='0.0.0.0', port=6969, debug=True)

    
