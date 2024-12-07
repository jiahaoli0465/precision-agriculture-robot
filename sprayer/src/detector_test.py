import base64
from openai import OpenAI
from detector import Detector

# client = OpenAI(api_key = "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")

# Function to encode the image
def encode_image(image_path):
  with open(image_path, "rb") as image_file:
    return base64.b64encode(image_file.read()).decode('utf-8')

# Path to your image
image_path = "./images/basil.png"

# Getting the base64 string
base64_image = encode_image(image_path)


detector = Detector()


is_detected, plant_type  = detector.detect_plant(base64_image)

print('object is ', is_detected, ' and it is a ', plant_type)

# print(detector.is_plant(base64_image))


# retries = 10

# for i in range(retries):
#     try:
#         response = client.chat.completions.create(
#         model="gpt-4o-mini",
#         messages=[
#             {
#             "role": "user",
#             "content": [
#                 {
#                 "type": "text",
#                 "text": "Output in one word true or false if the image contains any plant",
#                 },
#                 {
#                 "type": "image_url",
#                 "image_url": {
#                     "url":  f"data:image/jpeg;base64,{base64_image}"
#                 },
#                 },
#             ],
#             }
#         ],
#         )
#         print(response.choices[0].message.content)

#         break
#     except Exception as e:
#         print('failed times: ', i, 'error: ', e)



