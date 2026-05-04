# EE496: Sonia Ashly Blake

## Guide through this directory:
README.md <br/>
Readme file

/GND_station <br/>
Contains user interface code and serial handler, call main.py to start a session

/mesh_code/drone <br/>
Contains drone networking ESP32 code + CMakeLists.txt to run the IDF compiler

/mesh_code/ground  <br/>
Contains ground side networking ESP32 code + CMakeLists.txt to run the IDF compiler

/payload/drone/human_detect <br/>
Contains ESP32-S3 Camera code, with the main file being human_detect.ino + the model hex file, also tflite_convert.py to convert the model into hex and camera visualization code

/payload/drone/tinyML <br/>
Contains the data processing and model training code + unconverted forms of the model file

/payload/drone/mavlink <br/>
contains firmware librairies for mavlink which is used to interface with the Pixhawk flight controller


## ESPIDF Setup

## Model Dataset Pipeline
Download the 2 datasets here and move them into a desired directory: <br/>
https://www.kaggle.com/datasets/nikolasgegenava/sard-search-and-rescue/data <br/>
https://universe.roboflow.com/sar-datasets/heridal-dhyqp/dataset/1/images

Run preprocess.py to obtain compressed SARD images. The dimension of the output images is set to be 128x128.
Run data_augment.py to obtain cropped and compressed HERIDAL images. The dimensions are also set to be 128x128 but can be adjusted as well as number of output datapoints through the BG_KEEP_PROB parameter.

Run both files twice for both training and testing images (simply change the path).
Run train.py to train, evaluate the model, and save the model as both an h5 and tflite file.

For deployment, run tflite_convert.py under the human_detect directory.
