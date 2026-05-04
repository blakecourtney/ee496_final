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
LINUX:
Configuring a PC to Flash Code(linux):
# install dependencies
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

# clone ESP-IDF
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v5.5.2  # match your current version

# install tools
./install.sh esp32

# add to shell
echo '. ~/esp/esp-idf/export.sh' >> ~/.bashrc
source ~/.bashrc

sudo usermod -a -G dialout $USER

. ~/esp/esp-idf/export.sh

MAC:
# install Homebrew if not already installed
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# install dependencies
brew install cmake ninja dfu-util python3

# clone ESP-IDF
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v5.5.2  # match your current version

# install tools
./install.sh esp32

# add to shell
echo '. ~/esp/esp-idf/export.sh' >> ~/.zshrc
source ~/.zshrc

. ~/esp/esp-idf/export.sh

## Configuring Current Terminal to Flash Code
Configure terminal to flash
. ~/esp/esp-idf/export.sh

List USB Com ports (MacOS)
ls /dev/cu.*

List USB Com ports (Ubuntu)
ls /dev/ttyUSB*

Flash to usbserial-0001:
idf.py -p /dev/cu.usbserial-0001 flash monitor

Monitor usbserial-4
idf.py -p /dev/cu.usbserial-4 monitor

## Model Dataset Pipeline
Download the 2 datasets here and move them into a desired directory: <br/>
https://www.kaggle.com/datasets/nikolasgegenava/sard-search-and-rescue/data <br/>
https://universe.roboflow.com/sar-datasets/heridal-dhyqp/dataset/1/images

Run preprocess.py to obtain compressed SARD images. The dimension of the output images is set to be 128x128.
Run data_augment.py to obtain cropped and compressed HERIDAL images. The dimensions are also set to be 128x128 but can be adjusted as well as number of output datapoints through the BG_KEEP_PROB parameter.

Run both files twice for both training and testing images (simply change the path).
Run train.py to train, evaluate the model, and save the model as both an h5 and tflite file.

For deployment, run tflite_convert.py under the human_detect directory.
