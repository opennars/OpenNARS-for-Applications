cd /home/jetson
git clone http://91.203.212.130/Transbot/tensorrt_demos.zip
unzip tensorrt_demos.zip
##alternatively:
#git clone https://github.com/patham9/tensorrt_demos
#cd tensorrt_demos/ssd
#./install_pycuda.sh
#sudo pip3 install onnx==1.4.1
#cd ..
#cd plugins
#make
#cd ..
#cd yolo
#./download_yolo.sh
##give 2GB of extra SD card swap space via jtop
#python3 yolo_to_onnx.py -m yolov4-416
#python3 onnx_to_tensorrt.py -m yolov4-416
##wait till it finishes, this can take up to an hour
##Remove the 2GB of extra SD card swap space
