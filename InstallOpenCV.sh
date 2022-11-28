sudo -v

#!/bin/bash
set -e

# reveal the CUDA location
cd ..
sudo sh -c "echo '/usr/local/cuda/lib64' >> /etc/ld.so.conf.d/nvidia-tegra.conf"
sudo ldconfig


# set install dir
cd opencv/
mkdir build || echo "Build folder already existed, skipping..."
cd build

#-D WITH_NVCUVID=ON \ potetiellement pour cudacodec::VideoCapture
#-D VIDEOIO_PLUGIN_LIST=ffmpeg,gstreamer \

# CUDA_ARCH_BIN : 53 pour la jetson Nano, 50,52,61,75,86 pour les GPU
# run cmake
cmake -G Ninja \
-D CMAKE_BUILD_TYPE=Release \
-D CMAKE_INSTALL_PREFIX=/usr \
-D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
-D EIGEN_INCLUDE_PATH=/usr/include/eigen3 \
-D WITH_OPENCL=ON \
-D WITH_OPENMP=ON \
-D WITH_OPENGL=ON \
-D WITH_CUDA=OFF \
-D WITH_CUDNN=OFF \
-D WITH_NVCUVID=ON \
-D WITH_CUBLAS=ON \
-D ENABLE_FAST_MATH=ON \
-D CUDA_FAST_MATH=ON \
-D OPENCV_DNN_CUDA=OFF \
-D CUDA_GENERATION=Auto \
-D WITH_QT=OFF \
-D BUILD_TIFF=ON \
-D WITH_FFMPEG=ON \
-D WITH_GSTREAMER=ON \
-D WITH_TBB=ON \
-D BUILD_TBB=ON \
-D BUILD_TESTS=OFF \
-D WITH_EIGEN=ON \
-D WITH_V4L=ON \
-D WITH_VTK=OFF \
-D WITH_LIBV4L=ON \
-D WITH_GTK=ON \
-D WITH_GTK_2_X=OFF \
-D OPENCV_ENABLE_NONFREE=ON \
-D INSTALL_C_EXAMPLES=OFF \
-D INSTALL_PYTHON_EXAMPLES=OFF \
-D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D HIGHGUI_ENABLE_PLUGINS=OFF \
-D BUILD_opencv_python2=OFF \
-D BUILD_opencv_python3=OFF \
-D BUILD_JAVA=OFF \
-D BUILD_EXAMPLES=OFF ../

sudo -v
ninja

sudo rm -r /usr/include/opencv4/opencv2 || echo "No previous installation of opencv, skipping..."
sudo ninja install
sudo ldconfig

clear
echo "Done!"
