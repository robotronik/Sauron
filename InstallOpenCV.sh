#refresh admin unlock
sudo -v

#exit script if fail
set -e

cd ..

# set install dir
cd opencv/
mkdir build || echo "Build folder already existed, skipping..."
cd build

#-D WITH_NVCUVID=ON \ potetiellement pour cudacodec::VideoCapture
#-D VIDEOIO_PLUGIN_LIST=ffmpeg,gstreamer \
#-D EIGEN_INCLUDE_PATH=/usr/include/eigen3 \

# Ajouter les lignes suivantes si compilation avec CUDA/CuDNN
#-D WITH_CUDA=OFF \
#-D WITH_NVCUVID=ON \
#-D WITH_CUBLAS=ON \
#-D CUDA_FAST_MATH=ON \
#-D WITH_CUDNN=OFF \
#-D OPENCV_DNN_CUDA=ON \


#-D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages \

# CUDA_ARCH_BIN : 53 pour la jetson Nano, 50,52,61,75,86 pour les GPU
# run cmake

cmake -G Ninja \
-D CMAKE_BUILD_TYPE=Release \
-D CMAKE_INSTALL_PREFIX=/usr \
-D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
-D WITH_OPENCL=ON \
-D WITH_OPENMP=ON \
-D WITH_OPENGL=ON \
-D WITH_EIGEN=ON \
-D ENABLE_FAST_MATH=ON \
-D CUDA_GENERATION=Auto \
-D WITH_TBB=ON \
\
-D WITH_CUDA=ON \
-D WITH_NVCUVID=ON \
-D WITH_CUBLAS=ON \
-D CUDA_FAST_MATH=ON \
-D WITH_CUDNN=ON \
-D OPENCV_DNN_CUDA=ON \
\
-D WITH_V4L=ON \
-D WITH_LIBV4L=OFF \
-D WITH_FFMPEG=OFF \
-D WITH_GSTREAMER=ON \
\
-D WITH_QT=ON \
-D WITH_GTK=OFF \
-D WITH_GTK_2_X=OFF \
-D WITH_VTK=OFF \
\
-D OPENCV_ENABLE_NONFREE=ON \
-D INSTALL_C_EXAMPLES=OFF \
-D INSTALL_PYTHON_EXAMPLES=OFF \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D HIGHGUI_ENABLE_PLUGINS=ON \
-D BUILD_opencv_python2=OFF \
-D BUILD_opencv_python3=OFF \
-D BUILD_JAVA=OFF \
-D BUILD_PERF_TESTS=OFF \
-D BUILD_TESTS=OFF \
-D BUILD_opencv_apps=OFF \
-D BUILD_EXAMPLES=OFF \
-D BUILD_wechat_qrcode=OFF ../

sudo -v
ninja

sudo rm -r /usr/include/opencv4/opencv2 || echo "No previous installation of opencv, skipping..."
sudo ninja install
sudo ldconfig

clear
echo "Done!"
