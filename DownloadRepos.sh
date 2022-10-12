./InstallRequirement.sh
cd ..
echo "Déverouillage admin, ca sert pour installer"
sudo -v 
git clone https://gitlab.kitware.com/vtk/vtk.git || echo "Failed to clone vtk : already exists"
cd vtk/
git checkout v9.2.2
mkdir build || echo "Build folder already existed, skipping..."
cd build
cmake -G Ninja -D CMAKE_BUILD_TYPE=RELEASE ../
sudo -v
ninja
sudo ninja install
cd ../..
sudo -v


git clone https://github.com/opencv/opencv_contrib.git || echo "Failed to clone opencv_contrib : already exists"
cd opencv_contrib/
git checkout 4.6.0
sudo -v
cd ..
git clone https://github.com/opencv/opencv.git || echo "Failed to clone opencv : already exists"
cd opencv/
git checkout 4.6.0
sudo -v
cd ..
cd jetsonMV/
./InstallOpenCV.sh