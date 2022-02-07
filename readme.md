# JetsonMV
ou comment dire que c'est le code pour la machine vision de la CDFR

## Installation
Utiliser Ubuntu

Avoir Cuda d'installé

Installer cmake et Ninja

Installer tout ca, si une ligne marche pas supprimez les trucs qui veulent pas
```console
sudo apt-get install -y build-essential cmake git unzip pkg-config zlib1g-dev
sudo apt-get install -y libjpeg-dev libjpeg8-dev libjpeg-turbo8-dev libpng-dev libtiff-dev
sudo apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libglew-dev
sudo apt-get install -y libgtk2.0-dev libgtk-3-dev libgtkglext1-dev libcanberra-gtk*
sudo apt-get install -y python-dev python-numpy python-pip
sudo apt-get install -y python3-dev python3-numpy python3-pip
sudo apt-get install -y libxvidcore-dev libx264-dev libgtk-3-dev
sudo apt-get install -y libtbb2 libtbb-dev libdc1394-22-dev libxine2-dev
sudo apt-get install -y gstreamer1.0-tools libv4l-dev v4l-utils v4l2ucp  qv4l2 
sudo apt-get install -y libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev
sudo apt-get install -y libavresample-dev libvorbis-dev libxine2-dev libtesseract-dev
sudo apt-get install -y libfaac-dev libmp3lame-dev libtheora-dev libpostproc-dev
sudo apt-get install -y libopencore-amrnb-dev libopencore-amrwb-dev
sudo apt-get install -y libopenblas-dev libatlas-base-dev libblas-dev
sudo apt-get install -y liblapack-dev liblapacke-dev libeigen3-dev gfortran
sudo apt-get install -y libhdf5-dev protobuf-compiler
sudo apt-get install -y libprotobuf-dev libgoogle-glog-dev libgflags-dev
```
Télécharger opencv et opencv_contrib et les mettre dans home, créer un fichier build dans `~/opencv`

Télécharger [libvtk](https://vtk.org/download/), et extraire les sources dans home

Créer un fichier pour le build de vtk : `mkdir buildvtk`

`cd buildvtk`

Puis build vtk : 
```console
cmake -G Ninja -D CMAKE_BUILD_TYPE=RELEASE ../VTK-9.1.0
ninja
sudo ninja install
```

VTK permet d'avoir le module Viz et Viz3D.

puis une fois tout ca, exécuter InstallOpenCV.sh

`./InstallOpenCV.sh`

## Coder dessus
Je recommande VisualStudioCode

Avec l'extension CMake, faire un Clean Reconfigure All Projects à chaque fois que un .h et/ou un .cpp est ajouté.

Le code est fait pour des C920, mais ca peut s'adapter.

## Lancer le code

l'exécutable se trouvera dans le fichier Build

`./Robotronikaruco -h` pour avoir l'aide