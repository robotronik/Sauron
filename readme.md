# JetsonMV
ou comment dire que c'est le code pour la machine vision de la CDFR

# Installation
Utiliser Ubuntu

Avoir Cuda d'installé et dans le PATH

https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#introduction

Por les dépendences, y'a soit un script `./InstallRequirement.sh`, soit la version manuelle (qui suit) :

Installer cmake et Ninja (`ninja-build`)

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
sudo apt-get install -y gstreamer1.0-tools libv4l-dev v4l-utils v4l2ucp qv4l2 
sudo apt-get install -y libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev
sudo apt-get install -y libavresample-dev libvorbis-dev libxine2-dev libtesseract-dev
sudo apt-get install -y libfaac-dev libmp3lame-dev libtheora-dev libpostproc-dev
sudo apt-get install -y libopencore-amrnb-dev libopencore-amrwb-dev
sudo apt-get install -y libopenblas-dev libatlas-base-dev libblas-dev
sudo apt-get install -y liblapack-dev liblapacke-dev libeigen3-dev gfortran
sudo apt-get install -y libhdf5-dev protobuf-compiler
sudo apt-get install -y libprotobuf-dev libgoogle-glog-dev libgflags-dev
```
Télécharger opencv, renommer le ficher extrait en `opencv` et opencv_contrib en `opencv_contrib` et les mettre dans home, créer un fichier build dans `~/opencv`

## VTK

Télécharger [libvtk](https://vtk.org/download/), et extraire les sources dans home
OpenCV 4.5.5 marche pas avec VTK 8.2.
VTK 9.1 requiert un CMake supérieur à 3.12, sur Ubuntu 2.18 il est possible que le cmake soit pas à jour, si c'est la cas, suivre [ce tuto](https://askubuntu.com/questions/355565/how-do-i-install-the-latest-version-of-cmake-from-the-command-line)

Créer un fichier pour le build de vtk : `mkdir buildvtk`

`cd buildvtk`

Puis build vtk : 
```console
cmake -G Ninja -D CMAKE_BUILD_TYPE=RELEASE ../
ninja
sudo ninja install
```

VTK permet d'avoir le module Viz et Viz3D.

## FFmpeg (OpenCV marche pas avec chez moi, ca sert à rien, mais au cas où je le laisse)

Il faut suivre ça 

https://docs.nvidia.com/video-technologies/video-codec-sdk/ffmpeg-with-nvidia-gpu/

Selon la distro, sauf que il faut utiliser ffmpeg version 4.4 (demande pas pk), et si tu as 

`ERROR: failed checking for nvcc.`, 

rajoute `--nvccflags="-gencode arch=compute_52,code=sm_52 -O2"`

## Compiler le module nvdec pour GStreamer (ca sert pas non plus)

Ca sert pas non plus parceque nvdec permet de décoder qu'un seul stream à la fois.

On va commencer par installer le video codec de NVidia :

[ça se dl ici](https://developer.nvidia.com/nvidia-video-codec-sdk/download)

Il va falloir copier les fichiers pour l'installer.
D'abord on extrait tout à un endroit, puis on lance un terminal dans le fichier extrait.
Ensuite :
```console
cp Interface/* /usr/local/cuda/include
cp Lib/linux/stubs/x86_64/* /usr/local/cuda/lib64/stubs
```

Puis après, il faut trouver la version de gstreamer installée : `gst-launch-1.0 --version`, pour moi c'est 1.16.2

Ensuite, dans un dossier, faire 
```console
git clone git://anongit.freedesktop.org/git/gstreamer/gst-plugins-bad
cd gst-plugins-bad
git checkout <version>
```
git checkout permet de changer de branche pour prendre la version de gst-plugins-bad qui correspond à notre version de gstreamer, remplacer \<version\> par ce qu'on avait trouvé 3 commandes avant.

Ensuite, on va faire :
```
./autogen.sh --disable-gtk-doc --with-cuda-prefix="/usr/local/cuda" --enable-hls=no
cd sys/nvdec
make
sudo make install
sudo cp .libs/libgstnvdec.so /usr/lib/x86_64-linux-gnu/gstreamer-1.0/
```
C'est pas grave si pendant le make il se plaint de libdrm, balec.
Ensuite on clear le cache de gstreamer : `rm -r ~/.cache/gstreamer-1.0`

Maintenant, `gst-inspect1.0 nvdec` va dire tout plein de trucs si tout est bon

Y'a moyen de faire la même chose sur nvenc si besoin.

## Ok c'est tout

Juste avant d'executer le script, il va falloir le modifier un peu : 
Aux alentours de la ligne 53, y'a `CUDA_ARCH_BIN = un truc` : il faut modifier le `un truc` en fonction de ce que ta carte graphique dispose, (ce lien te dit le niveau de compute
puis une fois tout ca, exécuter InstallOpenCV.sh

`./InstallOpenCV.sh`

# Coder dessus
Je recommande VisualStudioCode

Avec l'extension CMake, faire un Clean Reconfigure All Projects à chaque fois que un .h et/ou un .cpp est ajouté.

Le code est fait pour des C920, mais ca peut s'adapter.
Sur mon PC v4l2 est con, c'est à dire qu'il donne plus les noms des webcams, du coup dans Camera.cpp le autodetect est fait pour marcher avec des caméra dans le nom "Video Capture 4"...

Ci dessous je décris tous les modules de mon code.

## Boardviz

C'est un ancien module de visualisation 2D du terrain, utile pour quand j'arrivais pas à avoir VTK, maintenant c'est plus utile mais ca marche et c'est hella fast

## BoardViz3D

Bah y'a une classe, mais en vrai c'est des utilitaires pour viz3d

## Calibrate et Calibfile

Ca permet de calibrer les caméras et stocker les calibrations. Lors de la calibration, (`./Robotronikaruco -c`), ESPACE pour capturer une image, ESC pour quitter, et Entrée pour calculer la calibration. Les images utilisées sont stockées dans `build/TempCalib`.
La calibration c'est important parceque ca permet de détecter les paramètres genre angle de vue et tout de la caméra, pour transformer les tags aruco dans l'espace.
Pour calibrer, il faut imprimer un échiquier. Le coder détecte les intersections entre les carrés, et il faut absolument que la feuille avec l'échiquier soit plate, et que la largeur des cotés soit connue. La taille (en intersections) de l'échiquier et du coté des carrés est définie en haut du fichier

## Camera

Ca s'occupe de détecter, démarrer et configurer les caméra (tg ca marche)

## FrameCounter

Une classe qui permet d'afficher les fps, c'est utile

## Globalconf

Une classe de configuration et de boilerplate, pour les tags aruco principalement.

## math3d

Des fonctions de math, en 3D, no shit sherlock.

## ObjectTracker et TrackedObject

Alors ça, j'en suis pas peu fier: 
TrackedObject est une classe générique qui permet de représenter des objects dans l'espace en fonction des tags aruco. Les tags aruco ont leur position par rapport à leur parent, et nécéssitent d'être enregistrés dans le TrackedObject.

L'ObjectTracker permet de transformer des vues de tags Aruco en position des objets.

## OutputImage

C'est une classe de base qui permet d'afficher des images en mode grille quand la vue est mode direct. C'est le parent de Camera et de boardviz

## position

C'est une système de position 2D utilisé par boardviz. En vrai ca sert à rien.

# Lancer le code

L'exécutable se trouvera dans le fichier Build

`./Robotronikaruco -h` pour avoir l'aide

# Jetson

Surtout si c'est dans le cas où les caméras sont connectées en MIPI-CSI, des fois le daemon plante : éxecuter
`systemctl restart nvargus-daemon.service`