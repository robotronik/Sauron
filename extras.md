# Documentation sur comment compiler des trucs en plus (ca sert à rien, mais au cas où ca interesse des gens, je le laisse)

Pour savoir quelle architecture cuda ton gpu a, faut aller ici : https://developer.nvidia.com/cuda-gpus


## VTK (optionnel)

Télécharger [libvtk](https://vtk.org/download/), et extraire les sources dans home
OpenCV 4.5.5 marche pas avec VTK 8.2.
VTK 9.1 requiert un CMake supérieur à 3.12, sur Ubuntu 2.18 il est possible que le cmake soit pas à jour, si c'est la cas, suivre [ce tuto](https://askubuntu.com/questions/355565/how-do-i-install-the-latest-version-of-cmake-from-the-command-line)

Attention : Certaines versions de VTK ne marcheront pas avec OpenCV, rester sur les release et non les rc (release candidate)

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

C'est 52 parceque ma carte graphique est architecture 52

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