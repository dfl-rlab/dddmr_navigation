#!/bin/bash

mkdir ~/dddmr_bags

echo -n "Do you want to download rs455_rgbd_848x480 bag files (1.0GB)? (Y/N):"
read d_bag
if [ "$d_bag" != "${d_bag#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1JhBINIwiDVIxrBw7AojxzKa311KGm07T \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o rs455_rgbd_848x480.zip \
      'https://drive.usercontent.google.com/download?id='1JhBINIwiDVIxrBw7AojxzKa311KGm07T'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip rs455_rgbd_848x480.zip
fi

