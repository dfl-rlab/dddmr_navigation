#!/bin/bash

mkdir ~/dddmr_bags/cicdtest

echo -n "Do you want to download mapping_airy_t45 (Y/N):"
read d_bag0
if [ "$d_bag0" != "${d_bag0#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1x-RiGBvGuA70Fc7Z6yaYUp2q_4LFWV0M \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mapping_airy_t45.zip \
      'https://drive.usercontent.google.com/download?id='1x-RiGBvGuA70Fc7Z6yaYUp2q_4LFWV0M'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mapping_airy_t45.zip
fi

echo -n "Do you want to download mapping_c16_t0 (Y/N):"
read d_bag1
if [ "$d_bag1" != "${d_bag1#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1WEbZc5NxbI5eJE0T449aImvfk-8nqMvk \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mapping_c16_t0.zip \
      'https://drive.usercontent.google.com/download?id='1WEbZc5NxbI5eJE0T449aImvfk-8nqMvk'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mapping_c16_t0.zip
fi

echo -n "Do you want to download mapping_mid360_t13 (Y/N):"
read d_bag2
if [ "$d_bag2" != "${d_bag2#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='173k2bLIDDXebG06N3dl_ZCA-PmkqFRYV \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mapping_mid360_t13.zip \
      'https://drive.usercontent.google.com/download?id='173k2bLIDDXebG06N3dl_ZCA-PmkqFRYV'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mapping_mid360_t13.zip
fi

echo -n "Do you want to download mapping_mid360_t180 (Y/N):"
read d_bag3
if [ "$d_bag3" != "${d_bag3#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1thij50P78jlxH-7JOqKefDZ1v1NJO5is \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mapping_mid360_t13.zip \
      'https://drive.usercontent.google.com/download?id='1thij50P78jlxH-7JOqKefDZ1v1NJO5is'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mapping_mid360_t13.zip
fi

echo -n "Do you want to download mcl_3dl_c16 (Y/N):"
read d_bag4
if [ "$d_bag4" != "${d_bag4#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='14qVJYT42Lm2vC5oUE1oJ5xKktvc2_cex \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mcl_3dl_c16.zip \
      'https://drive.usercontent.google.com/download?id='14qVJYT42Lm2vC5oUE1oJ5xKktvc2_cex'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mcl_3dl_c16.zip
fi

echo -n "Do you want to download mapping_airy_t45_trt (Y/N):"
read d_bag5
if [ "$d_bag5" != "${d_bag5#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1D3V93raJa36koFYBsrItM5I_NLJl1-uh \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mapping_airy_t45_trt.zip \
      'https://drive.usercontent.google.com/download?id='1D3V93raJa36koFYBsrItM5I_NLJl1-uh'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mapping_airy_t45_trt.zip
fi

echo -n "Do you want to download mapping_gpulidar_t0 (Y/N):"
read d_bag6
if [ "$d_bag6" != "${d_bag6#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1vP578A_npdXtBkolVaBKhgICrEmoYdQP \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mapping_gpulidar_t0.zip \
      'https://drive.usercontent.google.com/download?id='1vP578A_npdXtBkolVaBKhgICrEmoYdQP'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mapping_gpulidar_t0.zip
fi

echo -n "Do you want to download mcl_3dl_gpulidar (Y/N):"
read d_bag7
if [ "$d_bag7" != "${d_bag7#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1yZ9xu-0X5cwo_YgGczdTLG46jFTLyH4j \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mcl_3dl_gpulidar.zip \
      'https://drive.usercontent.google.com/download?id='1yZ9xu-0X5cwo_YgGczdTLG46jFTLyH4j'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mcl_3dl_gpulidar.zip
fi

echo -n "Do you want to download perception_3d_multilayer_spinning_lidar_hokuyo2d (Y/N):"
read d_bag8
if [ "$d_bag8" != "${d_bag8#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1OOBStIDW5eispViKaS9nYX5KOBSMSQ9D \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o perception_3d_multilayer_spinning_lidar_hokuyo2d.zip \
      'https://drive.usercontent.google.com/download?id='1OOBStIDW5eispViKaS9nYX5KOBSMSQ9D'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip perception_3d_multilayer_spinning_lidar_hokuyo2d.zip
fi
