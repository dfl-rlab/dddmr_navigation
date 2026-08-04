#!/bin/bash

mkdir ~/dddmr_bags/cicdtest

echo -n "Do you want to download mapping_airy_t45:"
read d_bag0
if [ "$d_bag0" != "${d_bag0#[Yy]}" ] ;then 
  echo "Download map"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1x-RiGBvGuA70Fc7Z6yaYUp2q_4LFWV0M \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mapping_airy_t45.zip \
      'https://drive.usercontent.google.com/download?id='1x-RiGBvGuA70Fc7Z6yaYUp2q_4LFWV0M'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mapping_airy_t45.zip
fi

echo -n "Do you want to download mapping_c16_t0:"
read d_bag1
if [ "$d_bag1" != "${d_bag1#[Yy]}" ] ;then 
  echo "Download map"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1WEbZc5NxbI5eJE0T449aImvfk-8nqMvk \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mapping_c16_t0.zip \
      'https://drive.usercontent.google.com/download?id='1WEbZc5NxbI5eJE0T449aImvfk-8nqMvk'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mapping_c16_t0.zip
fi

echo -n "Do you want to download mapping_mid360_t13:"
read d_bag2
if [ "$d_bag2" != "${d_bag2#[Yy]}" ] ;then 
  echo "Download map"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='173k2bLIDDXebG06N3dl_ZCA-PmkqFRYV \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mapping_mid360_t13.zip \
      'https://drive.usercontent.google.com/download?id='173k2bLIDDXebG06N3dl_ZCA-PmkqFRYV'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mapping_mid360_t13.zip
fi

echo -n "Do you want to download mapping_mid360_t180:"
read d_bag3
if [ "$d_bag3" != "${d_bag3#[Yy]}" ] ;then 
  echo "Download map"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1thij50P78jlxH-7JOqKefDZ1v1NJO5is \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mapping_mid360_t13.zip \
      'https://drive.usercontent.google.com/download?id='1thij50P78jlxH-7JOqKefDZ1v1NJO5is'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mapping_mid360_t13.zip
fi

echo -n "Do you want to download mcl_3dl_c16:"
read d_bag4
if [ "$d_bag4" != "${d_bag4#[Yy]}" ] ;then 
  echo "Download map"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1Whou_U42qgsI0ssA0_cF5kzmFw7TKAfc \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mcl_3dl_c16.zip \
      'https://drive.usercontent.google.com/download?id='1Whou_U42qgsI0ssA0_cF5kzmFw7TKAfc'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mcl_3dl_c16.zip
fi

echo -n "Do you want to download mapping_airy_t45_trt:"
read d_bag5
if [ "$d_bag5" != "${d_bag5#[Yy]}" ] ;then 
  echo "Download map"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1D3V93raJa36koFYBsrItM5I_NLJl1-uh \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mapping_airy_t45_trt.zip \
      'https://drive.usercontent.google.com/download?id='1D3V93raJa36koFYBsrItM5I_NLJl1-uh'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mapping_airy_t45_trt.zip
fi