#!/bin/bash

mkdir ~/dddmr_bags/cicdtest

echo -n "Do you want to download mapping_airy_t45:"
read d_bag0
if [ "$d_bag0" != "${d_bag0#[Yy]}" ] ;then 
  echo "Download map"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1B-f2edVrgO4Bw-tVY9VpVGsTTup6F8Ly \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mapping_airy_t45.zip \
      'https://drive.usercontent.google.com/download?id='1B-f2edVrgO4Bw-tVY9VpVGsTTup6F8Ly'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mapping_airy_t45.zip
fi

echo -n "Do you want to download mapping_c16_t0:"
read d_bag1
if [ "$d_bag1" != "${d_bag1#[Yy]}" ] ;then 
  echo "Download map"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1WVpDCs0Mq-ok8mEBRtnCTEkjLyFuO_Bl \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mapping_c16_t0.zip \
      'https://drive.usercontent.google.com/download?id='1WVpDCs0Mq-ok8mEBRtnCTEkjLyFuO_Bl'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mapping_c16_t0.zip
fi

echo -n "Do you want to download mapping_mid360_t13:"
read d_bag2
if [ "$d_bag2" != "${d_bag2#[Yy]}" ] ;then 
  echo "Download map"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1SJzm7K-0RLl9JbWq_5YiYYnDTKsgYcdZ \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mapping_mid360_t13.zip \
      'https://drive.usercontent.google.com/download?id='1SJzm7K-0RLl9JbWq_5YiYYnDTKsgYcdZ'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mapping_mid360_t13.zip
fi

echo -n "Do you want to download mapping_mid360_t180:"
read d_bag3
if [ "$d_bag3" != "${d_bag3#[Yy]}" ] ;then 
  echo "Download map"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='17U16V2Rg4WNrqfp7zHsEeYQih-0zNT5C \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mapping_mid360_t13.zip \
      'https://drive.usercontent.google.com/download?id='17U16V2Rg4WNrqfp7zHsEeYQih-0zNT5C'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mapping_mid360_t13.zip
fi

echo -n "Do you want to download mcl_3dl_c16:"
read d_bag4
if [ "$d_bag4" != "${d_bag4#[Yy]}" ] ;then 
  echo "Download map"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1AWGE5bjq251zbBEGecH0s-7UuWQMDW1L \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mcl_3dl_c16.zip \
      'https://drive.usercontent.google.com/download?id='1AWGE5bjq251zbBEGecH0s-7UuWQMDW1L'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mcl_3dl_c16.zip
fi
