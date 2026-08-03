#!/bin/bash

mkdir ~/dddmr_bags/cicdtest

echo -n "Do you want to download mapping_airy_t45:"
read d_bag0
if [ "$d_bag0" != "${d_bag0#[Yy]}" ] ;then 
  echo "Download map"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1F3JzeDHAH_jcMQ-tM0Pk45_Hsukcpu0y \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mapping_airy_t45.zip \
      'https://drive.usercontent.google.com/download?id='1F3JzeDHAH_jcMQ-tM0Pk45_Hsukcpu0y'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mapping_airy_t45.zip
fi

echo -n "Do you want to download mapping_c16_t0:"
read d_bag1
if [ "$d_bag1" != "${d_bag1#[Yy]}" ] ;then 
  echo "Download map"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1X6dzZWHxqs3VPdz-ntogRTEHIU_CYOfL \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mapping_c16_t0.zip \
      'https://drive.usercontent.google.com/download?id='1X6dzZWHxqs3VPdz-ntogRTEHIU_CYOfL'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mapping_c16_t0.zip
fi

echo -n "Do you want to download mapping_mid360_t13:"
read d_bag2
if [ "$d_bag2" != "${d_bag2#[Yy]}" ] ;then 
  echo "Download map"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1CMP2Am_quh2m_S727-YlJ2KODowsuPUE \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mapping_mid360_t13.zip \
      'https://drive.usercontent.google.com/download?id='1CMP2Am_quh2m_S727-YlJ2KODowsuPUE'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mapping_mid360_t13.zip
fi

echo -n "Do you want to download mapping_mid360_t180:"
read d_bag3
if [ "$d_bag3" != "${d_bag3#[Yy]}" ] ;then 
  echo "Download map"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1ZBDFErOhZa4I2Ep1vREVXBou_temCu1 \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mapping_mid360_t13.zip \
      'https://drive.usercontent.google.com/download?id='1ZBDFErOhZa4I2Ep1vREVXBou_temCu1'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mapping_mid360_t13.zip
fi

echo -n "Do you want to download mcl_3dl_c16:"
read d_bag4
if [ "$d_bag4" != "${d_bag4#[Yy]}" ] ;then 
  echo "Download map"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1JY99lFre4ycW7fd-muTUB-INYfDwtKAq \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mcl_3dl_c16.zip \
      'https://drive.usercontent.google.com/download?id='1JY99lFre4ycW7fd-muTUB-INYfDwtKAq'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mcl_3dl_c16.zip
fi
