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
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1Y5qwgmV0Mrk2rL9WiT2UTooJNlbeKekf \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o perception_3d_multilayer_spinning_lidar_hokuyo2d.zip \
      'https://drive.usercontent.google.com/download?id='1Y5qwgmV0Mrk2rL9WiT2UTooJNlbeKekf'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip perception_3d_multilayer_spinning_lidar_hokuyo2d.zip
fi

echo -n "Do you want to download perception_3d_multilayer_spinning_lidar_gpulidar (Y/N):"
read d_bag9
if [ "$d_bag9" != "${d_bag9#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1IbOGpiooe1YNonGCNgCpt8IkuH4qhazg \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o perception_3d_multilayer_spinning_lidar_gpulidar.zip \
      'https://drive.usercontent.google.com/download?id='1IbOGpiooe1YNonGCNgCpt8IkuH4qhazg'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip perception_3d_multilayer_spinning_lidar_gpulidar.zip
fi


echo -n "Do you want to download mapping_jt128_t45 (Y/N):"
read d_bag10
if [ "$d_bag10" != "${d_bag10#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1VE6zUmlT21IA9MIoN36JZYuBUB6w6He2 \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o mapping_jt128_t45.zip \
      'https://drive.usercontent.google.com/download?id='1VE6zUmlT21IA9MIoN36JZYuBUB6w6He2'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip mapping_jt128_t45.zip
fi

echo -n "Do you want to download perception_3d_multilayer_spinning_lidar_gpulidar_static (Y/N):"
read d_bag11
if [ "$d_bag11" != "${d_bag11#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1mIJIGwdDQF0AzbRxjiQDBRSqv79YaX8v \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o perception_3d_multilayer_spinning_lidar_gpulidar_static.zip \
      'https://drive.usercontent.google.com/download?id='1mIJIGwdDQF0AzbRxjiQDBRSqv79YaX8v'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip perception_3d_multilayer_spinning_lidar_gpulidar_static.zip
fi

echo -n "Do you want to download perception_3d_depth_camera_rs457 (Y/N):"
read d_bag12
if [ "$d_bag12" != "${d_bag12#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1O1EHucj1S_RocQdnTQR8dd7UfdN3h7xq \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o perception_3d_depth_camera_rs457.zip \
      'https://drive.usercontent.google.com/download?id='1O1EHucj1S_RocQdnTQR8dd7UfdN3h7xq'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip perception_3d_depth_camera_rs457.zip
fi

echo -n "Do you want to download nav_ackermann_p2p (Y/N):"
read d_bag13
if [ "$d_bag13" != "${d_bag13#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags/cicdtest && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='14CN4Z5-GSCMPXG9MkSmX6fPS6LZPHxTM \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o nav_ackermann_p2p.zip \
      'https://drive.usercontent.google.com/download?id='14CN4Z5-GSCMPXG9MkSmX6fPS6LZPHxTM'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip nav_ackermann_p2p.zip
fi

