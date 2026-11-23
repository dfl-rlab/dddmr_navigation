#!/bin/bash
set -euo pipefail

mkdir -p "$HOME/dddmr_bags"

echo -n "Do you want to download the Lite3 navigation demo map (stair_2_map.zip)? (Y/N): "
read -r d_bag
if [[ "$d_bag" == [Yy]* ]]; then
  echo "Downloading stair_2_map.zip"
  cd "$HOME/dddmr_bags"
  curl --fail --location --output stair_2_map.zip \
    'https://drive.usercontent.google.com/uc?export=download&id=1FWVlrNWR_z7shAPioH6O1eaEaGCIPd7L'
  unzip stair_2_map.zip
fi
