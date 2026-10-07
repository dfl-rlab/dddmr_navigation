#!/bin/bash

sudo add-apt-repository ppa:mozillateam/ppa

echo "
Package: firefox*
Pin: release o=LP-PPA-mozillateam
Pin-Priority: 501

Package: firefox*
Pin: release o=Ubuntu
Pin-Priority: -1
" | sudo tee /etc/apt/preferences.d/mozilla-firefox

sudo apt install --verbose-versions firefox
