#!/bin/bash

wget -q https://packages.mozilla.org/apt/repo-signing-key.gpg -O- | gpg --dearmor | sudo tee /etc/apt/keyrings/packages.mozilla.org.gpg > /dev/null

echo "
Types: deb
URIs: https://packages.mozilla.org/apt
Suites: mozilla
Components: main
Signed-By: /etc/apt/keyrings/packages.mozilla.org.gpg
" | sudo tee /etc/apt/sources.list.d/mozilla.sources

echo "
Package: firefox*
Pin: origin packages.mozilla.org
Pin-Priority: 1001
" | sudo tee /etc/apt/preferences.d/mozilla

echo "
Unattended-Upgrade::Origins-Pattern { "archive=mozilla"; };
" | sudo tee /etc/apt/apt.conf.d/51unattended-upgrades-firefox

sudo snap remove firefox
sudo apt remove firefox

sudo apt install firefox