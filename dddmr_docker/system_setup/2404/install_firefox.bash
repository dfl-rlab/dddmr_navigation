#!/bin/bash
sudo apt update
sudo apt install flatpak gnome-software-plugin-flatpak
# Force delete the current broken flathub configuration
flatpak remote-delete --force flathub

# Re-add flathub cleanly using the official repository file
flatpak remote-add --if-not-exists flathub https://dl.flathub.org/repo/flathub.flatpakrepo
sudo flatpak update
sudo flatpak install flathub org.mozilla.firefox
