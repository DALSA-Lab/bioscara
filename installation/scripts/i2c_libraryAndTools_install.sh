#!/bin/bash

# This script has been created by AI

# Function to display help
function show_help {
    echo "Usage: install_script.sh [options]"
    echo
    echo "Options:"
    echo "  -h, --help        Show this help message"
    echo "  -i, --install-i2c Install i2c-tools"
    echo
    echo "This script installs SWIG and the lg library from GitHub."
}

# Check for options
INSTALL_I2C=false

while [[ "$1" != "" ]]; do
    case $1 in
        -h | --help )        show_help
                             exit
                             ;;
        -i | --install-i2c ) INSTALL_I2C=true
                             ;;
        * )                  echo "Invalid option: $1"
                             show_help
                             exit 1
    esac
    shift
done

# Install SWIG
cd ~
echo "Installing SWIG..."
sudo apt update
sudo apt install -y swig

# Download and install lg
echo "Downloading lg library..."
wget https://github.com/joan2937/lg/archive/master.zip
unzip master.zip
cd lg-master
echo "Building lg library..."
make
sudo make install
cd ..
sudo rm -rf lg-master
rm master.zip

# Optionally install i2c-tools
if [ "$INSTALL_I2C" = true ]; then
    echo "Installing i2c-tools..."
    sudo apt install -y i2c-tools
fi

echo "Installation completed."
