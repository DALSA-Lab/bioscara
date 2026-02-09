#!/bin/bash

# This script has been created by AI

# Define the config file and the udev rules file
CONFIG_FILE="/boot/firmware/config.txt"
UDEV_RULES_FILE="/etc/udev/rules.d/99-pwm.rules"
UDEV_RULE_CONTENT="SUBSYSTEM==\"pwm*\", ACTION==\"add|change\", \\
        RUN+=\"/bin/chgrp -R plugdev '/sys%p'\", \\
        RUN+=\"/bin/chmod -R g=u '/sys%p'\""

# Function to add dtoverlay=pwm to config.txt if not already present
add_dtoverlay() {
    if ! grep -q "^dtoverlay=pwm" "$CONFIG_FILE"; then
        echo "Adding 'dtoverlay=pwm' to $CONFIG_FILE"
        echo "dtoverlay=pwm" | sudo tee -a "$CONFIG_FILE" > /dev/null
    else
        echo "'dtoverlay=pwm' is already present in $CONFIG_FILE"
    fi
}

# Function to create the udev rule
create_udev_rule() {
    if [ ! -f "$UDEV_RULES_FILE" ]; then
        echo "Creating udev rule file at $UDEV_RULES_FILE"
        echo "$UDEV_RULE_CONTENT" | sudo tee "$UDEV_RULES_FILE" > /dev/null
    else
        echo "Udev rule file already exists at $UDEV_RULES_FILE"
    fi
}

# Main script execution
add_dtoverlay
create_udev_rule

# Reload udev rules
echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Script execution completed. REBOOT required"