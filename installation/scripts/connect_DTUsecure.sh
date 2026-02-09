#!/bin/bash

skipstep=1
credsload=1

if [ "$EUID" -ne 0 ]
  then echo "Permission denied... Run as root."
    exit 1
fi


if ! command -v nmcli &> /dev/null; then
  echo "nmcli is not installed. Exiting script..."
  exit 0
fi

# Checks if the connection profile already exists
check_nmcli_profile_exist() {
  if [[ $1 == 0 ]]; then
    read -r -p "The $2 connection profile already exists.
Do you wish to continue and overwrite it? [y/N] " answer

    if [[ $answer == "y" || $answer == "Y" ]]; then
      nmcli connection delete id "$2"
      skipstep=1
    else
      skipstep=0
    fi
  else 
    skipstep=1
  fi
}


get_creds() {
  # Get user credentials
  if [[ credsload -ne 0 ]]; then
    read -r -p "Username: " username
    read -r -p "Password: " -s password
    echo
    credsload=0
  fi
}


create_secure_nmcli() {
  echo "Creating connection profile for DTUsecure..."

  get_creds

  UUID=$(nmcli connection add \
    type wifi con-name "DTUsecure" ifname "$interface" ssid "DTUsecure" \
    | grep -oP '(?<=[(])[^)]*')

  CONF_PATH="/etc/netplan/90-NM-"$UUID".yaml"

  TEMPORARY="--temporary"
  read -r -p "Do you want to save the to make it persistent between boots? 
[WARNING] This will store your DTU accounts password in plain text in $CONF_PATH! [y/N]: " continue
  if [[ $continue == "y" || $continue == "y" ]]; then
      TEMPORARY=
      echo "Saving credentials"
  fi

  # Creates connection profile
  nmcli connection modify $TEMPORARY "$nwid" \
    wifi-sec.key-mgmt wpa-eap 802-1x.eap peap 802-1x.phase2-auth mschapv2 \
    802-1x.identity "$username" 802-1x.password "$password" \
    802-1x.anonymous-identity "anonymous@dtu.dk"
}


nmcli_main() {
  nwid="DTUsecure"
  nmcli -f GENERAL.STATE con show $nwid &> /dev/null
  state=$?
  # Gets the name of the wireless interface using nmcli
  interface=$(nmcli dev status | grep -E "(^| )wifi( |$)" | awk '{print $1}')
  check_nmcli_profile_exist "$state" "$nwid"

  if [[ $skipstep -ne 0 ]]; then
    create_secure_nmcli

    # Initial state
    PREV=""
    while true; do
      # Get the current state of $interface
      STATE=$(nmcli -f GENERAL.STATE device show "$interface" \
        | awk '{$1=$2=""; print $0}' \
        | xargs) # Remove leading and trailing white spaces
      
      STATE=${STATE:1:-1}
      
      # Check if the state has changed
      if [[ "$STATE" != "$PREV" ]]; then
          echo "$interface" State: "$STATE"
          PREV="$STATE"
      fi
      if [[ "$STATE" == "connected" ]]; then
          break
      fi
      if [[ "$STATE" == "connecting (need authentication)" ]]; then
          echo "Failed to connect to the network. Check your credentials."
          break
      fi

      sleep 0.1
    done
  fi

}

# Initiate the main suitable for the system
nmcli_main

echo "Exiting script..."