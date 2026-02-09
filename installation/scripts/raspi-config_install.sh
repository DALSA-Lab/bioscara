#!/usr/bin/env bash
set -euo pipefail

pkgs=(
  https://archive.raspberrypi.org/debian/pool/main/r/raspi-utils/raspi-utils-core_20240402-4_arm64.deb
  https://archive.raspberrypi.org/debian/pool/main/r/raspi-utils/raspi-utils-otp_20240903-1_all.deb
  https://archive.raspberrypi.org/debian/pool/main/r/raspi-utils/raspi-utils-dt_20240402-4_arm64.deb
  https://archive.raspberrypi.org/debian/pool/main/r/raspi-utils/raspi-utils-eeprom_20240903-1_arm64.deb
  https://archive.raspberrypi.org/debian/pool/main/k/kms++/libkms++-dev_0\~git20231115\~065257+9ae90ce-1_arm64.deb
  https://archive.raspberrypi.org/debian/pool/main/k/kms++/libkms++0_0\~git20231115\~065257+9ae90ce-1_arm64.deb
  https://archive.raspberrypi.org/debian/pool/main/k/kms++/kms++-utils_0\~git20231115\~065257+9ae90ce-1_arm64.deb
  https://archive.raspberrypi.org/debian/pool/main/r/raspi-utils/raspinfo_20240903-1_all.deb
  https://archive.raspberrypi.org/debian/pool/main/r/raspi-utils/raspi-utils_20240903-1_all.deb
  https://archive.raspberrypi.org/debian/pool/main/r/raspi-config/raspi-config_20250210_all.deb
)

conflicts=(
  /usr/bin/dtmerge
  /usr/bin/dtoverlay
  /usr/bin/dtparam
  /usr/bin/vcgencmd
  /usr/bin/vcmailbox
  /usr/share/man/man1/dtmerge.1.gz
  /usr/share/man/man1/dtoverlay.1.gz
  /usr/share/man/man1/dtparam.1.gz
  /usr/share/man/man1/vcgencmd.1.gz
  /usr/share/man/man1/vcmailbox.1.gz
  /usr/share/man/man7/raspiotp.7.gz
  /usr/share/man/man7/raspirev.7.gz
  /usr/share/man/man7/vcmailbox.7.gz
)

for f in ${conflicts[@]}
do
  sudo dpkg-divert --package libraspberrypi-bin --divert $(dirname $f)/raspi_$(basename $f) --rename $f
done

echo ${pkgs[@]} | xargs -n 1 -P $(nproc) wget -q

sudo apt -y install gdebi

echo ${pkgs[@]} | xargs -n 1 basename | xargs -n 1 sudo gdebi -n
