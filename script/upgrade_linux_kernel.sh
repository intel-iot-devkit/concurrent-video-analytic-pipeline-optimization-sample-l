#!/bin/bash

root_path=$PWD
target_dir="model"
kernel_name="lts-v5.4.102-yocto-210310T010318Z"
kernel_src_packet="${kernel_name}.tar.gz"
kernel_src_link="https://github.com/intel/linux-intel-lts/archive/refs/tags/${kernel_src_packet}"
kernel_src_md5sum="9492d01c326a2277ce1be72ab070ecb7"
echo $kernel_src_link

#instal md5sum and kernel build tools
sudo apt install coreutils build-essential bc kmod cpio flex libncurses5-dev libelf-dev libssl-dev

if [ $? != 0 ]; 
then
  echo "Failed to run command 'sudo apt install coreutils'"
  echo "Please confirm the network connection is good"
 fi

download_kernel_package () {
  echo "Downloading kernel source code package from $kernel_src_link"
  wget $kernel_src_link
}

compare_kernel_md5 () {
  md5_res=`md5sum $kernel_src_packet`
  if [[ ${kernel_src_md5sum} ==  ${md5_res%% *} ]]
  then
    echo "Md5sum of $kernel_src_packet is ${md5_res}"
    return 0
  else
    echo "Md5sum of $kernel_src_packet $md5_res is not equal to ${kernel_src_md5sum}"
    return 1
  fi
}

retry_count=3
for i in `seq $retry_count`
do
  if [[ $i > 1 ]]
  then
    echo "Try again to download Linux keren source code package."
  fi

  if [[ ! -f "${kernel_src_packet}" ]]
  then 
    download_kernel_package
  fi

  compare_kernel_md5

  if [[ $? == 0 ]]
  then
    break;
  fi
  rm ${kernel_src_packet}
done

compare_kernel_md5()
if [[ $? != 0 ]]
then
  echo "Please try to download $kernel_src_link manually and re-run this script"
  exit -1;
fi

tar zxf $kernel_src_packet --one-top-level=$kernel_name --strip-components 1

if [[ ! -d $kernel_name ]]
then
  echo "Error: $kernel_name doesn't exist!"
  exit -1;
fi
	
echo "Begin to compiling kernel $kernel_name. It will takes about 25 minutes."
sleep 1

cp ./patches/config-5.4.102 $kernel_name/.config
cd $kernel_name
make oldconfig
make -j8
sudo make INSTALL_MOD_STRIP=1 modules_install
sudo make install

echo "Begin to install guc/huc/dmc firmware binaries"
mkdir -p i915_firmware
cd i915_firmware
wget https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/plain/i915/tgl_guc_35.2.0.bin
wget https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/plain/i915/tgl_huc_7.0.3.bin
wget https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/plain/i915/tgl_dmc_ver2_04.bin
sudo cp *.bin /lib/firmware/i915
sudo update-initramfs -u -k all

sudo sed 's/GRUB_CMDLINE_LINUX_DEFAULT.*/GRUB_CMDLINE_LINUX_DEFAULT="quiet splash i915.force_probe=* i915.enable_guc=2"/' -i /etc/default/grub

sudo update-grub
echo "Please reboot and select kernel 5.4.102 for boot"

exit 0

