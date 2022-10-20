#!/bin/bash
export PATH=/opt/intel/svet/onevpl/bin:$PATH
export PKG_CONFIG_PATH=/opt/intel/svet/onevpl/lib/pkgconfig/:$PKG_CONFIG_PATH
export LIBRARY_PATH=/opt/intel/svet/onevpl/lib:$LIBRARY_PATH
export LIBVA_DRIVERS_PATH=/opt/intel/svet/onevpl/lib/dri
export LD_LIBRARY_PATH=/opt/intel/svet/onevpl/lib:/opt/intel/openvino_2022/extras/opencv/lib/:$LD_LIBRARY_PATH
export LIBVA_DRIVER_NAME=iHD
export MFX_HOME=/opt/intel/svet/onevpl
export SVET_INSTALL_OVERWRITE=False
mkdir -p ~/cl_cache
export cl_cache_dir=~/cl_cache


