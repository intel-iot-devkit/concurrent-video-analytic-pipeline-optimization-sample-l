#!/usr/bin/python
"""

Copyright (c) 2018 Intel Corporation All Rights Reserved.

THESE MATERIALS ARE PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL INTEL OR ITS
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THESE
MATERIALS, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

File Name: msdk_pre_install.py

Abstract: Complete install for Intel Visual Analytics, including
 * Intel(R) Media SDK
 * Intel(R) Media driver 
 * Libva
 * Drivers
 * Prerequisites
"""

import os, sys, platform
import os.path
import argparse
import subprocess
import grp
import getopt
import hashlib

class diagnostic_colors:
    ERROR   = '\x1b[31;1m'  # Red/bold
    SUCCESS = '\x1b[32;1m'  # green/bold
    RESET   = '\x1b[0m'     # Reset attributes
    INFO    = '\x1b[34;1m'  # info
    OUTPUT  = ''            # command's coutput printing
    STDERR  = '\x1b[36;1m'  # cyan/bold
    SKIPPED = '\x1b[33;1m'  # yellow/bold

class loglevelcode:
    ERROR   = 0
    SUCCESS = 1
    INFO    = 2

GLOBAL_LOGLEVEL=3

md5_data = {'libva.tar.gz':'4fe5390d84aa4b225987e814e2ea6bdf','libva-utils.tar.gz':'0943f69a7a3841775e7e749093b3b60c','intel-gmmlib-22.1.2.tar.gz':'9470de10a8dcf8589ce37f01002e0336','intel-media-22.3.1.tar.gz':'bddff31bcd2b4555e0d24e36073fd53a','intel-onevpl-22.3.2.tar.gz':'cfbdbbcbd2e2dd66a5ffdb908b0c4e6c','v2022.0.3.tar.gz':'ca1733f00d4ebe2e27f1a06bddb1986a'}

def print_info( msg, loglevel ):
    global GLOBAL_LOGLEVEL

    """ printing information """

    if loglevel==loglevelcode.ERROR and GLOBAL_LOGLEVEL>=0:
        color = diagnostic_colors.ERROR
        msgtype=" [ ERROR ] "
        print( color + msgtype + diagnostic_colors.RESET + msg )
    elif loglevel==loglevelcode.SUCCESS and GLOBAL_LOGLEVEL>=1:
        color = diagnostic_colors.SUCCESS
        msgtype=" [ OK ] "
        print( color + msgtype + diagnostic_colors.RESET + msg )
    elif loglevel==loglevelcode.INFO and GLOBAL_LOGLEVEL>=2:
        color = diagnostic_colors.INFO
        msgtype=" [ INFO ] "
        print( color + msgtype + diagnostic_colors.RESET + msg )
    return

def run_cmd(cmd):
    output=""
    fin=os.popen(cmd+" 2>&1","r")
    for line in fin:
        output+=line
    fin.close()
    return output

def out_md5(src):
    m = hashlib.md5()
    m.update(src.encode('utf-8'))
    return m.hexdigest()

def checkmd5sum(pkg,pkg_dir,cmd):
    install_success = False
    for i in range(3):
        if os.path.exists(pkg) and (out_md5(pkg) == md5_data[pkg]):
            install_success = True
            break          
        else:
            os.system(cmd)
    if install_success:
        print("install %s successful ..."%pkg)
    else:
        print("install %s failed, please install the package manually ..."%pkg)
        sys.exit(-1)


if __name__ == "__main__":

    WORKING_DIR=run_cmd("pwd").strip()
    msg_tmp = "Working directory: " + WORKING_DIR
    print_info(msg_tmp, loglevelcode.INFO)

    BUILD_TARGET=""
    build_msdk=False
    build_all=True
    pre_install=True
    install_overwrite=False

    args = sys.argv[1:]
    optlist = getopt.getopt(args, 'b:i:ham')
    print(optlist)
    for o, a in optlist[0]:
        if o in ("-h"):
            print("[%s" % sys.argv[0] + " usage]")
            print( "\t -h: display help")
            print( "\t -a: install all components")
            print( "\t -b BUILD_TARGET: build all components")
            print( "\t -m : build msdk only")
            exit(0)
        if o in ("-b"):
            BUILD_TARGET=a
            build_all=True
            pre_install=False
            print("BUILD_TARGET=", BUILD_TARGET)
        if o in ("-m"):
            print_info("Build MSDK", loglevelcode.INFO)
            build_msdk=True
            pre_install=False
        if o in ("-a"):
            pre_install=True
        if o in ("-i"):
            if "overwrite" == a:
                print("install_overwrite=True")
                install_overwrite=True
   
    MFX_HOME="/opt/intel/svet/onevpl"
    LIBVA_INSTALL_PATH=MFX_HOME+"/lib"
    LIBVA_DRIVERS_PATH=MFX_HOME+"/dri"
    LIBVA_DRIVER_NAME="iHD"
    LIBVA_INSTALL_PREFIX=MFX_HOME
    
    if install_overwrite == True:
        MFX_HOME="/opt/intel/mediasdk"
        LIBVA_INSTALL_PATH="/usr/lib/x86_64-linux-gnu"
        LIBVA_DRIVERS_PATH="/usr/lib/x86_64-linux-gnu/dri"
        LIBVA_DRIVER_NAME="iHD"
        LIBVA_INSTALL_PREFIX="/usr"
    
    if pre_install == True:
        print("")
        print("************************************************************************")
        print_info("Install required tools and create build environment.", loglevelcode.INFO)
        print("************************************************************************")

        # Install the necessary tools

        print_info("Please input the sudo password to proceed", loglevelcode.INFO)
        cmd ="sudo apt-get -y install git libssl-dev dh-autoreconf cmake libgl1-mesa-dev libpciaccess-dev build-essential curl unzip libavcodec-dev libavutil-dev libavformat-dev libdrm-dev libxext-dev libxfixes-dev libmp3lame-dev libasound2-dev;"
        cmd+="sudo apt-get -y install coreutils pkg-config opencl-clhpp-headers opencl-c-headers ocl-icd-opencl-dev"
        os.system(cmd)

        print("")
        print("************************************************************************")
        print_info("Pull all the source code.", loglevelcode.INFO)
        print("************************************************************************")

        # Pull all the source code
        print("libva")
        if not os.path.exists("%s/libva"%(WORKING_DIR)):
            cmd = "cd %s; rm -f libva.tar.gz; wget https://github.com/intel/libva/archive/refs/tags/2.14.0.tar.gz -O libva.tar.gz;"%(WORKING_DIR)
            cmd+= "tar zxf libva.tar.gz --one-top-level=libva --strip-components 1"
            print(cmd)
            os.system(cmd);
            checkmd5sum("libva.tar.gz","%s/libva"%(WORKING_DIR),cmd)

        print("libva-utils")
        if not os.path.exists("%s/libva-utils"%(WORKING_DIR)):
            cmd = "cd %s;rm -f libva-utils.tar.gz;"%(WORKING_DIR)
            cmd += "wget https://github.com/intel/libva-utils/archive/refs/tags/2.14.0.tar.gz -O libva-utils.tar.gz;"
            cmd += "tar zxf libva-utils.tar.gz --one-top-level=libva-utils --strip-components 1;"
            print(cmd)
            os.system(cmd);
            checkmd5sum("libva-utils.tar.gz","%s/libva-utils"%(WORKING_DIR),cmd)

        print("media-driver")
        if not os.path.exists("%s/media-driver"%(WORKING_DIR)):
            cmd = "cd %s; rm -f intel-media-21.1.3.tar.gz;"%(WORKING_DIR)
            cmd += "wget https://github.com/intel/media-driver/archive/refs/tags/intel-media-22.3.1.tar.gz; "
            cmd += "tar zxf intel-media-22.3.1.tar.gz --one-top-level=media-driver --strip-components 1"
            print(cmd)
            os.system(cmd);
            checkmd5sum("intel-media-22.3.1.tar.gz","%s/media-driver"%(WORKING_DIR),cmd)

        print("gmmlib")
        if not os.path.exists("%s/gmmlib"%(WORKING_DIR)):
            cmd = "cd %s; wget https://github.com/intel/gmmlib/archive/refs/tags/intel-gmmlib-22.1.2.tar.gz; "%(WORKING_DIR)
            cmd += "tar zxf intel-gmmlib-22.1.2.tar.gz --one-top-level=gmmlib --strip-components 1"
            print(cmd)
            os.system(cmd);
            checkmd5sum("intel-gmmlib-22.1.2.tar.gz","%s/gmmlib"%(WORKING_DIR),cmd)

        print("onevpl_gpu")
        if not os.path.exists("%s/onevpl_gpu"%(WORKING_DIR)):
            cmd = "cd %s; rm -f intel-onevpl-22.3.2.tar.gz;  wget https://github.com/oneapi-src/oneVPL-intel-gpu/archive/refs/tags/intel-onevpl-22.3.2.tar.gz;"%(WORKING_DIR)
            cmd+= "tar zxf intel-onevpl-22.3.2.tar.gz --one-top-level=onevpl_gpu --strip-components 1"
            print(cmd)
            os.system(cmd);
            checkmd5sum("intel-onevpl-22.3.2.tar.gz","%s/onevpl_gpu"%(WORKING_DIR),cmd)

        print("onevpl")
        if not os.path.exists("%s/onevpl"%(WORKING_DIR)):
            cmd = "cd %s; rm -f v2022.0.3.tar.gz;  wget https://github.com/oneapi-src/oneVPL/archive/refs/tags/v2022.0.3.tar.gz;"%(WORKING_DIR)
            cmd+= "tar zxf v2022.0.3.tar.gz --one-top-level=onevpl --strip-components 1"
            print(cmd)
            os.system(cmd);
            checkmd5sum("v2022.0.3.tar.gz","%s/onevpl"%(WORKING_DIR),cmd)
   

    if build_all == True:
        print("")
        print("************************************************************************")
        print_info("Build and Install libVA", loglevelcode.INFO)
        print("************************************************************************")

        # Build and install libVA including the libVA utils for vainfo.
        cmd ="cd %s/libva; "%(WORKING_DIR)
        if install_overwrite == False:
            cmd +="export LIBRARY_PATH=%s:$LIBRARY_PATH; "%(LIBVA_INSTALL_PATH)
        cmd+="./autogen.sh --prefix=%s --libdir=%s; make -j4; sudo make install"%(LIBVA_INSTALL_PREFIX, LIBVA_INSTALL_PATH)
        print(cmd)
        os.system(cmd)
        
        cmd ="cd %s/libva-utils; "%(WORKING_DIR)
        if install_overwrite == False:
            cmd+="export PKG_CONFIG_PATH=%s/pkgconfig:$PKG_CONFIG_PATH; "%(LIBVA_INSTALL_PATH)
            cmd+="export LIBRARY_PATH=%s:$LIBRARY_PATH; "%(LIBVA_INSTALL_PATH)
            cmd+="export C_INCLUDE_PATH=%s/include:$C_INCLUDE_PATH; "%(LIBVA_INSTALL_PREFIX)
            cmd+="export CPLUS_INCLUDE_PATH=%s/include:$CPLUS_INCLUDE_PATH; "%(LIBVA_INSTALL_PREFIX)

        cmd+="./autogen.sh --prefix=%s --libdir=%s; make -j4; sudo make install"%(LIBVA_INSTALL_PREFIX, LIBVA_INSTALL_PATH)
        print(cmd)
        os.system(cmd)

        print("")
        print("************************************************************************")
        print_info("Build and Install media driver", loglevelcode.INFO)
        print("************************************************************************")

        # Build and install media driver
        cmd= "rm -rf %s/gmmlib/build; "%(WORKING_DIR)
        cmd+= "mkdir -p %s/gmmlib/build; "%(WORKING_DIR)
        cmd+= "cd %s/gmmlib/build; "%(WORKING_DIR)
        if install_overwrite == False:
            cmd+="export LIBRARY_PATH=%s:$LIBRARY_PATH; "%(LIBVA_INSTALL_PATH)
            cmd+= "cmake ../ -DCMAKE_INSTALL_PREFIX=%s ; "%(LIBVA_INSTALL_PREFIX)
        else:
            cmd+= "cmake ../; "
        cmd+= "make -j4; "
        cmd+= "sudo make install; "
        print(cmd)
        os.system(cmd)

        cmd= "rm -rf %s/media_build; "%(WORKING_DIR)
        cmd+= "mkdir -p %s/media_build; "%(WORKING_DIR)
        cmd+= "cd %s/media_build; "%(WORKING_DIR)
        if install_overwrite == True:
            cmd+= "cmake ../media-driver; make -j4; sudo make install"
        else:
            cmd+="export PKG_CONFIG_PATH=%s/pkgconfig:$PKG_CONFIG_PATH; "%(LIBVA_INSTALL_PATH)
            cmd+="export LD_LIBRARY_PATH=%s:$LD_LIBRARY_PATH; "%(LIBVA_INSTALL_PATH)
            cmd+="export LIBRARY_PATH=%s:$LIBRARY_PATH; "%(LIBVA_INSTALL_PATH)
            cmd+="export C_INCLUDE_PATH=%s/include:$C_INCLUDE_PATH; "%(LIBVA_INSTALL_PREFIX)
            cmd+="export CPLUS_INCLUDE_PATH=%s/include:$CPLUS_INCLUDE_PATH; "%(LIBVA_INSTALL_PREFIX)
            cmd+= "cmake ../media-driver -DCMAKE_INSTALL_PREFIX=%s -DLIBVA_INSTALL_PATH=%s; "%(LIBVA_INSTALL_PREFIX, LIBVA_INSTALL_PATH)
            cmd+= "make -j8; "
            cmd+= "sudo env LD_LIBRARY_PATH=%s:$LD_LIBRARY_PATH LIBRARY_PATH=%s:$LIBRARY_PATH make install; "%(LIBVA_INSTALL_PATH, LIBVA_INSTALL_PATH)
        print(cmd)
        os.system(cmd)

        cmd= "export MFX_HOME=%s/onevpl_gpu; "%(WORKING_DIR)
        cmd+= "mkdir -p %s/onevpl_gpu/build; "%(WORKING_DIR)
        cmd+= "cd %s/onevpl_gpu/build; "%(WORKING_DIR)
        cmd+= "cmake ../ -DCMAKE_INSTALL_PREFIX=%s ; "%(LIBVA_INSTALL_PREFIX)
        cmd+= "make -j8; "
        cmd+= "sudo make install; "
        print(cmd)
        os.system(cmd)
    
        cmd= "export MFX_HOME=%s/onevpl; "%(WORKING_DIR)
        cmd+= "mkdir -p %s/onevpl/build; "%(WORKING_DIR)
        cmd+= "cd %s/onevpl/build; "%(WORKING_DIR)
        cmd+= "cmake ../ -DCMAKE_INSTALL_PREFIX=%s ; "%(LIBVA_INSTALL_PREFIX)
        cmd+= "make -j8; "
        cmd+= "sudo make install; "
        print(cmd)
        os.system(cmd)
