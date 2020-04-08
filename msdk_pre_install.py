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

def fnParseCommandline():
    if len(sys.argv) == 1:		
        return "-all"
    elif len(sys.argv) > 3:
        return "not support"
        exit(0)
	
    if sys.argv[1] == "-h":
        print "[%s" % sys.argv[0] + " usage]"
        print "\t -h: display help"
        print "\t -all: install all components"
        print "\t -b BUILD_TARGET: build all components"
        print "\t -m : build msdk only"

        exit(0)

    return sys.argv[1]

if __name__ == "__main__":

    WORKING_DIR=run_cmd("pwd").strip()
    msg_tmp = "Working directory: " + WORKING_DIR
    print_info(msg_tmp, loglevelcode.INFO)

    BUILD_TARGET=""
    build_msdk = False 
    build_all = False

    cmd = fnParseCommandline()

    if cmd == "-b":
        BUILD_TARGET=sys.argv[2]
        build_all = True
        pre_install = False
        print "BUILD_TARGET=", BUILD_TARGET
    elif cmd == "-m":
        print_info("Build MSDK", loglevelcode.INFO)
        build_msdk = True
        pre_install = False
    else:
        pre_install = True

        
    if pre_install == True:
        print ""
        print "************************************************************************"
        print_info("Install required tools and create build environment.", loglevelcode.INFO)
        print "************************************************************************"
       
        # Install the necessary tools
        
        print "Please input the sudo password to proceed\n"
        cmd ="sudo apt-get -y install git libssl-dev dh-autoreconf cmake libgl1-mesa-dev libpciaccess-dev build-essential curl imagemagick unzip yasm libjpeg-dev libavcodec-dev libavutil-dev libavformat-dev;"
        cmd+="sudo apt-get -y install libopencv-dev checkinstall pkg-config libgflags-dev"
        os.system(cmd)

        print ""
        print "************************************************************************"
        print_info("Pull all the source code.", loglevelcode.INFO)
        print "************************************************************************"

        # Pull all the source code
        print "libva"
        if not os.path.exists("%s/libva"%(WORKING_DIR)):
            cmd = "cd %s; git clone https://github.com/intel/libva.git;"%(WORKING_DIR) 
            cmd+= "cd libva;"
            cmd+="git checkout 5f693d9e603e0e83928cec67c30b6ac902d7aa85;" # 04/12/18
            print cmd
            os.system(cmd);

        print "libva-utils"
        if not os.path.exists("%s/libva-utils"%(WORKING_DIR)):
            cmd = "cd %s; git clone https://github.com/intel/libva-utils.git;"%(WORKING_DIR)
            cmd += "cd libva-utils;"
            cmd+="git checkout 8ea1eba433dcbceb0e5dcb54b8e3f984987f7a17; " # 03/20/18
            print cmd
            os.system(cmd);

        print "media-driver"
        if not os.path.exists("%s/media-driver"%(WORKING_DIR)): 
            cmd = "cd %s; git clone https://github.com/intel/media-driver.git; "%(WORKING_DIR)
            cmd += "cd media-driver;"
            cmd+= "git checkout 12b7fcded6c74377ecf57eb8258f5e3d55ca722e" # 04/16/18
            print cmd
            os.system(cmd);

        print "gmmlib"
        if not os.path.exists("%s/gmmlib"%(WORKING_DIR)): 
            cmd = "cd %s; git clone https://github.com/intel/gmmlib.git; "%(WORKING_DIR)
            cmd += "cd gmmlib;"
            cmd+= "git checkout ebfcfd565031dbd7b45089d9054cd44a501f14a9" # 04/05/18
            print cmd
            os.system(cmd);

        print "MediaSDK"
        if not os.path.exists("%s/MediaSDK"%(WORKING_DIR)): 
            cmd = "cd %s; git clone https://github.com/Intel-Media-SDK/MediaSDK.git; "%(WORKING_DIR)
            cmd+= "cd MediaSDK;"
            cmd+= "git checkout f8f6646afcaec410ca037f0176814dd9d7ca4900" # 04/16/18
            print cmd
            os.system(cmd);


    if build_all == True:
        print ""
        print "************************************************************************"
        print_info("Build and Install libVA", loglevelcode.INFO)
        print "************************************************************************"

        # Build and install libVA including the libVA utils for vainfo.
        # libVA origin:fbf7138389f7d6adb6ca743d0ddf2dbc232895f6 (011118), libVA utils origin: 7b85ff442d99c233fb901a6fe3407d5308971645 (011118)
        cmd ="cd %s/libva; "%(WORKING_DIR)
        cmd+="./autogen.sh --prefix=/usr --libdir=/usr/lib/x86_64-linux-gnu; make -j4; sudo make install"
        print cmd
        os.system(cmd)

        cmd ="cd %s/libva-utils; "%(WORKING_DIR)
        cmd+="./autogen.sh --prefix=/usr --libdir=/usr/lib/x86_64-linux-gnu; make -j4; sudo make install"
        print cmd
        os.system(cmd)

        print ""		
        print "************************************************************************"
        print_info("Build and Install media driver", loglevelcode.INFO)
        print "************************************************************************"

        # Build and install media driver
        cmd = "mkdir -p %s/gmmlib/build; "%(WORKING_DIR)
        cmd+= "cd %s/gmmlib/build; "%(WORKING_DIR)
        cmd+= "cmake ../;"
        cmd+= "make -j4;"
        cmd+= "sudo make install;"
        print cmd
        os.system(cmd)

        cmd = "mkdir -p %s/media-driver/build; "%(WORKING_DIR)
        cmd+= "cd %s/media-driver/build; "%(WORKING_DIR)
        cmd+= "cmake ../; "
        cmd+= "make -j4; sudo make install"
        print cmd
        os.system(cmd)

    if build_all == True or build_msdk == True:
        print ""
        print "************************************************************************"
        print_info("Build and Install Media SDK and samples", loglevelcode.INFO)
        print "************************************************************************"

        if not os.path.exists("%s/MediaSDK"%(WORKING_DIR)): 
            print "MediaSDK source code doen't exist!"
            sys.exit() 

        # Build and install Media SDK library and samples
        # MediaSDK origin: 22dae397ae17d29b4734adb0e5a998f32a9c25b2 (022218)
        cmd ="cd %s/MediaSDK; "%(WORKING_DIR)
        cmd+="mkdir -p build; "
        cmd+="cd build; "
        cmd+="cmake ../; "
        cmd+="make -j4; "
        cmd+="sudo make install; "
        print cmd
        os.system(cmd)

    if pre_install == True:

        if not os.path.exists("%s/MediaSDK"%(WORKING_DIR)): 
            print "MediaSDK source code doen't exist!"
            sys.exit()

        r=subprocess.check_output("grep -c \"export LIBVA_DRIVER_NAME=iHD\" ~/.bashrc", shell=True)

        if r >= 1:
            sys.exit()

        print ""
        print "************************************************************************"
        print_info("Set environment variables. ", loglevelcode.INFO)
        print "************************************************************************"
        cmd = "echo 'export MFX_HOME=/opt/intel/mediasdk' >> ~/.bashrc; "
        cmd+= "echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu:/usr/lib' >> ~/.bashrc; "
        cmd+= "echo 'export LIBVA_DRIVERS_PATH=/usr/lib/x86_64-linux-gnu/dri' >> ~/.bashrc; "
        cmd+= "echo 'export LIBVA_DRIVER_NAME=iHD' >> ~/.bashrc"
        print cmd
        os.system(cmd)

        cmd = "sudo echo '/usr/lib/x86_64-linux-gnu' > /etc/ld.so.conf.d/libdrm-intel.conf; "
        cmd+= "sudo echo '/usr/lib' >> /etc/ld.so.conf.d/libdrm-intel.conf; "
        cmd+= "sudo ldconfig"
        print cmd
        os.system(cmd)

        print "************************************************************************"
        print "    Done, all installation, please reboot system !!! "
        print "************************************************************************"

