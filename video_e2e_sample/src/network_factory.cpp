/******************************************************************************\
Copyright (c) 2005-2020, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

\**********************************************************************************/



#include <mutex>

#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"

#include <cldnn/cldnn_config.hpp>
#include "sample_defs.h"
#include <samples/common.hpp>
#include "network_factory.hpp"

using namespace InferenceEngine;

//static NetworkFactory::mNetworks = {};
int NetworkInfo::Init(const std::string &modelPath, const std::string &device, const NetworkOptions &opt)
{
    Core ie;
    std::string str_ir_bin = fileNameNoExt(modelPath) + ".bin";
    mNetwork = ie.ReadNetwork(modelPath, str_ir_bin);
    mNetwork.setBatchSize(1);
    
    auto inputP = mNetwork.getInputsInfo().begin();
    if (inputP == mNetwork.getInputsInfo().end())
    {
        return -1;
    } 
    InferenceEngine::InputInfo::Ptr inputInfo = inputP->second;

    inputInfo->setPrecision(Precision::U8);
    inputInfo->getInputData()->setLayout(Layout::NCHW);

    InferenceEngine::OutputsDataMap outputInfo = mNetwork.getOutputsInfo();
    auto outputBlobsIt = outputInfo.begin();
    if (outputBlobsIt == outputInfo.end())
    {
        return -1;
    }

    DataPtr& output = outputInfo.begin()->second;

    output->setPrecision(Precision::FP32);
    output->setLayout(Layout::NCHW);

    if (opt.enableRemoteBlob)
    {

        inputInfo->getPreProcess().setColorFormat(ColorFormat::NV12);
        mSharedVAContext = gpu::make_shared_context(ie, device, opt.vaDpy);
        mExeNetwork = mIE.LoadNetwork(mNetwork,
                mSharedVAContext,
                { { InferenceEngine::CLDNNConfigParams::KEY_CLDNN_NV12_TWO_INPUTS,
                PluginConfigParams::YES } });
    }
    else
    {
        mExeNetwork = ie.LoadNetwork(mNetwork, device);;
    }
    return 0;
}

void NetworkFactory::PutNetwork(NetworkInfo *network)
{
    if (!network)
    {
        return;
    }
    static std::mutex initLock;
    std::lock_guard<std::mutex> lock(initLock);
    network->ReleaseInferRequest();
    if (network->isAllInferRequestReleased())
    {
        delete network;
    }
    return;
}

NetworkInfo *NetworkFactory::GetNetwork(const std::string &modelPath, const std::string &device, const NetworkOptions &opt)
{
    static std::map<std::string, NetworkInfo *> mNetworks;
    static std::mutex initLock;
    std::lock_guard<std::mutex> lock(initLock);

    std::string key = modelPath + device;
    auto it = mNetworks.find(key);
    //Multiple sessions can share the network instance with same model name and device
    if (it != mNetworks.end())
    {
        return it->second;
    }
    else
    {
        NetworkInfo *network = new NetworkInfo;
        if (network->Init(modelPath, device, opt) < 0)
        {
            delete network;
            return nullptr;
        }
        else
        {
            auto pr = std::make_pair(key, network);
            mNetworks.insert(pr);
            return network;
        }
    }
}


