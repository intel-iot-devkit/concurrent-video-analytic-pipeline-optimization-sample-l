/******************************************************************************\
Copyright (c) 2005-2020, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

\**********************************************************************************/

#pragma once

#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"

#include <string>
#include <map>

#include <inference_engine.hpp>
#include <opencv2/core/core.hpp>
#include <gpu/gpu_context_api_va.hpp>

#include "sample_utils.h"

struct NetworkOptions
{
    NetworkOptions() : enableRemoteBlob(false), vaDpy(nullptr) {};
    bool enableRemoteBlob;
    VADisplay vaDpy;
};

//Note, please use mutex lock when try to create instance,
//modify members, or delete instance of NetworkInfo.
class NetworkInfo {
public:
    InferenceEngine::Core mIE;
    InferenceEngine::CNNNetwork mNetwork;
    InferenceEngine::ExecutableNetwork mExeNetwork;
    InferenceEngine::gpu::VAContext::Ptr mSharedVAContext;

    int Init(const std::string &modelPath, const std::string &device, const NetworkOptions &opt);

    InferenceEngine::InferRequest CreateNewInferRequest()
    {
        mRefNum++;
        return mExeNetwork.CreateInferRequest();
    }

    void ReleaseInferRequest()
    {
        mRefNum--;
    }

    bool isAllInferRequestReleased()
    {
        return (mRefNum <= 0);
    }

private:
    int mRefNum = 0;
};

class NetworkFactory {
public:
    static NetworkInfo *GetNetwork(const std::string &modelPath, const std::string &device, const NetworkOptions &opt);
    static void PutNetwork(NetworkInfo *);

private:
    static std::map<std::string, NetworkInfo *> mNetworks;
};

