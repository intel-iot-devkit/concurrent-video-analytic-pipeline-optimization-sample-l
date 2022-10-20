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
#include <queue>
#include <vector>
#include <map>

#include "openvino/openvino.hpp"
#include <inference_engine.hpp>
#include <opencv2/core/core.hpp>
#include <intel_gpu/ocl/va.hpp>

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
    ov::CompiledModel mCompiled_model;
    ov::CompiledModel* mCompiled_model2;
    std::shared_ptr<ov::CompiledModel> mCompiled_model1;
    std::vector<std::shared_ptr<ov::InferRequest>>  reqVector;
    std::shared_ptr<ov::Model> mModel;

    //to be removed
    //InferenceEngine::Core mIE;
    InferenceEngine::CNNNetwork mNetwork;
    InferenceEngine::ExecutableNetwork mExeNetwork;

    std::shared_ptr<ov::InferRequest> m_request;
    std::string input_tensor_name;
    std::string output_tensor_name;
    int m_object_size;
    int m_max_detections_count;
    ov::Shape mInputShape;
    ov::Shape outputDims_;

    //for remote blob
    ov::intel_gpu::ocl::VAContext *mVAContext;
    int Init(const std::string &modelPath, const std::string &device, const NetworkOptions &opt);

    NetworkInfo():
        mVAContext(nullptr)
    {

    }

    ~NetworkInfo()
    {
        if (mVAContext)
            delete  mVAContext;
    }


    InferenceEngine::InferRequest CreateNewInferRequest()
    {
        mRefNum++;
        return mExeNetwork.CreateInferRequest();
    }
    std::shared_ptr<ov::InferRequest> CreateNewInferRequest2()
    {
        mRefNum++;
        return reqVector[mRefNum-1];
    }

    void ReleaseInferRequest()
    {
        mRefNum--;
    }

    bool isAllInferRequestReleased()
    {
        return (mRefNum <= 0);
    }

    ov::Shape GetoutputDims()
    {
        return outputDims_;
    }

    void SetoutputDims(ov::Shape outputDims )
    {
        outputDims_ = outputDims;
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

