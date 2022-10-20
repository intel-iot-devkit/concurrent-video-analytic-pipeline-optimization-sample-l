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
#include <ie/gpu/gpu_context_api_va.hpp>
#include <intel_gpu/properties.hpp>
#include "sample_defs.h"
#include <samples/common.hpp>
#include "network_factory.hpp"

//using namespace InferenceEngine;
using namespace ov;
using namespace ov::preprocess;

int NetworkInfo::Init(const std::string &modelPath, const std::string &device, const NetworkOptions &opt)
{
    ov::Core core;
    int input_height=300;
    int input_width = 300;

    ov::Layout desiredLayout = {"NHWC"};
       // -------- Step 2. Read a model --------
    std::shared_ptr<ov::Model> model = core.read_model(modelPath);

        // -------- Step 3. Configure preprocessing  --------
       // PrePostProcessor ppp = PrePostProcessor(model);
    ov::Output<ov::Node> input = model->input();
    input_tensor_name = input.get_any_name();
    ov::Layout modelLayout = ov::layout::get_layout(input);
    
    if (modelLayout.empty())
        modelLayout = {"NCHW"};

    mInputShape = input.get_shape();
    bool   m_new_model = false;
    if ( model->outputs().size() == 1)
    {
         ov::Output<ov::Node> output = model->output();
         output_tensor_name = output.get_any_name();
     
         ov::Shape outputDims = output.get_shape();
         SetoutputDims(outputDims);

         m_max_detections_count = outputDims[2];
         m_object_size = outputDims[3];
         if (m_object_size != 7 && m_object_size != 117 ) {
             throw std::runtime_error("Face Detection network output layer should have 7 as a last dimension");
         }
         if (outputDims.size() != 4 && outputDims.size() != 2 ) {
             throw std::runtime_error("Face Detection network output should have 4 dimensions, but had " +
                 std::to_string(outputDims.size()));
         }
    } else {
         ov::OutputVector outputs = model->outputs();
         auto cmp = [&](const ov::Output<ov::Node>& output) {
             /* this info will be used in HP and vd detection function
                std::cout << " output name : " << output.get_any_name() << std::endl;
                ov::Shape outputDims = output.get_shape();
                std::cout << outputDims[0] << " " << outputDims[1] << " " << outputDims[2] << " " << outputDims[3] << std::endl;
              */
             return output.get_any_name() == "data"; };
         auto it = std::find_if(outputs.begin(), outputs.end(), cmp);
         if (it != outputs.end())
         m_new_model = true;
    }

    ov::preprocess::PrePostProcessor ppp(model);

    if (opt.enableRemoteBlob)
    {
        if (mInputShape.size() != 4)
        {
            std::cout<<"Input mInputShape size isn't 4 ! Remote blob only support fd inference."<<std::endl;
            return -1;
        }
        static constexpr auto surface = "GPU_SURFACE";
        ppp.input().tensor().set_element_type(ov::element::u8)
            .set_color_format(ov::preprocess::ColorFormat::NV12_TWO_PLANES, {"y", "uv"})
            .set_memory_type(ov::intel_gpu::memory_type::surface);
        ppp.input().preprocess().convert_color(ov::preprocess::ColorFormat::BGR);

        ppp.input().model().set_layout("NCHW");
        model = ppp.build();

        mVAContext = new ov::intel_gpu::ocl::VAContext(core, opt.vaDpy);
        mCompiled_model = core.compile_model(model, *mVAContext);

        auto input = model->get_parameters().at(0);
        auto input1 = model->get_parameters().at(1);
        auto shape = input->get_shape();
        auto height = shape[2];
        auto width = shape[1];

        for (int i = 0 ; i < 16; i++) {
            auto infer_request = mCompiled_model.create_infer_request();
            reqVector.push_back(std::make_shared<ov::InferRequest>(infer_request));
        }
    }
    else
    {
        ppp.input().tensor()
            .set_element_type(ov::element::u8)
            .set_layout(desiredLayout);

        ppp.input().preprocess()
            .convert_layout(modelLayout)
            .convert_element_type(ov::element::f32);
        ppp.input().model().set_layout(modelLayout);

        model = ppp.build();

        mVAContext = nullptr;
        mCompiled_model = core.compile_model(model, device);
        /* support most 16 channel detection */
        for (int i = 0 ; i < 16; i++) {
            reqVector.push_back(std::make_shared<ov::InferRequest>(mCompiled_model.create_infer_request()));
        }
    }


    mModel = model;

    std::cout<<"Loading model "<<modelPath<<" to device "<<device<<std::endl;
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


