/******************************************************************************\
Copyright (c) 2005-2020, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

This sample was distributed or derived from the Intel's Media Samples package.
The original version of this sample may be obtained from https://software.intel.com/en-us/intel-media-server-studio
or https://software.intel.com/en-us/media-client-solutions-support.
\**********************************************************************************/

#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <iostream>

template <typename T>
class SampleQueue
{
    public:
        SampleQueue();
        ~SampleQueue();

        T& front();
        void pop_front();
        void push_back(const T& item);
        int size();
        bool empty();

    private:
        std::deque<T> m_queue;
        std::mutex m_rw_lock;
        std::condition_variable m_cond;
}; 

template <typename T>
SampleQueue<T>::SampleQueue(){}

template <typename T>
SampleQueue<T>::~SampleQueue(){ std::cout<<"Deconstrution is called"<<std::endl;}

template <typename T>
T& SampleQueue<T>::front()
{
    std::unique_lock<std::mutex> mlock(m_rw_lock);
    while (m_queue.empty())
    {
        m_cond.wait(mlock);
    }
    return m_queue.front();
}

template <typename T>
void SampleQueue<T>::pop_front()
{
    std::unique_lock<std::mutex> mlock(m_rw_lock);
    while (m_queue.empty())
    {
        m_cond.wait(mlock);
    }
    return m_queue.pop_front();
}     

template <typename T>
void SampleQueue<T>::push_back(const T& item)
{
    std::unique_lock<std::mutex> mlock(m_rw_lock);
    m_queue.push_back(item);
    mlock.unlock();     
    m_cond.notify_one();

}

template <typename T>
int SampleQueue<T>::size()
{
    std::unique_lock<std::mutex> mlock(m_rw_lock);
    int size = m_queue.size();
    mlock.unlock();
    return size;
}
