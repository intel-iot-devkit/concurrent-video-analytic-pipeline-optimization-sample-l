// Copyright (C) 2018-2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

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

#include "mot_core.hpp"

#include <memory>
#include <vector>


///
/// \brief The KuhnMunkres class
///
/// Solves the assignment problem.
///
class KuhnMunkres {
public:
    KuhnMunkres();

    ///
    /// \brief Solves the assignment problem for given dissimilarity matrix.
    /// It returns a vector that where each element is a column index for
    /// corresponding row (e.g. result[0] stores optimal column index for very
    /// first row in the dissimilarity matrix).
    /// \param dissimilarity_matrix CV_32F dissimilarity matrix.
    /// \return Optimal column index for each row. -1 means that there is no
    /// column for row.
    ///
    std::vector<size_t> Solve(const cv::Mat &dissimilarity_matrix);

private:
    static constexpr int kStar = 1;
    static constexpr int kPrime = 2;

    cv::Mat dm_;
    cv::Mat marked_;
    std::vector<cv::Point> points_;

    std::vector<int> is_row_visited_;
    std::vector<int> is_col_visited_;

    int n_;

    void TrySimpleCase();
    bool CheckIfOptimumIsFound();
    cv::Point FindUncoveredMinValPos();
    void UpdateDissimilarityMatrix(float val);
    int FindInRow(int row, int what);
    int FindInCol(int col, int what);
    void Run();
};

