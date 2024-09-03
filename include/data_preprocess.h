// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <vector>

#include "dnn_node/dnn_node_data.h"

#define ALIGNED_2E(w, alignment) \
  ((static_cast<uint32_t>(w) + (alignment - 1U)) & (~(alignment - 1U)))
#define ALIGN_4(w) ALIGNED_2E(w, 4U)
#define ALIGN_8(w) ALIGNED_2E(w, 8U)
#define ALIGN_16(w) ALIGNED_2E(w, 16U)
#define ALIGN_64(w) ALIGNED_2E(w, 64U)

using hobot::dnn_node::DNNTensor;

#ifndef DATA_PREPROCESS_H_
#define DATA_PREPROCESS_H_

int GenScaleBox(std::shared_ptr<std::vector<hbDNNRoi>> &rois,
            std::vector<std::vector<float>> &boxes,
            float ratio);

class InputPreProcessor {
 public:
  InputPreProcessor() {}
  ~InputPreProcessor() {}

  /**
   * @brief 图片数据预处理
   * @param in_data 数据
   * @param tensor_properties DNN输入tensor的属性信息
   * @return DNNTensor向量
   */
  static std::shared_ptr<hobot::dnn_node::DNNTensor> GetBoxTensor(
        const std::vector<std::vector<float>> & boxes,
        hbDNNTensorProperties tensor_properties);
};

#endif // DATA_PREPROCESS_H_