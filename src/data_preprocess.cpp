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

#include "include/data_preprocess.h"

std::shared_ptr<DNNTensor> InputPreProcessor::GetBoxTensor(
    const std::vector<std::vector<float>> & boxes,
    hbDNNTensorProperties tensor_properties) {

  int src_elem_size = 4;

  int num_boxes = tensor_properties.alignedShape.dimensionSize[1];

  auto *mem = new hbSysMem;
  hbSysAllocCachedMem(mem, num_boxes * 4 * src_elem_size);
  //内存初始化
  memset(mem->virAddr, 0, num_boxes * 4 * src_elem_size);
  auto *hb_mem_addr = reinterpret_cast<uint8_t *>(mem->virAddr);

  for (auto &box: boxes) {
    const uint8_t *data = reinterpret_cast<const uint8_t *>(box.data());
    memcpy(hb_mem_addr, data, 4 * src_elem_size);
    hb_mem_addr += 4 * src_elem_size;
  }

  hbSysFlushMem(mem, HB_SYS_MEM_CACHE_CLEAN);
  auto input_tensor = new DNNTensor;

  input_tensor->properties = tensor_properties;
  input_tensor->sysMem[0].virAddr = reinterpret_cast<void *>(mem->virAddr);
  input_tensor->sysMem[0].phyAddr = mem->phyAddr;
  input_tensor->sysMem[0].memSize = num_boxes * 4 * src_elem_size;

  return std::shared_ptr<DNNTensor>(
      input_tensor, [mem](DNNTensor *input_tensor) {
        // Release memory after deletion
        hbSysFreeMem(mem);
        delete mem;
        delete input_tensor;
      });
}


int GenScaleBox(std::shared_ptr<std::vector<hbDNNRoi>> &rois,
                                    std::vector<std::vector<float>> &boxes,
                                    float ratio) {
  for (auto it = rois->begin(); it != rois->end(); ++it) {
    // 访问每个hbDNNRoi实例
    hbDNNRoi roi = *it;
    std::vector<float> box;
    float x1 = static_cast<float>(roi.left / ratio);
    float y1 = static_cast<float>(roi.top / ratio);
    float x2 = static_cast<float>(roi.right / ratio);
    float y2 = static_cast<float>(roi.bottom / ratio);
    box.push_back(x1);
    box.push_back(y1);
    box.push_back(x2);
    box.push_back(y2);
    boxes.push_back(box);
  }
  return 0;
}