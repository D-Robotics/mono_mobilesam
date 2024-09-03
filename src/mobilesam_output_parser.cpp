
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

#include "include/mobilesam_output_parser.h"

int32_t MobileSamOutputParser::Parse(
    std::shared_ptr<DnnParserResult> &result,
    const int resized_img_h,
    const int resized_img_w,
    const int model_h,
    const int model_w,
    std::vector<std::shared_ptr<DNNTensor>>& output_tensors,
    float ratio) {

  if (!result) {
    result = std::make_shared<DnnParserResult>();
  }
  int ret = PostProcess(output_tensors, 
                        resized_img_h,
                        resized_img_w,
                        model_h,
                        model_w,
                        result->perception,
                        ratio);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("SamOutputParser"),
                "postprocess return error, code = %d",
                ret);
  }

  return ret;
}

int32_t MobileSamOutputParser::PostProcess(
    std::vector<std::shared_ptr<DNNTensor>>& output_tensors,
    const int resized_img_h,
    const int resized_img_w,
    const int model_h,
    const int model_w,
    Perception& perception,
    float ratio) {
    
  if (output_tensors.size() == 0) {
    return -1;
  }

  perception.type = Perception::SEG;
  hbSysFlushMem(&(output_tensors[1]->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  float* data = reinterpret_cast<float*>(output_tensors[1]->sysMem[0].virAddr);
  int ret = GenMask(data, resized_img_h, resized_img_w, model_h, model_w, perception, ratio);

  hbSysFreeMem(&(output_tensors[0]->sysMem[0]));
  hbSysFreeMem(&(output_tensors[1]->sysMem[0]));

  return 0;
}

int32_t MobileSamOutputParser::GenMask(const float* mask,
                                        const int resized_img_h,
                                        const int resized_img_w,
                                        const int model_h,
                                        const int model_w,
                                        Perception& perception,
                                        float ratio) {

  perception.type = Perception::SEG;

  int channel = 4;

  float valid_h_ratio = static_cast<float>(resized_img_h) / static_cast<float>(model_h);
  float valid_w_ratio = static_cast<float>(resized_img_w) / static_cast<float>(model_w);

  int valid_h = static_cast<int>(valid_h_ratio * output_height_);
  int valid_w = static_cast<int>(valid_w_ratio * output_width_);

  int stride = channel * output_height_ * output_width_;
  std::vector<cv::Mat> parsing_imgs;
  for (int n = 0; n < num_classes_; n++) {
    cv::Mat parsing_img(valid_h, valid_w, CV_32FC1);
    float *parsing_img_ptr = parsing_img.ptr<float>();
    for (int h = 0; h < valid_h; h++) {
      for (int w = 0; w < valid_w; w++) {
        int offect = h * output_width_ + w;
        const float* data = mask + n * stride + offect;
        *parsing_img_ptr++ = data[0];
      }
    }
    parsing_imgs.push_back(parsing_img);
  }

  valid_h = static_cast<int>(static_cast<float>(resized_img_h) * ratio);
  valid_w = static_cast<int>(static_cast<float>(resized_img_w) * ratio);
  cv::Size size(valid_w, valid_h);

  for (auto &parsing_img: parsing_imgs) {
    // resize parsing image
    cv::resize(parsing_img, parsing_img, size, 0, 0, cv::INTER_LINEAR);
  }

  perception.seg.data.resize(valid_h * valid_w);
  perception.seg.seg.resize(valid_h * valid_w);

  perception.seg.valid_h = valid_h;
  perception.seg.valid_w = valid_w;
  perception.seg.height = static_cast<int>(model_h * valid_h_ratio);
  perception.seg.width = static_cast<int>(model_w * valid_w_ratio);
  perception.seg.channel = channel;
  perception.seg.num_classes = num_classes_ + 1;

  for (int n = 0; n < num_classes_; n++) {
    auto &parsing_img = parsing_imgs[n];
    float *parsing_img_ptr = parsing_img.ptr<float>();
    for (int h = 0; h < valid_h; h++) {
      for (int w = 0; w < valid_w; w++) {
        int offect = h * valid_w + w;
        int top_index = -1;
        if (n == 0) {
          top_index = 0;
        }
        if (*parsing_img_ptr++ > mask_threshold_) {
          top_index = n + 1;
        }
        if (top_index != -1) {
          perception.seg.seg[h * valid_w + w] = top_index;
          perception.seg.data[h * valid_w + w] = static_cast<float>(top_index);
        }  
      }
    }
  }
  return 0;
}

int32_t MobileSamOutputParser::UpdateBox(std::vector<float> &box,
                    Perception& perception) {

  int valid_h = perception.seg.valid_h;
  int valid_w = perception.seg.valid_w;

  int count = 0;
  for (int h = 0; h < valid_h; h++) {
    int w = static_cast<int>(box[0]);
    count += perception.seg.data[h * valid_w + w] != 0 ? 1 : 0;
  }
  box[0] += count > 5 ? -1 : 1;

  count = 0;
  for (int h = 0; h < valid_h; h++) {
    int w = static_cast<int>(box[2]);
    count += perception.seg.data[h * valid_w + w] != 0 ? 1 : 0;
  }
  box[2] += count > 5 ? 1 : -1;

  count = 0;
  for (int w = 0; w < valid_w; w++) {
    int h = static_cast<int>(box[1]);
    count += perception.seg.data[h * valid_w + w] != 0 ? 1 : 0;
  }
  box[1] += count > 5 ? -1 : 1;

  count = 0;
  for (int w = 0; w < valid_w; w++) {
    int h = static_cast<int>(box[3]);
    count += perception.seg.data[h * valid_w + w] != 0 ? 1 : 0;
  }
  box[3] += count > 5 ? 1 : -1;

  return 0;
}

int RenderSeg(cv::Mat &mat, Parsing &seg, std::string& saving_path) {
  static uint8_t bgr_putpalette[] = {
      0, 0, 0, 128, 64,  128, 244, 35,  232, 70,  70,  70,  102, 102, 156, 190, 153, 153,
      153, 153, 153, 250, 170, 30,  220, 220, 0,   107, 142, 35,  152, 251, 152,
      0,   130, 180, 220, 20,  60,  255, 0,   0,   0,   0,   142, 0,   0,   70,
      0,   60,  100, 0,   80,  100, 0,   0,   230, 119, 11,  32};

  int parsing_width = seg.valid_w;
  int parsing_height = seg.valid_h;
  cv::Mat parsing_img(parsing_height, parsing_width, CV_8UC3);
  uint8_t *parsing_img_ptr = parsing_img.ptr<uint8_t>();

  for (int h = 0; h < parsing_height; ++h) {
    for (int w = 0; w < parsing_width; ++w) {
      auto id = seg.seg[h * parsing_width + w];
      *parsing_img_ptr++ = bgr_putpalette[id * 3];
      *parsing_img_ptr++ = bgr_putpalette[id * 3 + 1];
      *parsing_img_ptr++ = bgr_putpalette[id * 3 + 2];
    }
  }

  // resize parsing image
  cv::resize(parsing_img, parsing_img, mat.size(), 0, 0);

  // alpha blending
  float alpha_f = 0.5;
  cv::Mat dst;
  addWeighted(mat, alpha_f, parsing_img, 1 - alpha_f, 0.0, dst);
  mat = std::move(dst);

  RCLCPP_INFO(rclcpp::get_logger("MobileSam"),
              "Draw result to file: %s",
              saving_path.c_str());
  cv::imwrite(saving_path, mat);
  return 0;
}