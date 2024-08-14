// Copyright (c) 2024，Horizon Robotics.
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

#include <fstream>
#include <math.h>
#include <memory>
#include <unistd.h>
#include <utility>

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "include/mobilesam_node.h"

builtin_interfaces::msg::Time ConvertToRosTime(
    const struct timespec& time_spec) {
  builtin_interfaces::msg::Time stamp;
  stamp.set__sec(time_spec.tv_sec);
  stamp.set__nanosec(time_spec.tv_nsec);
  return stamp;
}

int CalTimeMsDuration(const builtin_interfaces::msg::Time& start,
                      const builtin_interfaces::msg::Time& end) {
  return (end.sec - start.sec) * 1000 + end.nanosec / 1000 / 1000 -
         start.nanosec / 1000 / 1000;
}


template<typename T>
int readbinary(const std::string &filename, const T* &dataOut) {
  std::cout << "readbinary: " << filename + ".bin" << std::endl;
  std::ifstream ifs(filename + ".bin", std::ios::in | std::ios::binary);
  if (!ifs) {
    return -1;
  }
  ifs.seekg(0, std::ios::end);
  int len = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  char* data = new char[len];
  ifs.read(data, len);
  dataOut = reinterpret_cast<const T *>(data);
  return len / sizeof(T);
}

MobileSamNode::MobileSamNode(const std::string& node_name,
                               const NodeOptions& options)
    : DnnNode(node_name, options) {
  this->declare_parameter<int>("cache_len_limit", cache_len_limit_);
  this->declare_parameter<int>("dump_render_img", dump_render_img_);
  this->declare_parameter<int>("feed_type", feed_type_);
  this->declare_parameter<std::string>("image", image_file_);
  this->declare_parameter<int>("is_regular_box", is_regular_box_);
  this->declare_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->declare_parameter<int>("is_sync_mode", is_sync_mode_);
  this->declare_parameter<std::string>("ai_msg_pub_topic_name",
                                       ai_msg_pub_topic_name_);
  this->declare_parameter<std::string>("ai_msg_sub_topic_name",
                                       ai_msg_sub_topic_name_);
  this->declare_parameter<std::string>("ros_img_sub_topic_name",
                                       ros_img_sub_topic_name_);

  this->get_parameter<int>("cache_len_limit", cache_len_limit_);
  this->get_parameter<int>("dump_render_img", dump_render_img_);
  this->get_parameter<int>("feed_type", feed_type_);
  this->get_parameter<std::string>("image", image_file_);
  this->get_parameter<int>("is_regular_box", is_regular_box_);
  this->get_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->get_parameter<int>("is_sync_mode", is_sync_mode_);
  this->get_parameter<std::string>("ai_msg_pub_topic_name",
                                   ai_msg_pub_topic_name_);
  this->get_parameter<std::string>("ai_msg_sub_topic_name",
                                   ai_msg_sub_topic_name_);
  this->get_parameter<std::string>("ros_img_sub_topic_name",
                                   ros_img_sub_topic_name_);

  std::stringstream ss;
  ss << "Parameter:"
     << "\n cache_len_limit: " << cache_len_limit_
     << "\n dump_render_img: " << dump_render_img_
     << "\n feed_type(0:local, 1:sub): " << feed_type_
     << "\n image: " << image_file_
     << "\n is_regular_box: " << is_regular_box_
     << "\n is_shared_mem_sub: " << is_shared_mem_sub_
     << "\n is_sync_mode: " << is_sync_mode_
     << "\n ai_msg_pub_topic_name: " << ai_msg_pub_topic_name_
     << "\n ai_msg_sub_topic_name: " << ai_msg_sub_topic_name_
     << "\n ros_img_sub_topic_name: " << ros_img_sub_topic_name_;
  RCLCPP_WARN(rclcpp::get_logger("mono_mobilesam"), "%s", ss.str().c_str());

  if(LoadModels() != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_mobilesam"), "Load Model Failed!");
    return;
  }
  if(SetNodePara() != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_mobilesam"), "Init Node Para Failed!");
    return;
  }


  if (0 == feed_type_) {
    dump_render_img_ = 1;
    FeedFromLocal();
  } else {
    ai_msg_manage_ = std::make_shared<AiMsgManage>();
    ai_msg_subscription_ =
        this->create_subscription<ai_msgs::msg::PerceptionTargets>(
            ai_msg_sub_topic_name_,
            10,
            std::bind(
                &MobileSamNode::AiMsgProcess, this, std::placeholders::_1));
                
    msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
        ai_msg_pub_topic_name_, 10);

    predict_task_ = std::make_shared<std::thread>(
        std::bind(&MobileSamNode::RunPredict, this));

    if (is_shared_mem_sub_) {
#ifdef SHARED_MEM_ENABLED
      RCLCPP_WARN(rclcpp::get_logger("mono_mobilesam"),
                  "Create hbmem_subscription with topic_name: %s",
                  sharedmem_img_topic_name_.c_str());
      sharedmem_img_subscription_ =
          this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
              sharedmem_img_topic_name_,
              rclcpp::SensorDataQoS(),
              std::bind(&MobileSamNode::SharedMemImgProcess,
                        this,
                        std::placeholders::_1));
#else
      RCLCPP_ERROR(rclcpp::get_logger("mono_mobilesam"), "Unsupport shared mem");
#endif
    } else {
      RCLCPP_WARN(rclcpp::get_logger("mono_mobilesam"),
                  "Create subscription with topic_name: %s",
                  ros_img_sub_topic_name_.c_str());
      ros_img_subscription_ =
          this->create_subscription<sensor_msgs::msg::Image>(
              ros_img_sub_topic_name_,
              10,
              std::bind(
                  &MobileSamNode::RosImgProcess, this, std::placeholders::_1));
    }
  }

}

MobileSamNode::~MobileSamNode() {
  for (auto &packed_dnn_handle: packed_dnn_handles_) {
    hbDNNRelease(packed_dnn_handle);
  }
}

int MobileSamNode::SetNodePara() {
  hbDNNTensorProperties tensor_properties;
  models_[0]->GetInputTensorProperties(tensor_properties, 0);
  if (tensor_properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    model_input_height_ = tensor_properties.alignedShape.dimensionSize[2];
    model_input_width_ = tensor_properties.alignedShape.dimensionSize[3];
  } else if (tensor_properties.tensorLayout == HB_DNN_LAYOUT_NHWC) {
    model_input_height_ = tensor_properties.alignedShape.dimensionSize[1];
    model_input_width_ = tensor_properties.alignedShape.dimensionSize[2];
  }

  models_[1]->GetOutputTensorProperties(tensor_properties, 1);
  int output_height = tensor_properties.alignedShape.dimensionSize[2];
  int output_width = tensor_properties.alignedShape.dimensionSize[3];
  int num_classes = tensor_properties.alignedShape.dimensionSize[0];

  output_parser_ = std::make_shared<MobileSamOutputParser>(output_height, output_width, num_classes, 0.1);
  return 0;
}

int MobileSamNode::RunMulti(std::vector<std::shared_ptr<DNNTensor>>& inputs,
                      const std::shared_ptr<DnnNodeOutput> &output,
                      const bool is_sync_mode) {

  output->rt_stat = std::make_shared<hobot::dnn_node::DnnNodeRunTimeStat>();
  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  output->rt_stat->infer_timespec_start = time_now;

  int32_t const input_size{static_cast<int32_t>(inputs.size())};
  std::vector<hbDNNTensor> input_dnn_tensors;
  input_dnn_tensors.resize(static_cast<size_t>(input_size));     
  for (int32_t i{0}; i < input_size; ++i) {
    if (inputs[i] == nullptr) {
      RCLCPP_ERROR(rclcpp::get_logger("mono_mobilesam"), 
        "inputs[%d] is null", i);
      return -1;
    }
    input_dnn_tensors[i] = static_cast<hbDNNTensor>(*(inputs[i]));
  }

  // 1. mobile sam encode
  std::vector<hbDNNTensor> encode_inputs;
  encode_inputs.push_back(input_dnn_tensors[0]);
  std::vector<hbDNNTensor> encode_outputs;
  Infer(encode_inputs, encode_outputs, 0);

  // 2 mobile sam decode
  std::vector<hbDNNTensor> decode_inputs;
  std::vector<hbDNNTensor> decode_outputs;
  decode_inputs.push_back(input_dnn_tensors[1]);
  decode_inputs.push_back(encode_outputs[0]);
  Infer(decode_inputs, decode_outputs, 1);
  hbSysFreeMem(&(encode_outputs[0].sysMem[0]));

  std::vector<hbDNNTensor> output_dnn_tensors = decode_outputs;
  int32_t const output_size{static_cast<int32_t>(output_dnn_tensors.size())};
  output->output_tensors.resize(static_cast<size_t>(output_size));     
  for (int32_t i{0}; i < output_size; ++i) {
    output->output_tensors[i] = std::make_shared<DNNTensor>(output_dnn_tensors[i]);
  }

  clock_gettime(CLOCK_REALTIME, &time_now);
  output->rt_stat->infer_timespec_end = time_now;
  output->rt_stat->fps_updated = true;
  auto time_start = ConvertToRosTime(output->rt_stat->infer_timespec_start);
  auto time_end = ConvertToRosTime(output->rt_stat->infer_timespec_end);
  output->rt_stat->infer_time_ms = CalTimeMsDuration(time_start, time_end);

  if (is_sync_mode) {
    threadPool.enqueue([&, output] {
      PostProcess(output);
    });
  } else {
    PostProcess(output);
  }
  return 0;
}


int MobileSamNode::Infer(std::vector<hbDNNTensor>& inputs,
                          std::vector<hbDNNTensor>& outputs,
                          int idx) {
  // 1. 获取dnn_handle
  hbDNNHandle_t dnn_handle = models_[idx]->GetDNNHandle();

  // 2. 准备模型输出数据的空间
  int output_count;
  hbDNNGetOutputCount(&output_count, dnn_handle);
  for (int i = 0; i < output_count; i++) {
    hbDNNTensor output;
    hbDNNTensorProperties &output_properties = output.properties;
    hbDNNGetOutputTensorProperties(&output_properties, dnn_handle, i);
    int out_aligned_size = output_properties.alignedByteSize;
    hbSysMem &mem = output.sysMem[0];
    hbSysAllocCachedMem(&mem, out_aligned_size);
    outputs.push_back(output);
  }

  auto *output_ptr{outputs.data()};
  // 3. 推理模型
  hbDNNTaskHandle_t task_handle = nullptr;
  hbDNNInferCtrlParam infer_ctrl_param;
  HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
  hbDNNInfer(&task_handle,
              &output_ptr,
              inputs.data(),
              dnn_handle,
              &infer_ctrl_param);

  // 4. 等待任务结束
  hbDNNWaitTaskDone(task_handle, 0);

  // 释放任务
  hbDNNReleaseTask(task_handle);

  return 0;
}


int MobileSamNode::PostProcess(
    const std::shared_ptr<DnnNodeOutput>& node_output) {
  if (!rclcpp::ok()) {
    return 0;
  }

  auto parser_output = std::dynamic_pointer_cast<SamOutput>(node_output);
  if (!parser_output) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_mobilesam"), "Invalid node output");
    return -1;
  }

  // 1. 获取后处理开始时间
  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);

  // 2. 模型后处理解析
  auto det_result = std::make_shared<DnnParserResult>();
  output_parser_->Parse(det_result,
                parser_output->resized_h,
                parser_output->resized_w,
                model_input_height_,
                model_input_width_,
                parser_output->output_tensors);
  ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(
      new ai_msgs::msg::PerceptionTargets());
  pub_data->header.set__stamp(parser_output->msg_header->stamp);
  pub_data->header.set__frame_id(parser_output->msg_header->frame_id);
  // 如果开启了渲染，本地渲染并存储图片
  if (dump_render_img_ == 1 && !parser_output->bgr_mat.empty()) {
    std::string saving_path = "render_sam_" + pub_data->header.frame_id + "_" +
                            std::to_string(pub_data->header.stamp.sec) + "_" +
                            std::to_string(pub_data->header.stamp.nanosec) +
                            ".jpeg";
    RenderSeg(parser_output->bgr_mat, det_result->perception.seg, saving_path);
  }
  if (feed_type_ == 0) {
    return 0;
  }

  // 3. 创建用于发布的AI消息
  if (!msg_publisher_) {
    RCLCPP_ERROR(this->get_logger(), "Invalid msg_publisher_");
    return -1;
  }
  
  // 3.1 发布检测AI消息
  det_result->perception.det = parser_output->det;
  RCLCPP_INFO(rclcpp::get_logger("mono_mobilesam"),
              "out box size: %d",
              det_result->perception.det.size());
  for (auto &rect : det_result->perception.det) {
    if (rect.bbox.xmin < 0) rect.bbox.xmin = 0;
    if (rect.bbox.ymin < 0) rect.bbox.ymin = 0;
    if (rect.bbox.xmax >= model_input_width_) {
      rect.bbox.xmax = model_input_width_ - 1;
    }
    if (rect.bbox.ymax >= model_input_height_) {
      rect.bbox.ymax = model_input_height_ - 1;
    }

    std::stringstream ss;
    ss << "det rect: " << rect.bbox.xmin << " " << rect.bbox.ymin << " "
       << rect.bbox.xmax << " " << rect.bbox.ymax
       << ", det type: " << rect.class_name << ", score: " << rect.score;
    RCLCPP_INFO(rclcpp::get_logger("mono_mobilesam"), "%s", ss.str().c_str());

    ai_msgs::msg::Roi roi;
    roi.set__type(rect.class_name);
    roi.rect.set__x_offset(rect.bbox.xmin);
    roi.rect.set__y_offset(rect.bbox.ymin);
    roi.rect.set__width(rect.bbox.xmax - rect.bbox.xmin);
    roi.rect.set__height(rect.bbox.ymax - rect.bbox.ymin);
    roi.set__confidence(rect.score);

    ai_msgs::msg::Target target;
    target.set__type(rect.class_name);
    target.rois.emplace_back(roi);
    pub_data->targets.emplace_back(std::move(target));
  }

  if (parser_output->ratio != 1.0) {

    // 前处理有对图片进行resize，需要将坐标映射到对应的订阅图片分辨率
    for (auto &target : pub_data->targets) {
      for (auto &roi : target.rois) {
        roi.rect.x_offset *= parser_output->ratio;
        roi.rect.y_offset *= parser_output->ratio;
        roi.rect.width *= parser_output->ratio;
        roi.rect.height *= parser_output->ratio;
      }
    }
  }

  // 3.2 发布分割AI消息
  auto &seg = det_result->perception.seg;
  if (seg.height != 0 && seg.width != 0) {
    ai_msgs::msg::Capture capture;
    capture.features.swap(seg.data);
    capture.img.height = seg.valid_h;
    capture.img.width = seg.valid_w;

    capture.img.step = model_input_width_ / seg.width;

    RCLCPP_INFO(rclcpp::get_logger("mono_mobilesam"),
                "features size: %d, width: %d, height: %d, num_classes: %d, step: %d",
                capture.features.size(),
                capture.img.width,
                capture.img.height,
                seg.num_classes,
                capture.img.step);

    ai_msgs::msg::Target target;
    target.set__type("parking_space");
    
    ai_msgs::msg::Attribute attribute;
    attribute.set__type("segmentation_label_count");
    attribute.set__value(seg.num_classes);
    target.attributes.emplace_back(std::move(attribute));

    target.captures.emplace_back(std::move(capture));
    pub_data->targets.emplace_back(std::move(target));
  }

  pub_data->header.set__stamp(parser_output->msg_header->stamp);
  pub_data->header.set__frame_id(parser_output->msg_header->frame_id);

  // 填充perf性能统计信息
  // 前处理统计
  ai_msgs::msg::Perf perf_preprocess = std::move(parser_output->perf_preprocess);
  perf_preprocess.set__time_ms_duration(CalTimeMsDuration(
      perf_preprocess.stamp_start, perf_preprocess.stamp_end));
  pub_data->perfs.push_back(perf_preprocess);

  // predict
  if (parser_output->rt_stat) {
    ai_msgs::msg::Perf perf;
    perf.set__type(model_name_ + "_predict_infer");
    perf.set__stamp_start(
        ConvertToRosTime(parser_output->rt_stat->infer_timespec_start));
    perf.set__stamp_end(
        ConvertToRosTime(parser_output->rt_stat->infer_timespec_end));
    perf.set__time_ms_duration(parser_output->rt_stat->infer_time_ms);
    pub_data->perfs.push_back(perf);
  }

  ai_msgs::msg::Perf perf_postprocess;
  perf_postprocess.set__type(model_name_ + "_postprocess");
  perf_postprocess.set__stamp_start(ConvertToRosTime(time_now));
  clock_gettime(CLOCK_REALTIME, &time_now);
  perf_postprocess.set__stamp_end(ConvertToRosTime(time_now));
  perf_postprocess.set__time_ms_duration(CalTimeMsDuration(
      perf_postprocess.stamp_start, perf_postprocess.stamp_end));
  pub_data->perfs.emplace_back(perf_postprocess);

  // 如果当前帧有更新统计信息，输出统计信息
  if (parser_output->rt_stat->fps_updated) {
      RCLCPP_WARN(rclcpp::get_logger("mono_mobilesam"),
                  "pre process time ms: %d, "
                  "infer time ms: %d, "
                  "post process time ms: %d",
                  static_cast<int>(perf_preprocess.time_ms_duration),
                  parser_output->rt_stat->infer_time_ms,
                  static_cast<int>(perf_postprocess.time_ms_duration));
  }
  msg_publisher_->publish(std::move(pub_data));

  return 0;
}

void MobileSamNode::RosImgProcess(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  if (!img_msg) {
    RCLCPP_DEBUG(rclcpp::get_logger("mono_mobilesam"), "Get img failed");
    return;
  }

  if (!rclcpp::ok()) {
    return;
  }

  std::stringstream ss;
  ss << "Recved img encoding: " << img_msg->encoding
     << ", h: " << img_msg->height << ", w: " << img_msg->width
     << ", step: " << img_msg->step
     << ", frame_id: " << img_msg->header.frame_id
     << ", stamp: " << img_msg->header.stamp.sec << "_"
     << img_msg->header.stamp.nanosec
     << ", data size: " << img_msg->data.size();
  RCLCPP_INFO(rclcpp::get_logger("mono_mobilesam"), "%s", ss.str().c_str());

  auto dnn_output = std::make_shared<SamOutput>();
  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->perf_preprocess.stamp_start.sec = time_now.tv_sec;
  dnn_output->perf_preprocess.stamp_start.nanosec = time_now.tv_nsec;

  // 1. 将图片处理成模型输入数据类型DNNTensor
  hbDNNTensorProperties tensor_properties;
  models_[0]->GetInputTensorProperties(tensor_properties, 0);
  std::shared_ptr<DNNTensor> tensor_image = nullptr;
  cv::Mat bgr_mat;
  if ("rgb8" == img_msg->encoding) {
    auto cv_img =
        cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(img_msg), "bgr8");
    tensor_image = hobot::dnn_node::ImageProc::GetBGRTensorFromBGRImg(
      cv_img->image,
      model_input_height_,
      model_input_width_,
      tensor_properties,
      dnn_output->ratio,
      hobot::dnn_node::ImageType::BGR
    );
    bgr_mat = cv_img->image;
  } else if ("bgr8" == img_msg->encoding) {
    auto cv_img =
        cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(img_msg), "bgr8");
    tensor_image = hobot::dnn_node::ImageProc::GetBGRTensorFromBGRImg(
      cv_img->image,
      model_input_height_,
      model_input_width_,
      tensor_properties,
      dnn_output->ratio,
      hobot::dnn_node::ImageType::RGB);
    bgr_mat = cv_img->image;
  } else if ("nv12" == img_msg->encoding) {  // nv12格式使用hobotcv resize
    hobot::dnn_node::ImageProc::Nv12ToBGR(reinterpret_cast<const char *>(img_msg->data.data()), img_msg->height, img_msg->width, bgr_mat);
    tensor_image = hobot::dnn_node::ImageProc::GetBGRTensorFromBGRImg(
      bgr_mat,
      model_input_height_,
      model_input_width_,
      tensor_properties,
      dnn_output->ratio,
      hobot::dnn_node::ImageType::RGB);
  }

  if (!tensor_image) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_mobilesam"), "Get Tensor fail");
    return;
  }

  // 2. 创建推理输出数据
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id(img_msg->header.frame_id);
  dnn_output->msg_header->set__stamp(img_msg->header.stamp);
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->perf_preprocess.stamp_end.sec = time_now.tv_sec;
  dnn_output->perf_preprocess.stamp_end.nanosec = time_now.tv_nsec;
  dnn_output->resized_h = bgr_mat.rows / dnn_output->ratio;
  dnn_output->resized_w = bgr_mat.cols / dnn_output->ratio;

  if (dump_render_img_) {
    dnn_output->bgr_mat = std::move(bgr_mat);
  }

  // 3. 将准备好的输入输出数据存进缓存
  std::unique_lock<std::mutex> lg(mtx_img_);
  if (cache_img_.size() > cache_len_limit_) {
    CacheImgType img_msg = cache_img_.front();
    cache_img_.pop();
    auto drop_dnn_output = img_msg.first;
    std::string ts =
        std::to_string(drop_dnn_output->msg_header->stamp.sec) + "." +
        std::to_string(drop_dnn_output->msg_header->stamp.nanosec);
    RCLCPP_INFO(rclcpp::get_logger("mono_mobilesam"),
                "drop cache_img_ ts %s",
                ts.c_str());
  }
  CacheImgType cache_img = std::make_pair<std::shared_ptr<SamOutput>,
                                          std::shared_ptr<DNNTensor>>(
      std::move(dnn_output), std::move(tensor_image));
  cache_img_.push(cache_img);
  cv_img_.notify_one();
  lg.unlock();
}


#ifdef SHARED_MEM_ENABLED
void MobileSamNode::SharedMemImgProcess(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg) {
  if (!img_msg || !rclcpp::ok()) {
    return;
  }

  struct timespec time_start = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);

  std::stringstream ss;
  ss << "Recved img encoding: "
     << std::string(reinterpret_cast<const char*>(img_msg->encoding.data()))
     << ", h: " << img_msg->height << ", w: " << img_msg->width
     << ", step: " << img_msg->step << ", index: " << img_msg->index
     << ", stamp: " << img_msg->time_stamp.sec << "_"
     << img_msg->time_stamp.nanosec << ", data size: " << img_msg->data_size;
  RCLCPP_INFO(rclcpp::get_logger("mono_mobilesam"), "%s", ss.str().c_str());

  auto dnn_output = std::make_shared<SamOutput>();
  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->perf_preprocess.stamp_start.sec = time_now.tv_sec;
  dnn_output->perf_preprocess.stamp_start.nanosec = time_now.tv_nsec;

  // 1. 将图片处理成模型输入数据类型DNNTensor
  hbDNNTensorProperties tensor_properties;
  models_[0]->GetInputTensorProperties(tensor_properties, 0);
  std::shared_ptr<DNNTensor> tensor_image = nullptr;
  cv::Mat bgr_mat;
  if ("nv12" ==
      std::string(reinterpret_cast<const char *>(img_msg->encoding.data()))) {
    hobot::dnn_node::ImageProc::Nv12ToBGR(reinterpret_cast<const char *>(img_msg->data.data()), img_msg->height, img_msg->width, bgr_mat);
    tensor_image = hobot::dnn_node::ImageProc::GetBGRTensorFromBGRImg(
          bgr_mat,
          model_input_height_,
          model_input_width_,
          tensor_properties,
          dnn_output->ratio,
          hobot::dnn_node::ImageType::RGB);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("mono_mobilesam"),
                 "Unsupported img encoding: %s, only nv12 img encoding is "
                 "supported for shared mem.",
                 img_msg->encoding.data());
    return;
  }

  if (!tensor_image) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_mobilesam"), "Get Tensor fail");
    return;
  }

  // 2. 创建推理输出数据
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id(std::to_string(img_msg->index));
  dnn_output->msg_header->set__stamp(img_msg->time_stamp);
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->perf_preprocess.stamp_end.sec = time_now.tv_sec;
  dnn_output->perf_preprocess.stamp_end.nanosec = time_now.tv_nsec;
  dnn_output->resized_h = bgr_mat.rows / dnn_output->ratio;
  dnn_output->resized_w = bgr_mat.cols / dnn_output->ratio;

  if (dump_render_img_) {
    dnn_output->bgr_mat = std::move(bgr_mat);
  }

  // 3. 将准备好的输入输出数据存进缓存
  std::unique_lock<std::mutex> lg(mtx_img_);
  if (cache_img_.size() > cache_len_limit_) {
    CacheImgType img_msg = cache_img_.front();
    cache_img_.pop();
    auto drop_dnn_output = img_msg.first;
    std::string ts =
        std::to_string(drop_dnn_output->msg_header->stamp.sec) + "." +
        std::to_string(drop_dnn_output->msg_header->stamp.nanosec);
    RCLCPP_INFO(rclcpp::get_logger("mono_mobilesam"),
                "drop cache_img_ ts %s",
                ts.c_str());
  }
  CacheImgType cache_img = std::make_pair<std::shared_ptr<SamOutput>,
                                          std::shared_ptr<DNNTensor>>(
      std::move(dnn_output), std::move(tensor_image));
  cache_img_.push(cache_img);
  cv_img_.notify_one();
  lg.unlock();
}
#endif

int MobileSamNode::FeedFromLocal() {

  if (access(image_file_.c_str(), R_OK) == -1) {
    RCLCPP_ERROR(
        rclcpp::get_logger("mono_mobilesam"), "Image: %s not exist!", image_file_.c_str());
    return -1;
  }

  auto dnn_output = std::make_shared<SamOutput>();
  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->perf_preprocess.stamp_start.sec = time_now.tv_sec;
  dnn_output->perf_preprocess.stamp_start.nanosec = time_now.tv_nsec;
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id("feedback");


  // 1. 获取图片数据DNNTensor
  hbDNNTensorProperties tensor_properties;
  models_[0]->GetInputTensorProperties(tensor_properties, 0);
  cv::Mat bgr_mat = cv::imread(image_file_, cv::IMREAD_COLOR);
  std::shared_ptr<DNNTensor> tensor_image = hobot::dnn_node::ImageProc::GetBGRTensorFromBGRImg(
          bgr_mat,
          model_input_height_,
          model_input_width_,
          tensor_properties,
          dnn_output->ratio,
          hobot::dnn_node::ImageType::RGB);

  dnn_output->resized_h = bgr_mat.rows / dnn_output->ratio;
  dnn_output->resized_w = bgr_mat.cols / dnn_output->ratio;

  if (dump_render_img_) {
    dnn_output->bgr_mat = std::move(bgr_mat);
  }

  if (!tensor_image) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_mobilesam"),
                 "Get tensor fail with image: %s",
                 image_file_.c_str());
    return -1;
  }

  // 2. 获取 Box Tensor
  models_[1]->GetInputTensorProperties(tensor_properties, 0);
  std::vector<std::vector<float>> boxes =   
        {{69.9, 166.20001, 264.30002, 203.70001}};

  std::shared_ptr<DNNTensor> tensor_box = 
      InputPreProcessor::GetBoxTensor(boxes, tensor_properties);

  // 3. inputs将会作为模型的输入通过RunInferTask接口传入
  std::vector<std::shared_ptr<DNNTensor>> inputs;
  inputs.push_back(tensor_image);
  inputs.push_back(tensor_box);
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->perf_preprocess.stamp_end.sec = time_now.tv_sec;
  dnn_output->perf_preprocess.stamp_end.nanosec = time_now.tv_nsec;

  // 4. 开始预测
  if (RunMulti(inputs, dnn_output, is_sync_mode_ == 1) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_mobilesam"), "Run Model Error");
    return -1;
  }

  return 0;
}


int MobileSamNode::LoadModels() {
  int32_t const models_size{static_cast<int32_t>(model_file_names_.size())};
  packed_dnn_handles_.resize(static_cast<size_t>(models_size));
  std::stringstream ss;
  for (int i = 0; i < models_size; i++) {
    auto model_file = model_file_names_[i];
    const char *file{model_file.c_str()};
    int ret = 0;
    //第一步加载模型
    ret = hbDNNInitializeFromFiles(&packed_dnn_handles_[i], static_cast<const char **>(&file), 1);
    if (ret != 0) {
      return ret;
    }

    // 第二步获取模型名称
    const char **model_names;
    int32_t model_count = 0;
    ret = hbDNNGetModelNameList(&model_names, &model_count, packed_dnn_handles_[i]);
    if (ret != 0) {
      return ret;
    }

    // 第三步获取dnn_handle
    hbDNNHandle_t dnn_handle{nullptr};
    ret = hbDNNGetModelHandle(&dnn_handle, packed_dnn_handles_[i], model_names[0]);
    if (ret != 0) {
      return ret;
    }
    Model* model = new Model(dnn_handle, model_names[0]);
    models_.push_back(model);
    model->PrintModelInfo(ss);
  }

  RCLCPP_INFO(rclcpp::get_logger("mono_mobilesam"), "%s", ss.str().c_str());
  return 0;
}

int MobileSamNode::Debug() {

  std::vector<std::vector<float>> boxes =   
      {{69.9, 166.20001, 264.30002, 203.70001}};

  const float* mask;

  readbinary("/mnt/ai_toolchain/models/sam/orlresult", mask);
  auto parser = std::make_shared<MobileSamOutputParser>(96, 96, 5, 0.1);
  auto det_result = std::make_shared<hobot::dnn_node::output_parser::DnnParserResult>();

  Perception perception;
  float ratio = 10.0 / 3;
  parser->GenMask(mask, 288, 384, 384, 384, perception);

  cv::Mat bgr_mat = cv::imread(image_file_, cv::IMREAD_COLOR);

  std::string saving_path = "/mnt/testdata/render_sam_0.jpeg";
  RenderSeg(bgr_mat, perception.seg, saving_path);

  return 0;
}

void MobileSamNode::AiMsgProcess(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {
  if (!msg || !rclcpp::ok() || !ai_msg_manage_) {
    return;
  }

  std::stringstream ss;
  ss << "Recved ai msg"
     << ", frame_id: " << msg->header.frame_id
     << ", stamp: " << msg->header.stamp.sec << "_"
     << msg->header.stamp.nanosec;
  RCLCPP_INFO(
      rclcpp::get_logger("mono_mobilesam"), "%s", ss.str().c_str());

  ai_msg_manage_->Feed(msg);
}

void MobileSamNode::RunPredict() {
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lg(mtx_img_);
    cv_img_.wait(lg, [this]() { return !cache_img_.empty() || !rclcpp::ok(); });
    if (cache_img_.empty()) {
      continue;
    }
    if (!rclcpp::ok()) {
      break;
    }
    CacheImgType img_msg = cache_img_.front();
    cache_img_.pop();
    lg.unlock();

    // 1. 获取 Image Tensor
    auto dnn_output = img_msg.first;
    auto image_tensor = img_msg.second;

    std::string ts =
        std::to_string(dnn_output->msg_header->stamp.sec) + "." +
        std::to_string(dnn_output->msg_header->stamp.nanosec);

    // 2. 获取 Box Tensor
    std::vector<std::vector<float>> boxes;
    std::vector<std::string> class_names;
    std::vector<float> confidences;
    if (is_regular_box_ == 1) {
      boxes.push_back(regular_box_);
      class_names.push_back("anything");
      confidences.push_back(1.0);
    } else {
      std::shared_ptr<std::vector<hbDNNRoi>> rois = nullptr;
      std::map<size_t, size_t> valid_roi_idx;
      ai_msgs::msg::PerceptionTargets::UniquePtr ai_msg = nullptr;
      if (ai_msg_manage_->GetTargetRois(dnn_output->msg_header->stamp,
                                        rois,
                                        class_names,
                                        confidences,
                                        valid_roi_idx,
                                        ai_msg,
                                        500) < 0 ||
          !ai_msg) {
        RCLCPP_INFO(rclcpp::get_logger("mono_mobilesam"),
                    "Frame ts %s get hand roi fail",
                    ts.c_str());
        continue;
      }
      if (!rois || rois->empty() || rois->size() != valid_roi_idx.size()) {
        RCLCPP_INFO(rclcpp::get_logger("mono_mobilesam"),
                    "Frame ts %s has no hand roi",
                    ts.c_str());
        ai_msgs::msg::PerceptionTargets::UniquePtr msg(
                      new ai_msgs::msg::PerceptionTargets());
        msg->header.set__stamp(dnn_output->msg_header->stamp);
        msg->header.set__frame_id(dnn_output->msg_header->frame_id);
        msg_publisher_->publish(std::move(msg));
        continue;
      }
      if (GenScaleBox(rois, boxes, dnn_output->ratio)) {
        RCLCPP_ERROR(rclcpp::get_logger("mono_mobilesam"),
              "get scale box failed.");
      }
    }

    for (int i = 0; i < boxes.size(); i++) {
      auto box = boxes[i];
      hobot::dnn_node::output_parser::Bbox bbox = {box[0], box[1], box[2], box[3]};
      hobot::dnn_node::output_parser::Detection det;
      det.bbox = bbox;
      det.class_name = class_names[i].c_str();
      det.score = confidences[i];
      dnn_output->det.push_back(det);
    }

    hbDNNTensorProperties tensor_properties;
    models_[1]->GetInputTensorProperties(tensor_properties, 0);
    std::shared_ptr<DNNTensor> tensor_box = 
        InputPreProcessor::GetBoxTensor(boxes, tensor_properties);

    // 3. 使用DNNTensor创建DNNInput对象inputs
    // inputs将会作为模型的输入通过RunInferTask接口传入
    std::vector<std::shared_ptr<DNNTensor>> inputs;
    inputs.push_back(image_tensor);
    inputs.push_back(tensor_box);

    uint32_t ret = 0;
    // 4. 开始预测
    ret = RunMulti(inputs, dnn_output, is_sync_mode_ == 1);

    // 5. 处理预测结果，如渲染到图片或者发布预测结果
    if (ret != 0) {
      continue;
    }
  }
}
