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

#include <memory>
#include <string>
#include <vector>

#include "ai_msgs/msg/perception_targets.hpp"
#include "cv_bridge/cv_bridge.h"
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "dnn_node/util/output_parser/perception_common.h"

#ifdef SHARED_MEM_ENABLED
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#endif
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "include/ai_msg_manage.h"
#include "include/data_preprocess.h"
#include "include/mobilesam_output_parser.h"

#ifndef MOBILESAM_NODE_H_
#define MOBILESAM_NODE_H_

using rclcpp::NodeOptions;

using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::DnnNodePara;

using hobot::dnn_node::DNNTensor;
using hobot::dnn_node::ModelTaskType;
using hobot::dnn_node::Model;

using ai_msgs::msg::PerceptionTargets;

class ThreadPool {
public:
    ThreadPool() : stop(false) {
        worker = std::thread(&ThreadPool::workerThread, this);
    }

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            stop = true;
        }
        condition.notify_all();
        if (worker.joinable()) {
            worker.join();
        }
    }

    void enqueue(std::function<void()> task) {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            tasks.push(task);
        }
        condition.notify_one();
    }

private:
    void workerThread() {
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(queue_mutex);
                condition.wait(lock, [this] { return stop || !tasks.empty(); });
                if (stop && tasks.empty())
                    return;
                if (!tasks.empty()) {
                    task = std::move(tasks.front());
                    tasks.pop();
                }
            }
            if (task) {
                task();
            }
        }
    }

    std::thread worker;
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;
};

struct SamOutput : public DnnNodeOutput {
  // resize参数，用于算法检测结果的映射
  float ratio = 1.0;  //缩放比例系数，无需缩放为1

  ai_msgs::msg::Perf perf_preprocess;

  int resized_w = 0; // 经过resize后图像的w
  int resized_h = 0; // 经过resize后图像的w

  std::shared_ptr<std::vector<hbDNNRoi>> rois = nullptr;

  std::vector<hobot::dnn_node::output_parser::Detection> det;

  cv::Mat bgr_mat;
};

class MobileSamNode : public DnnNode {
 public:
  MobileSamNode(const std::string &node_name,
                 const NodeOptions &options = NodeOptions());
  ~MobileSamNode() override;

 protected:
  int SetNodePara() override;

  int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs) override;

 private:

  int FeedFromLocal();

  int LoadModels();
  
  int Infer(std::vector<hbDNNTensor>& inputs,
            std::vector<hbDNNTensor>& outputs,
            int idx);

  int RunMulti(std::vector<std::shared_ptr<DNNTensor>>& inputs,
        const std::shared_ptr<DnnNodeOutput> &output = nullptr,
        const bool is_sync_mode = false);


#ifdef SHARED_MEM_ENABLED
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr
      sharedmem_img_subscription_ = nullptr;
  std::string sharedmem_img_topic_name_ = "/hbmem_img";
  void SharedMemImgProcess(
      const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
#endif

  std::string ai_msg_pub_topic_name_ = "/hobot_sam";
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr msg_publisher_ =
      nullptr;

  std::string ai_msg_sub_topic_name_ = "/hobot_detection";
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr
      ai_msg_subscription_ = nullptr;
  void AiMsgProcess(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);

  // 目前只支持订阅深度图原图
  std::string ros_img_sub_topic_name_ = "/image";
  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
      ros_img_subscription_ = nullptr;
  void RosImgProcess(const sensor_msgs::msg::Image::ConstSharedPtr msg);


  int dump_render_img_ = 0;
  // 用于预测的图片来源, 0：本地彩色图, 1： 订阅到的image msg
  int feed_type_ = 0;

  std::string image_file_ = "config/00131.jpg";
  // 加载模型后，查询出模型输入分辨率
  int model_input_width_ = 384;
  int model_input_height_ = 384;

  // 使用shared mem通信方式订阅图片
  int is_shared_mem_sub_ = 1;
  int is_sync_mode_ = 0;
  // 使用固定框进行sam分割
  int is_regular_box_ = 0;
  std::vector<float> regular_box_ = {96.0, 96.0, 288.0, 192};   // 对应 384 * 384 模型输入
  std::string model_name_ = "sam";

  // 多模型推理相关
  std::vector<std::string> model_file_names_ = {
                        "config/mobilesam_encoder_384.bin",
                        "config/mobilesam_decoder_384.bin"};
  std::vector<Model *> models_;
  std::vector<hbPackedDNNHandle_t> packed_dnn_handles_;
  ThreadPool threadPool;

  std::shared_ptr<AiMsgManage> ai_msg_manage_ = nullptr;

  std::shared_ptr<MobileSamOutputParser> output_parser_ = nullptr;

  // 将订阅到的图片数据转成pym之后缓存
  // 在线程中执行推理，避免阻塞订阅IO通道，导致AI msg消息丢失
  int cache_len_limit_ = 8;
  std::mutex mtx_img_;
  std::condition_variable cv_img_;
  using CacheImgType = std::pair<std::shared_ptr<SamOutput>,
                                 std::shared_ptr<DNNTensor>>;
  std::queue<CacheImgType> cache_img_;
  void RunPredict();
  std::shared_ptr<std::thread> predict_task_ = nullptr;
};

#endif  // MOBILESAM_NODE_H_
