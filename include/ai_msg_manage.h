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

#include <map>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node_data.h"

#ifndef AI_MSG_MANAGE_H_
#define AI_MSG_MANAGE_H_

using ai_msgs::msg::PerceptionTargets;

using feed_predict_type =
    std::shared_ptr<std::pair<ai_msgs::msg::PerceptionTargets::ConstSharedPtr,
                              sensor_msgs::msg::Image::ConstSharedPtr>>;

struct compare_msg {
  bool operator()(const std_msgs::msg::Header::_stamp_type m1,
                  const std_msgs::msg::Header::_stamp_type m2) {
    return ((m1.sec > m2.sec) ||
            ((m1.sec == m2.sec) && (m1.nanosec > m2.nanosec)));
  }
};

class AiMsgFeedCache {
 public:
  explicit AiMsgFeedCache(int cache_lint_len = 20)
      : cache_limt_len_(cache_lint_len) {}
  ~AiMsgFeedCache() {}

  int Feed(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr& msg) {
    ai_msgs::msg::PerceptionTargets::UniquePtr ai_msg(
        new ai_msgs::msg::PerceptionTargets());
    ai_msg->set__header(msg->header);
    ai_msg->set__fps(msg->fps);
    ai_msg->set__targets(msg->targets);
    ai_msg->set__disappeared_targets(msg->disappeared_targets);
    ai_msg->set__perfs(msg->perfs);
    std::string ts = std::to_string(msg->header.stamp.sec) + "." +
                     std::to_string(msg->header.stamp.nanosec);
    RCLCPP_DEBUG(
        rclcpp::get_logger("sam_msg_manage"), "Feed ts %s", ts.c_str());

    std::unique_lock<std::mutex> lg(cache_mtx_);
    // 判断是否超过最大长度
    if (recved_aimsg_cache_.size() > cache_limt_len_) {
      // 删掉最小时间戳
      std::string top_ts = std::to_string(recved_aimsg_ts_.top().sec) + "." +
                           std::to_string(recved_aimsg_ts_.top().nanosec);
      RCLCPP_WARN(rclcpp::get_logger("sam_msg_manage"),
                  "Ai msg cache len: %d exceeds limit: %d, erase ai ts: %s",
                  recved_aimsg_cache_.size(),
                  cache_limt_len_,
                  top_ts.data());
      recved_aimsg_cache_.erase(top_ts);
      recved_aimsg_ts_.pop();
    }

    if (recved_aimsg_ts_.size() > cache_limt_len_) {
      std::string top_ts = std::to_string(recved_aimsg_ts_.top().sec) + "." +
                           std::to_string(recved_aimsg_ts_.top().nanosec);
      RCLCPP_WARN(rclcpp::get_logger("sam_msg_manage"),
                  "Ts cache len: %d exceeds limit: %d, erase ts: %s",
                  recved_aimsg_ts_.size(),
                  cache_limt_len_,
                  top_ts.data());
      recved_aimsg_cache_.erase(top_ts);
      recved_aimsg_ts_.pop();
    }

    recved_aimsg_cache_[ts] = std::move(ai_msg);
    recved_aimsg_ts_.push(msg->header.stamp);
    RCLCPP_DEBUG(rclcpp::get_logger("sam_msg_manage"),
                 "top ts: %llu %llu, cache size: %d, ts size: %d",
                 recved_aimsg_ts_.top().sec,
                 recved_aimsg_ts_.top().nanosec,
                 recved_aimsg_cache_.size(),
                 recved_aimsg_ts_.size());
    cache_cv_.notify_one();
    return 0;
  }

  ai_msgs::msg::PerceptionTargets::UniquePtr Get(
      const std_msgs::msg::Header::_stamp_type& msg_ts,
      int time_out_ms = 1000) {
    std::string ts =
        std::to_string(msg_ts.sec) + "." + std::to_string(msg_ts.nanosec);
    RCLCPP_DEBUG(
        rclcpp::get_logger("sam_msg_manage"), "Get ts %s", ts.c_str());

    ai_msgs::msg::PerceptionTargets::UniquePtr feed_predict = nullptr;
    std::unique_lock<std::mutex> lg(cache_mtx_);
    cache_cv_.wait_for(lg, std::chrono::milliseconds(time_out_ms), [&]() {
      return recved_aimsg_cache_.find(ts) != recved_aimsg_cache_.end();
    });
    if (recved_aimsg_cache_.find(ts) != recved_aimsg_cache_.end()) {
      feed_predict = std::move(recved_aimsg_cache_.at(ts));
      recved_aimsg_cache_.erase(ts);

      RCLCPP_DEBUG(rclcpp::get_logger("sam_msg_manage"),
                   "find ts %s success, recved_aimsg_ts_ top ts: %llu %llu, "
                   "cache size: %d, ts size: %d",
                   ts.c_str(),
                   recved_aimsg_ts_.top().sec,
                   recved_aimsg_ts_.top().nanosec,
                   recved_aimsg_cache_.size(),
                   recved_aimsg_ts_.size());

      // 清理过期的时间戳缓存
      while (!recved_aimsg_ts_.empty() &&
             recved_aimsg_ts_.size() > recved_aimsg_cache_.size()) {
        if ((recved_aimsg_ts_.top() == msg_ts) ||
            (recved_aimsg_ts_.top().sec < msg_ts.sec) ||
            (recved_aimsg_ts_.top().sec == msg_ts.sec &&
             recved_aimsg_ts_.top().nanosec < msg_ts.nanosec)) {
          std::string top_ts = std::to_string(recved_aimsg_ts_.top().sec) +
                               "." +
                               std::to_string(recved_aimsg_ts_.top().nanosec);
          recved_aimsg_cache_.erase(top_ts);
          recved_aimsg_ts_.pop();

          RCLCPP_DEBUG(rclcpp::get_logger("sam_msg_manage"),
                       "Erase top ts: %s, cache len: %d ts len: %d",
                       top_ts.data(),
                       recved_aimsg_cache_.size(),
                       recved_aimsg_ts_.size());
        } else {
          break;
        }
      }
    } else {
      if (!recved_aimsg_ts_.empty() && recved_aimsg_ts_.top().sec == 0) {
        std::string ts = std::to_string(0) + "." + std::to_string(0);
        feed_predict = std::move(recved_aimsg_cache_.at(ts));
        recved_aimsg_ts_.pop();
      }
    }

    return feed_predict;
  }

 private:
  // 缓存的长度限制
  size_t cache_limt_len_ = 20;
  std::mutex cache_mtx_;
  std::condition_variable cache_cv_;
  // 缓存AI结果的时间戳
  std::priority_queue<std_msgs::msg::Header::_stamp_type,
                      std::vector<std_msgs::msg::Header::_stamp_type>,
                      compare_msg>
      recved_aimsg_ts_;
  // key is ts
  std::unordered_map<std::string, ai_msgs::msg::PerceptionTargets::UniquePtr>
      recved_aimsg_cache_;
};

// 用于订阅到的AI msg管理
// 当用户订阅到AI msg时，调用Feed接口根据时间戳缓存每帧AI msg
// 当用户订阅到图像数据时，调用GetTargetRois接口获取对应时间戳帧中的roi，同时返回每个roi的有效信息
class AiMsgManage {
 public:
  AiMsgManage();
  ~AiMsgManage();

  void Feed(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);
  int GetTargetRois(const std_msgs::msg::Header::_stamp_type& msg_ts,
                    std::shared_ptr<std::vector<hbDNNRoi>>& rois,
                    std::vector<std::string>& class_names,
                    std::vector<float>& confidences,
                    std::map<size_t, size_t>& valid_roi_idx,
                    ai_msgs::msg::PerceptionTargets::UniquePtr& ai_msg,
                    int time_out_ms = 200);

 private:
  AiMsgFeedCache ai_msg_feed_cache_;
  // resizer model input size limit
  // roi, width & hight must be in range [16, 256)
  int32_t roi_size_max_ = 255;
  int32_t roi_size_min_ = 16;
};

#endif  // AI_MSG_MANAGE_H_
