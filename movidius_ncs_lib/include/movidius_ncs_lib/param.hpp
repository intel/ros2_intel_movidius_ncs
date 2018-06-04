// Copyright (c) 2017 Intel Corporation. All Rights Reserved
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

#ifndef MOVIDIUS_NCS_LIB__PARAM_HPP_
#define MOVIDIUS_NCS_LIB__PARAM_HPP_

#include <memory>
#include <string>

namespace movidius_ncs_lib
{
class Param
{
public:
  using Ptr = std::shared_ptr<Param>;
  using ConstPtr = std::shared_ptr<Param const>;

  Param();

  bool loadParamFromYAML(const std::string & file_path);
  bool validateParam();

  int device_index_;
  int log_level_;
  std::string cnn_type_;
  std::string graph_file_path_;
  std::string category_file_path_;
  int network_dimension_;
  float channel1_mean_;
  float channel2_mean_;
  float channel3_mean_;
  float scale_;
  int top_n_;
};
}  // namespace movidius_ncs_lib

#endif  // MOVIDIUS_NCS_LIB__PARAM_HPP_
