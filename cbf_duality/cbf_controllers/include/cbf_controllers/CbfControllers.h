//
// Created by qiayuan on 2022/7/16.
//

#pragma once

#include <legged_controllers/LeggedController.h>

namespace cbf {
using namespace ocs2;
using namespace legged_robot;

class DualityController : public legged::LeggedController {
 protected:
  void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                            bool verbose) override;
  void setupMpc() override;
};

class DCbfDualityController : public legged::LeggedController {
 protected:
  void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                            bool verbose) override;
  void setupMpc() override;
};

}  // namespace cbf
