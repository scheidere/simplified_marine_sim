/* Copyright (C) 2025 Emily Scheide - All Rights Reserved
* Portions Copyright (C) 2015-2018 Michele Colledanchise - All Rights Reserved
* Portions Copyright (C) 2018-2020 Davide Faconti, Eurecat - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

// #pragma once

// #include "behaviortree_cpp/control_node.h"

// namespace BT
// {
/**
 * @brief The RepeatSequenceNode is used to tick children left to right recursively in order
 *      - If input information pertaining to convergence is given, this repeating will stop once the convergence threshold is met
 *          - If all the children return SUCCESS and the threshold the user provides is met, then this node returns SUCCESS.
 *          - If a child returns RUNNING, this node returns RUNNING.
 *            Loop is NOT restarted, the same running child will be ticked again.
 *            (Note this is the same as the typical sequence node implementation.)
 *          - If a child returns FAILURE, stop the loop and return FAILURE.
 * 
 *      - If no input information is given, the repeating will continue until external termination of the behavior tree
 *          - Always returns RUNNING regardless of child node status(es)
 *
 * Additional notes:
 *
 * - This node (with input information) is designed for the implementation of the Consensus-Based Bundle Algorithm in a behavior tree.
 *   It could be used for other things, but the input threshold names refer to the number of rounds of CBBA 
 *   that have not resulted in any change in local task allocation beliefs.
 * 
 * - This node (without input) is designed to repeat without limits. 
 * 
 */

#pragma once

#include "behaviortree_cpp/control_node.h"

namespace BT
{

class RepeatSequence : public ControlNode
{
public:
  RepeatSequence(const std::string& name, const BT::NodeConfig& config);

  static PortsList providedPorts()
  {
    return {
      InputPort<int>("convergence_threshold", // no default value
                     "Number of consecutive iterations of CBBA in which local convergence must be maintained"),
      InputPort<int>("cumulative_convergence_count_in"),
      OutputPort<int>("cumulative_convergence_count_out") // Need to fix
      //OutputPort<bool>("threshold_met", "Signal when threshold is reached") // Can't just set convergence count as output due to conflict with input port
    };

  }

  virtual ~RepeatSequence() override = default;
  virtual void halt() override;

private:
  virtual NodeStatus tick() override;

  size_t current_child_idx_ = 0;
  int convergence_count_ = 0;
  int convergence_threshold_ = 1;
  int cumulative_convergence_count_ = 0;
  bool has_threshold_ = false;


};

}  // namespace BT