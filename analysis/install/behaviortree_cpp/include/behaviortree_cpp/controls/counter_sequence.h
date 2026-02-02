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
 * @brief The CounterSequenceNode is used to tick one or all children from left to right per tracked failure information
 * 		- This node is designed to function for non-failing task execution and fault tolerance execution for both the helper and helpee robot
 * 		- By default, with no failures, operates as a normal sequence identically (left to right sequential execution)
 * 		- We denote a robot that fails a subtask to be the helpee robot
 * 			- In this case, the counter sequence remains running while the failing child node remains failed (until help arrives)
 * 				- This is typical sequence node behavior still
 * 		- We denote a robot that has been assigned a subtask explicitly (through CBGA) as the helper robot
 * 			- In this case, the counter sequence only runs the subtask the helper has been requested to assist with
 * 		- This is done through the tracking and sharing of information between the counter sequence and the HandleFailures node
 *
 * 		
 * Additional notes:
 * 		- Expected adjacent behavior tree structure:
 * 			- We consider a subtree responsible for executing some main task, that is comprised of subtasks (each represented by an action node)
 * 			- Under a root RepeatSequence
 * 				- First is the trigger condition checking that the current task in robot's CBGA path pertains to this subtree
 * 				- Second is the counter sequence itself
 * 			- Beneath the counter sequence:
 * 				- The first (leftmost) child node is expected to be an action node 'HandleFailures' that sends robot-specific info
 * 				- All child nodes to the right of HandleFailures are subtask action nodes that when done in sequence complete the main task this subtree represents
 * 	
 * 		- Requires three forms of input (passed from HandleFailures via ports):
 * 			- task_is_main: Bool denoting whether execution should be a single child node (denoted by associated task id)
 * 			- current_task_id: Int denoting current task id (whether task is main or subtask)
 * 			- subtask_failure_thresholds: std::unordered_map<int,int> that represents task id and task failure threshold
 * 
 * 		- Output:
 * 			- Subtask failure tracker std::unordered_map<int,bool> that has task id and bool denoting failure (1) or 0 otherwise 
 * 			- Counts fails (by return status of FAILURE) of each child node other than HandleFailures (which are all subtask action nodes) 
 * 			- When threshold number is reached for given subtask node, the subtask failure tracker is updated (0 to 1)
 * 			- Tracker is sent to HandleFailures every tick between them
 * 
 * 		- During exection:
 * 			- If task_is_main is true, counter sequence runs all children in sequence (with HandleFailures leftmost so first always)
 * 			- If task_is_main is false, then only the subtask action node that matches the current_task_id runs
 * 			- In either case, failure count nums are continuously updated for each subtask based on return statuses (so only one subtask if helper running only one subtask)
 * 				- If RUNNING, no change to subtask_failure_counts
 * 				- If FAILURE, +1 to count for given subtask id
 * 				- If SUCCESS, reset count to 0 for given subtask id
 * 					- Also update subtask failure tracker for same id to have 0 to denote task is successful (0 to 0 (never failed), or 1 (failing) to 0 (now successful given help))
 * 						- This will be most relevant for helpee running main task that has just been helped to complete failing subtask 
 *
 * InputPort<std::unordered_map<int,bool>>("self_subtask_failures"),
             OutputPort<bool>("task_is_main"),
             OutputPort<int>("current_task_id"),
             OutputPort<std::unordered_map<int,int>>("subtask_failure_thresholds")};
}
 */


#pragma once

#include "behaviortree_cpp/control_node.h"

namespace BT
{

class CounterSequence : public ControlNode
{
public:
  CounterSequence(const std::string& name, const BT::NodeConfig& config);

  static PortsList providedPorts()
  {
    return {
    	// Inputs sent from HandleFailures (first child node)
    	InputPort<bool>("task_is_main"),
        InputPort<int>("current_task_id"),
        InputPort<std::map<int,int>>("subtask_failure_thresholds"),
        OutputPort<std::unordered_map<int,bool>>("self_subtask_failures")
   	};

  }

  virtual ~CounterSequence() override = default;
  virtual void halt() override;
  virtual int getSubtaskID(int current_child_idx);
  virtual int getChildNodeID(int current_task_id);

private:
  virtual NodeStatus tick() override;

  size_t current_child_idx_ = 0;
  bool task_is_main_; // Input, robot info via HandleFailures
  int current_task_id_; // Input, robot info via HandleFailures
  std::map<int,int> subtask_failure_thresholds_; // Input, robot info via HandleFailures
  std::unordered_map<int,int> failure_counts_; // local, used against thresholds to update failure bools
  std::unordered_map<int,bool> self_subtask_failures_; // updated locally, output to HandleFailures
  


};

}  // namespace BT