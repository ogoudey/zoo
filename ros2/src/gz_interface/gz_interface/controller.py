import json
import random

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist



from ros_gz_interfaces.srv import ControlWorld


import threading

# .sdf to start is in /home/olin/../../usr/share/gz/gz-sim8/worlds/shapes.sdf
# now there by `sudo cp resources/vehicle/model.sdf ~/../../opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/`
# now just add a ground plane (take from empty.sdf)




class Policy:
    def __init__(self, actions):
        self.actions = actions
        self.action_selections = dict()
        self.id = "random"
        
    def action(self, state):
        if not state in self.action_selections.keys():
            return random.choice(list(self.actions))
        else:
            return self.action_selections[state]
    
    def max(self, state_dict):
        best_average_return = -999
        best_action = []
        for action in state_dict.keys():
            if state_dict[action] > best_average_return:
                best_action = [action]
                best_average_return = state_dict[action]
            elif state_dict[action] == best_average_return:
                best_action.append(action)
        if len(best_action) == 0:
            return random.choice(list(self.actions))
        return random.choice(best_action) # should be a choice of 1 in most later cases
    
    def from_table_greedy(self, table, _id=None):
        if _id:
            self.id = _id
        
        for state in table.keys():
            self.action_selections[state] = self.max(table[state])

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        
        self.controller = self.create_publisher(Twist, '/model/vehicle/cmd_vel', 10)
        self.odometer = self.create_subscription(Odometry, '/model/vehicle/odometry', self.act, 10)
        
        self.rollout_cnt = 0
        self.num_rollouts = 100
        
        self.step = 0
        self.len_episode = 10
        
        actions = {(1.0,0.0), (-1.0,0.0), (0.0,1.0), (0.0,-1.0), (1.0,1.0), (-1.0,1.0), (1.0,-1.0), (-1.0,-1.0), (0.0,0.0)}
        self.velocity = [0.0, 0.0]
        self.policy = Policy(actions)
        

        self.reward_pairs = [((3, 1), x) for x in actions] # if it does anything in this final state
        self.reward_pairs += [((4, 1), x) for x in actions] # if it does anything in this final state
        self.reward_pairs += [((3, 2), x) for x in actions] # if it does anything in this final state
        self.reward_pairs += [((4, 2), x) for x in actions] # if it does anything in this final state
        self.get_logger().info(str(self.reward_pairs))
    
        self.rewarded = False

        self.prev_state = None
        self.prev_action = None
        
        self.average_returns = dict()
        self.action_pair_cnts = dict()

        self.cli = self.create_client(ControlWorld, '/world/model/control')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('service not available, waiting again...')
            
        self.req = ControlWorld.Request()
        
        self.episode = {"states":[], "actions":[], "rewards":[]}
        
        self.event = threading.Event()
        self.resetter_thread = threading.Thread(target=self.reset_t, args=[])
        self.resetter_thread.start()
        self.get_logger().info('GZ reset thread started.')
        self.event.set() # reset upon gz_interface execution

    def reset_t(self):
        self.get_logger().info("Thread started.")
        while rclpy.ok():

            self.event.wait()
            result = self.new_episode()
            
                
            self.step = 0 
            self.prev_state = None
            self.prev_action = None  
            self.velocity = [0.0, 0.0]  
            self.event.clear()
            self.get_logger().info('Reset vehicle position. Episode # ' + str(self.rollout_cnt))
            self.get_logger().info('On policy ' + str(self.policy.id))
        self.get_logger().info("Shutting down")
        rclpy.shutdown()
        

    def new_episode(self):
        if self.rollout_cnt < self.num_rollouts:
            self.req.world_control.reset.all = True
            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
            return self.future.result().success
        else:
            #self.get_logger().info(str(self.average_returns))
            
            
            self.rollout_cnt = -999 # just assume that we do inf rollouts once the new policy is formed.
            self.get_logger().info("\n\t\tVVVVVVVVV\n")
            self.get_logger().info(str(self.policy.action_selections))
                
            #with open("data/average_returns.txt", "w") as json_file:
            #    json.dump({"average_returns": self.average_returns, "policy": self.policy.action_selections}, json_file, indent=4,)
            return True
       
        
        


            
                
    def act(self, odometry):
        self.step += 1
        resolution = 4
        state = int(odometry.pose.pose.position.x * resolution), int(odometry.pose.pose.position.y * resolution)
        self.get_logger().info("State: " + str(state) + "\tVelocity: " + str(self.velocity))
        self.episode["states"].append(state)
        if not state in self.average_returns.keys():
                self.average_returns[state] = {key: 0 for key in self.policy.actions}
                self.action_pair_cnts[state] = {key: 0 for key in self.policy.actions}
        if self.prev_state:
            if (self.prev_state, self.prev_action) in self.reward_pairs: # this seems to fall a little late, but its probably ok
                reward = 100
                self.get_logger().info("Rewarded")
                self.rewarded = True
                self.step = self.len_episode # shows over
            else:
                reward = 0
            self.episode["rewards"].append(reward)
            
        if self.step > self.len_episode:
            self.get_logger().info("Episode done.")
            gamma = 0.9
            for i in range(0, len(self.episode["rewards"])):
                _return = 0
                j = 0
                while (j + i) < len(self.episode["rewards"]):
                    _return += (gamma**j) * self.episode["rewards"][j + i]
                    j += 1
                if self.episode["actions"][i] in self.average_returns[self.episode["states"][i]].keys():
                    self.average_returns[self.episode["states"][i]][self.episode["actions"][i]] += _return
                    self.action_pair_cnts[self.episode["states"][i]][self.episode["actions"][i]] += 1
                else:
                    self.average_returns[self.episode["states"][i]][self.episode["actions"][i]] = _return
                    self.action_pair_cnts[self.episode["states"][i]][self.episode["actions"][i]] = 1
                    
            q_table = dict()
            
            for state in self.average_returns.keys():
                q_table[state] = dict()
                for action in self.average_returns[state].keys():
                    if self.action_pair_cnts[state][action] > 0: 
                        q_table[state][action] = self.average_returns[state][action] / self.action_pair_cnts[state][action]
                    else:
                        q_table[state][action] = 0
            _id = "learned-v"+ str(self.rollout_cnt)
            self.policy.from_table_greedy(q_table, _id)
            # Respawn. Restart episode.
            
            
            self.rollout_cnt += 1  
            self.event.set()   
        #endif
        
        if not self.rewarded:
            self.get_logger().info("Still not rewarded")
        
        action = self.policy.action(state)
        self.prev_state = state
        self.publish(action)
        

               
    def publish(self, action):
        #msg = {"linear": {"x": 5.0, "y": 0.0, "z": 0.0}, "angular": {"x": 1.0, "y": 1.0, "z": 0.0}}
        linear_scale = 0.5
        angular_scale = 0.5
        self.velocity[0] += action[0] * linear_scale
        self.velocity[1] += action[1] * angular_scale
        twist = Twist()
        twist.linear.x = self.velocity[0]
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.velocity[1]
        
        self.get_logger().info("Publishing Twist: " + str(self.velocity))
        self.controller.publish(twist)
        self.episode["actions"].append(action)
        self.prev_action = action



def main(args=None):
    rclpy.init(args=args)

    gz = Controller()

    rclpy.spin(gz)

    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
