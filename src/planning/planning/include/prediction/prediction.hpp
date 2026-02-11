#pragma once

#include <mapping/mapping.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <queue>

namespace prediction {

struct Node {
  Eigen::Vector3d p, v, a; // position, vel, accel (?)
  double t;
  double score;
  double h;
  Node* parent = nullptr;
};
typedef Node* NodePtr;
class NodeComparator {
 public:
  bool operator()(NodePtr& lhs, NodePtr& rhs) {
    return lhs->score + lhs->h > rhs->score + rhs->h;
  }
};
struct Predict {
 private:
  static constexpr int MAX_MEMORY = 1 << 22;
  // searching

  double dt;
  double pre_dur;
  double rho_a;
  double car_z, vmax;
  mapping::OccGridMap map;
  NodePtr data[MAX_MEMORY]; // Ptr to a Node struct
  int stack_top;

  // invalid if velocity exceeds specified max velocity or if point is in occupied space
  inline bool isValid(const Eigen::Vector3d& p, const Eigen::Vector3d& v) const {
    return (v.norm() < vmax) && (!map.isOccupied(p));
  }

 public:
  inline Predict(ros::NodeHandle& nh) {
    nh.getParam("tracking_dur", pre_dur);
    nh.getParam("tracking_dt", dt);
    nh.getParam("prediction/rho_a", rho_a);
    nh.getParam("prediction/vmax", vmax);
    for (int i = 0; i < MAX_MEMORY; ++i) {
      data[i] = new Node; // instantiate a new Node struct for each position in data
    }
  }
  inline void setMap(const mapping::OccGridMap& _map) {
    map = _map;
    // map.inflate_last();
  }

  inline bool predict(const Eigen::Vector3d& target_p, // target current position
                      const Eigen::Vector3d& target_v, // target current velocity
                      std::vector<Eigen::Vector3d>& target_predcit, // vector of predicted positions. to be populated by predict()
                      const double& max_time = 0.1) {
    auto score = [&](const NodePtr& ptr) -> double {
      return rho_a * ptr->a.norm();
    };
    Eigen::Vector3d end_p = target_p + target_v * pre_dur; // estimated target end position
    auto calH = [&](const NodePtr& ptr) -> double { // some lambda function
      return 0.001 * (ptr->p - end_p).norm();
    };
    ros::Time t_start = ros::Time::now();
    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set; // what is this?

    Eigen::Vector3d input(0, 0, 0); // probably acceleration input

    stack_top = 0;
    NodePtr curPtr = data[stack_top++];
    curPtr->p = target_p;
    curPtr->v = target_v;
    curPtr->a.setZero();
    curPtr->parent = nullptr;
    curPtr->score = 0;
    curPtr->h = 0;
    curPtr->t = 0;
    double dt2_2 = dt * dt / 2; // why divide by 2? -> to average it?
    while (curPtr->t < pre_dur) { // populate data LL for the entire prediction duration
      for (input.x() = -3; input.x() <= 3; input.x() += 3) {
        for (input.y() = -3; input.y() <= 3; input.y() += 3) {
          Eigen::Vector3d p = curPtr->p + curPtr->v * dt + input * dt2_2; // compute the next position based on previous position and input
          Eigen::Vector3d v = curPtr->v + input * dt; // compute the next velocity based on previous velocity and input
          if (!isValid(p, v)) {
            continue;
          }
          if (stack_top == MAX_MEMORY) {
            std::cout << "[prediction] out of memory!" << std::endl;
            return false;
          }
          double t_cost = (ros::Time::now() - t_start).toSec();
          if (t_cost > max_time) {
            std::cout << "[prediction] too slow!" << std::endl;
            return false;
          }
          NodePtr ptr = data[stack_top++]; // increment stack ptr
          ptr->p = p; // set this ptr to the predicted position and velocity
          ptr->v = v;
          ptr->a = input;
          ptr->parent = curPtr;
          ptr->t = curPtr->t + dt;
          ptr->score = curPtr->score + score(ptr);
          ptr->h = calH(ptr);
          open_set.push(ptr); // open set must be the possible predicted trajectories of the target?
          // std::cout << "open set push: " << state.transpose() << std::endl;
        }
      if (open_set.empty()) {
        std::cout << "[prediction] no way!" << std::endl;
        return false;
      }
      curPtr = open_set.top(); // get ptr to latest prediction
      open_set.pop();
    }
    target_predcit.clear();
    while (curPtr != nullptr) { // add predicted positions from end to start
      target_predcit.push_back(curPtr->p);
      curPtr = curPtr->parent; 
    }
    std::reverse(target_predcit.begin(), target_predcit.end()); // reverse predicted positions so it goes from start -> end
    return true;
  }
};

}  // namespace prediction
