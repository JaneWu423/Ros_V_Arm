#include "assignment_context.h"
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <cstdlib>
#include <cmath>
#include <iostream>

void help_print(const CS3891Context::vertex &q)
{
  for (auto i : q)
  {
    std::cout << i << " ";
  }
  std::cout << std::endl;
  return;
}

// utility function to test for a collision
bool CS3891Context::is_colliding(const vertex &q) const
{
  moveit::core::RobotState robotstate(robotmodel);
  if (getGroupName() == "left_arm")
  {

    robotstate.setJointGroupPositions("left_arm", q);
  }
  if (getGroupName() == "right_arm")
  {

    robotstate.setJointGroupPositions("right_arm", q);
  }
  if (getGroupName() == "both_arms")
  {
    vertex q_both(q.begin(), q.begin() + 7);
    q_both.insert(q_both.end(), q.begin() + 9, q.begin() + 16);
    robotstate.setJointGroupPositions("both_arms", q_both);
  }

  if (getPlanningScene()->isStateColliding(robotstate, getGroupName(), false))
  {
    return true;
  }
  else
  {
    return false;
  }
}

// utility function to interpolate between two configurations
CS3891Context::vertex CS3891Context::interpolate(const CS3891Context::vertex &qA,
                                                 const CS3891Context::vertex &qB,
                                                 double t)
{
  CS3891Context::vertex qt(qA.size(), 0.0);
  for (std::size_t i = 0; i < qt.size(); i++)
  {
    qt[i] = (1.0 - t) * qA[i] + t * qB[i];
  }
  return qt;
}

CS3891Context::CS3891Context(const robot_model::RobotModelConstPtr &robotmodel,
                             const std::string &name,
                             const std::string &group,
                             const ros::NodeHandle &nh) : planning_interface::PlanningContext(name, group),
                                                          robotmodel(robotmodel) {}

CS3891Context::~CS3891Context() {}

// TODO
CS3891Context::vertex CS3891Context::random_sample(const CS3891Context::vertex &q_goal) const
{
  CS3891Context::vertex q_rand(q_goal.size(), 0.0);

  // TODO return a random sample q_rand with goal bias.
  double r = (double)std::rand() / RAND_MAX;
  if (r < GOAL_BIAS)
  {
    return q_goal;
  }
  else
  {
    if (getGroupName() != "both_arms")
    {
      for (std::size_t i = 0; i < 7; i++)
      {
        double rand_num = ((double)std::rand()) / RAND_MAX;
        double radian = -M_PI + rand_num * 2 * M_PI;
        q_rand[i] = radian;
      }
    }
    else
    {
      for (std::size_t i = 0; i < 7; i++)
      {
        double rand_num_1 = ((double)std::rand()) / RAND_MAX;
        double rand_num_2 = ((double)std::rand()) / RAND_MAX;
        double radian_1 = -M_PI + rand_num_1 * 2 * M_PI;
        double radian_2 = -M_PI + rand_num_2 * 2 * M_PI;
        q_rand[i] = radian_1;
        q_rand[i + 9] = radian_2;
      }
    }
    return q_rand;
  }
}

// TODO
double CS3891Context::distance(const CS3891Context::vertex &q1, const CS3891Context::vertex &q2)
{

  double dot_product = 0.0;
  double norm_q1 = 0.0;
  double norm_q2 = 0.0;
  double cosine_dist = 0.0;

    // Compute the dot product and the norms of the two configurations
    for (size_t i = 0; i < q1.size(); i++) {
        dot_product += q1[i] * q2[i];
        norm_q1 += q1[i] * q1[i];
        norm_q2 += q2[i] * q2[i];
    }

    // Compute the cosine distance between the two configurations
    if (norm_q1 > epi && norm_q2 > epi) {
        cosine_dist = dot_product / (sqrt(norm_q1) * sqrt(norm_q2));
    } else if(norm_q1 < epi && norm_q2 < epi){
      return 0;
    } else if (norm_q1 < epi){
      norm_q1 = k*epi*epi;
      for (size_t i = 0; i < q1.size(); i++) {
        dot_product += epi * q2[i];
      }
    } else{
      norm_q2 = k*epi*epi;
      for (size_t i = 0; i < q1.size(); i++) {
        dot_product += epi * q1[i];
      }
    }

    return 1-cosine_dist;

//   double distance = 0.0;
//   for (std::size_t i = 0; i < q1.size(); i++)
//   {
//     distance += std::pow(q1[i] - q2[i], 2);
//   }
//   return std::sqrt(distance);
}

// TODO
CS3891Context::Node *CS3891Context::nearest_configuration(const CS3891Context::vertex &q_rand)
{
  // TODO find the nearest configuration in the tree to q_rand

  Node *best_node = root;

  Node *q_near = nearest_neighbor(root, q_rand, best_node);

  return q_near;
}

// TODO
bool CS3891Context::is_subpath_collision_free(CS3891Context::Node *q_near,
                                              const CS3891Context::vertex &q_new,
                                              double stepSize)
{
  vertex qt;
  int substep = 20;
  for (int i = 1; i <= substep; ++i)
  {
    qt = interpolate(q_near->config, q_new, stepSize / substep * i);
    if (is_colliding(qt))
    {
      return false;
    }
  }
  return true;
}

// TODO
CS3891Context::path CS3891Context::search_path(CS3891Context::Node *node)
{
  CS3891Context::path P;

  // TODO Once q_goal has been added to the tree, find the path (sequence of configurations) between
  // q_init and q_goal (hint: this is easier by using recursion).
  P = shortestPath(node);

  return P;
}

// TODO
CS3891Context::path CS3891Context::rrt(const CS3891Context::vertex &q_init,
                                       const CS3891Context::vertex &q_goal)
{
  std::cout << "enter rrt" << std::endl;
  if (distance(q_init, q_goal) < epi)
  {
    path P;
    P.push_back(q_goal);
    P.push_back(q_init);
    return P;
  }
  // initialize kd tree
  root = new Node(q_init, 0, nullptr);

  // start while loop
  bool no_goal = true;

  while (no_goal)
  {

    vertex q_rand = random_sample(q_goal);
    while (is_colliding(q_rand))
    {
      q_rand = random_sample(q_goal);
    }

    Node *q_near = nearest_configuration(q_rand);
    // go as far as possible:
    bool reached = false;

    // distance between q_rand and q_goal
    double r_goal = distance(q_rand, q_goal);

    while (!reached && is_subpath_collision_free(q_near, q_rand, STEP_SIZE))
    {

      // q_new, one step size towards q_rand
      Node *newAdded = insert(q_near, interpolate(q_near->config, q_rand, STEP_SIZE), q_near->depth, q_near);

      // distance between q_new and q_goal
      double d_goal = distance(newAdded->config, q_goal);
      // distance between q_rand and q_new
      double d = distance(newAdded->config, q_rand);

      // q_new reached goal
      if (d_goal < epi)
      {
        no_goal = false;
        // return searched path
        return search_path(newAdded);
      }

      if (r_goal > epi || d < epi)
      {
        reached = true;
      }

      q_near = newAdded;
      if(d_goal < 0.01){
          std::cout << d_goal << std::endl;
      }
      
    }
  }

  // TODO implement RRT algorithm and return the path (an ordered sequence of configurations).
}

// This is the method that is called each time a plan is requested
bool CS3891Context::solve(planning_interface::MotionPlanResponse &res)
{

  // Create a new empty trajectory
  res.trajectory_.reset(new robot_trajectory::RobotTrajectory(robotmodel, getGroupName()));
  res.trajectory_->clear();

  // copy the initial/final joints configurations to vectors qfin and qini
  // This is mainly for convenience.
  vertex q_init, q_goal;

  if (getGroupName() != "both_arms")
  {
    for (size_t i = 0; i < 7; i++)
    {
      if (getGroupName() == "right_arm")
      {
        q_goal.push_back(request_.goal_constraints[0].joint_constraints[i + 9].position);
        q_init.push_back(request_.start_state.joint_state.position[i + 9]);
      }
      if (getGroupName() == "left_arm")
      {
        q_goal.push_back(request_.goal_constraints[0].joint_constraints[i].position);
        q_init.push_back(request_.start_state.joint_state.position[i]);
      }
      // std::cout << "goal 0" << i << " " << request_.goal_constraints[0].joint_constraints[i].position << std::endl;
    }
  }
  else
  {
    for (size_t i = 0; i < k; ++i)
    {
      q_goal.push_back(request_.goal_constraints[0].joint_constraints[i].position);
      q_init.push_back(request_.start_state.joint_state.position[i]);
    }
  }
  
  vertex q_goal_t = {-0.488692, 0.698132, -0.279253, -1.58825, -0.837758, 1.02974, -0.733038, 0, 0, 0.436332, 0.610865, 0.261799, -1.72788, 0.785398, 1.01229, 0.663225, 0, 0};
  // hardcode V shape config
  vertex q_goal_b = {1.97413, -0.733038, -2.89725, -2.00713, -1.09956, 1.13446, -0.279253, 0, 0, 1.19116, 0.750492, -0.226893, -2.0944, 1.18682, 1.30899, 0.20944, 0, 0};
  // starting pose
  vertex q_init_b = {0, 0, 0, -1.5708, 0, 0, 0, 0, 0, 0, 0, 0, -1.5708, 0, 0, 0, 0, 0};

  k = q_init.size();
  if (getGroupName() == "both_arms")
  {
    if(distance(q_init, q_goal_b) > epi){
      q_goal = q_goal_b;
    }else{
      q_goal = q_init_b;
    }
    
  }

  std::cout << "q_init ";
  help_print(q_init);
  std::cout << "q_goal ";
  help_print(q_goal);

  std::cout << "size of joints: " << q_init.size() << std::endl;
  std::cout << "group name is: " << getGroupName() << std::endl;

  // start the timer
  ros::Time begin = ros::Time::now();

  // ------- search for goal
  path P = rrt(q_init, q_goal);

  // ------ testing
  // for (vertex a : P)
  // {
  //   std::cout << "found path is: ";
  //   help_print(a);
  // }
  // std::cout << "finished path " << std::endl;
  // ------ testing

  // end the timer
  ros::Time end = ros::Time::now();

  // The rest is to fill in the animation. You can ignore this part.
  moveit::core::RobotState robotstate(robotmodel);

  if (getGroupName() == "left_arm")
  {
    vertex q_left(q_init.begin(), q_init.begin() + 7);
    robotstate.setJointGroupPositions("left_arm", q_left);
  }
  if (getGroupName() == "right_arm")
  {
    vertex q_right(q_init.begin(), q_init.begin() + 7);
    robotstate.setJointGroupPositions("right_arm", q_right);
  }
  if (getGroupName() == "both_arms")
  {
    vertex q_both(q_init.begin(), q_init.begin() + 7);
    q_both.insert(q_both.end(), q_init.begin() + 9, q_init.begin() + 16);
    robotstate.setJointGroupPositions("both_arms", q_both);
  }

  for (int i = P.size() - 1; i >= 1; i--)
  {
    for (double t = 0.0; t <= 1.0; t += 0.01)
    {
      vertex q = interpolate(P[i], P[i - 1], t);
      if (getGroupName() == "left_arm")
      {
        vertex q_left(q.begin(), q.begin() + 7);
        robotstate.setJointGroupPositions("left_arm", q_left);
      }
      if (getGroupName() == "right_arm")
      {
        vertex q_right(q.begin(), q.begin() + 7);
        robotstate.setJointGroupPositions("right_arm", q_right);
      }

      if (getGroupName() == "both_arms")
      {
        vertex q_both(q.begin(), q.begin() + 7);
        q_both.insert(q_both.end(), q.begin() + 9, q.begin() + 16);
        robotstate.setJointGroupPositions("both_arms", q_both);
      }

      res.trajectory_->addSuffixWayPoint(robotstate, 0.01);
    }
  }

  // set the planning time
  ros::Duration duration = end - begin;
  res.planning_time_ = duration.toSec();
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  return true;
}

bool CS3891Context::solve(planning_interface::MotionPlanDetailedResponse &res)
{
  return true;
}

void CS3891Context::clear() {}

bool CS3891Context::terminate() { return true; }

CS3891Context::Node *CS3891Context::insert(CS3891Context::Node *node, const CS3891Context::vertex &config, int &depth, Node *pat)
{
  if (node == nullptr)
  {
    node = new Node(config, depth, pat);
    return node;
  }
  if (config[depth % k] < node->config[depth % k])
  {
    depth += 1;
    if (node->left != nullptr)
    {
      return insert(node->left, config, depth, pat);
    }
    else
    {
      node->left = new Node(config, depth, pat);
      return node->left;
    }
  }
  else
  {
    depth += 1;
    if (node->right != nullptr)
    {
      return insert(node->right, config, depth, pat);
    }
    else
    {
      node->right = new Node(config, depth, pat);
      return node->right;
    }
  }
}

CS3891Context::Node *CS3891Context::nearest_neighbor(CS3891Context::Node *node, const CS3891Context::vertex &target_config, Node *best_node)
{
  if (node == nullptr || (node->left == nullptr && node->right == nullptr))
  {
    return best_node;
  }

  // Calculate the distance between the search value and the current node's config
  double currentDistance = distance(node->config, target_config);

  // If the current node is closer than the current closest node, update the current closest node
  if (currentDistance < distance(best_node->config, target_config))
  {
    best_node = node;
  }

  // Check which subtree to search next
  int kd = node->depth % k;
  if (node->left != nullptr && target_config[kd] < node->config[kd])
  {
    best_node = nearest_neighbor(node->left, target_config, best_node);
    if (node->right != nullptr && distance(best_node->config, target_config) > distance(node->right->config, target_config))
    {
      best_node = nearest_neighbor(node->right, target_config, best_node);
    }
  }
  else
  {
    best_node = nearest_neighbor(node->right, target_config, best_node);
    if (node->left != nullptr && distance(best_node->config, target_config) > distance(node->left->config, target_config))
    {
      best_node = nearest_neighbor(node->left, target_config, best_node);
    }
  }

  return best_node;
}

CS3891Context::path CS3891Context::shortestPath(CS3891Context::Node *node)
{
  path P;
  if (node == nullptr)
  {
    return P;
  }
  else
  {
    P = shortestPath(node->pat);
    P.insert(P.begin(), node->config);
    return P;
  }
}