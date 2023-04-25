#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>

MOVEIT_CLASS_FORWARD( CS3891Context );

class CS3891Context : public planning_interface::PlanningContext {

public:

  typedef std::size_t index;
  typedef std::vector<double> vertex;
  typedef std::pair<vertex,vertex> edge;
  typedef std::vector<vertex> path;

   // kd tree node structure
  struct Node {
    vertex config;
    Node* left;
    Node* right;
    Node* pat;
    int depth;

    Node(const vertex& config, const int depth, Node* pat) : config(config), left(nullptr), right(nullptr), pat(pat), depth(depth) {}

    ~Node() {
        delete left;
        delete right;
    }
  };


  CS3891Context( const robot_model::RobotModelConstPtr& model, 
		const std::string &name, 
		const std::string& group, 
		const ros::NodeHandle &nh = ros::NodeHandle("~") );


  virtual ~CS3891Context();

  virtual bool solve( planning_interface::MotionPlanResponse &res );
  virtual bool solve( planning_interface::MotionPlanDetailedResponse &res );

  virtual void clear();
  virtual bool terminate();

  /**
     Utility method
     Determine if a configuration collides with obstacles.

     \input q Robot configuration
     \return True if the robot collides with an obstacle. False otherwise.
  */
  bool is_colliding( const vertex& q ) const;

  /**
     Utility method
     Interpolate between two configurations.

     \input qA The first configuration
     \input qB The second configuration
     \input t The interpolation parameter t=[0,1]
     \return The interpolated configuration
  */
  vertex interpolate( const vertex& qA, const vertex& qB, double t );


  /**
     TODO

     Compute the distance between two configurations. You decide which metric you want to use
     \input q1 First configuration
     \input q2 Second configuration
     \return distance between both configurations
  */
  double distance( const vertex& q1, const vertex& q2 );
  
  /**
     TODO

     Create a random sample. The retunred vertex represent a collision-free
     configuration (i.e. 6 joints) of the robot.
     \return  A collision-free configuration
  */
  vertex random_sample( const vertex& q_goal )const;

  /**
     TODO

     Find the nearest configuration in the tree to a random sample
     \input q_rand The random configuration
     \return The nearest vertex in the tree
  */
  Node* nearest_configuration( const vertex& q_rand );
  
  /**
     TODO

     Determine if a straight line path is collision free between two configurations.
     \input q_near The first configuration
     \input q_rand The second configuration
     \return True if the path is collision free. False otherwise.
  */
  bool is_subpath_collision_free(Node* q_near, const vertex &q_new, double stepSize);
  
  /**
     TODO
     
     Once the goal configuration has been added to the tree. Search the tree to find and return the 
     path between the root (q_init) and the goal (q_goal).
     \input q_init The root configuration
     \input q_goal The goal configuration
     \return The path (a sequence of configurations) between q_init and q_goal.
  */
  path search_path(Node* rootNode);

  /**
     TODO
     
     This is the main RRT algorithm that adds vertices/edges to a tree and search for a path.
     \input q_init The initial configuration
     \input q_goal The goal configuration
     \return The path between q_init and q_goal
  */
  path rrt( const vertex& q_init, const vertex& q_goal );


   // find shortest path
  path shortestPath(Node *node);

  // find nearest neighbor
  Node* nearest_neighbor(Node* node, const vertex& target_config,  Node* best_node);

// add new node to kdtree
  Node* insert(Node *node, const vertex &config, int &depth, Node* pat);
  
 protected:

  robot_model::RobotModelConstPtr robotmodel;

  // goal bias
  double GOAL_BIAS = 0.05;
   // step size
  double STEP_SIZE = 0.1;

   // root of constructed kd tree
  Node* root;

  // num of joints
   int k = 18;

  // episilon for testing equality...
  double epi = 1e-4;

};
