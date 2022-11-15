#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H
#include <ros/ros.h>

namespace SafetyManager {
  /**
   * @class RobotControl
   * @brief 
   */
  class RobotControl {
    public:
      /**
       * @brief  Default constructor for the RobotControl object
       */
      RobotControl();
      ~RobotControl(){}

      /**
       * @brief  Constructor for the RobotControl object
       * @param  name The name of this planner
       * @param  costmap A pointer to the ROS wrapper of the costmap to use
       */
      //RobotControl();


      /**
       * @brief  Initialization function for the NavFnROS object
       * @param  name The name of this planner
       * @param  costmap A pointer to the ROS wrapper of the costmap to use for planning
       */
      //void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);





    protected:

      /**
       * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
       */



    private:

  };
};

#endif // ROBOT_CONTROL_H
