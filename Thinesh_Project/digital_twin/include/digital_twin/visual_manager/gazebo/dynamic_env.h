#ifndef DYNAMIC_ENV_H
#define DYNAMIC_ENV_H

#include <ros/ros.h>

namespace VisualManager {
  /**
   * @class DynamicEnv
   * @brief 
   */
  class DynamicEnv {
    public:
      /**
       * @brief  Default constructor for the DynamicEnv object
       */
      DynamicEnv();
      ~DynamicEnv(){}

      /**
       * @brief  Constructor for the DynamicEnv object
       * @param  name The name of this planner
       * @param  costmap A pointer to the ROS wrapper of the costmap to use
       */
      //DynamicEnv();


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

#endif // DYNAMIC_ENV_H
