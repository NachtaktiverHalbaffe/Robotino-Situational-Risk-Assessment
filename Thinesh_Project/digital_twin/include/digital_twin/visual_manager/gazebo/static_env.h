#ifndef STATIC_ENV_H
#define STATIC_ENV_H

#include <ros/ros.h>

namespace VisualManager {
  /**
   * @class StaticEnv
   * @brief 
   */
  class StaticEnv {
    public:
      /**
       * @brief  Default constructor for the StaticEnv object
       */
      StaticEnv();
      ~StaticEnv(){}

      /**
       * @brief  Constructor for the StaticEnv object
       * @param  name The name of this planner
       * @param  costmap A pointer to the ROS wrapper of the costmap to use
       */
      //StaticEnv();


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

#endif // STATIC_ENV_H
