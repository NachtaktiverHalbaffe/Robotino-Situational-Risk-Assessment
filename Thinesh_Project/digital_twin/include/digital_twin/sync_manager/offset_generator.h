#ifndef OFFSET_GENERATOR_H
#define OFFSET_GENERATOR_H
#include <ros/ros.h>

namespace SyncManager {
  /**
   * @class OffsetManager
   * @brief 
   */
  class OffsetManager {
    public:
      /**
       * @brief  Default constructor for the OffsetManager object
       */
      OffsetManager();
      ~OffsetManager(){}

      /**
       * @brief  Constructor for the OffsetManager object
       * @param  name The name of this planner
       * @param  costmap A pointer to the ROS wrapper of the costmap to use
       */
      //OffsetManager();


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

#endif // OFFSET_GENERATOR_H
