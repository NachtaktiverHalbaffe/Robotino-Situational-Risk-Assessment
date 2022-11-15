#ifndef ROUTE_SYNC_H
#define ROUTE_SYNC_H

#include <ros/ros.h>

namespace SyncManager {
  /**
   * @class RouteSync
   * @brief 
   */
  class RouteSync {
    public:
      /**
       * @brief  Default constructor for the RouteSync object
       */
      RouteSync();
      ~RouteSync(){}

      /**
       * @brief  Constructor for the RouteSync object
       * @param  name The name of this planner
       * @param  costmap A pointer to the ROS wrapper of the costmap to use
       */
      //RouteSync();


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

#endif // ROUTE_SYNC_H
