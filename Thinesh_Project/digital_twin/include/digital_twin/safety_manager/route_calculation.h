#ifndef ROUTE_CALCULATION_H
#define ROUTE_CALCULATION_H
#include <ros/ros.h>

namespace SafetyManager {
  /**
   * @class RouteCalculation
   * @brief 
   */
  class RouteCalculation {
    public:
      /**
       * @brief  Default constructor for the RouteCalculation object
       */
      RouteCalculation();
      ~RouteCalculation(){}

      /**
       * @brief  Constructor for the RouteCalculation object
       * @param  name The name of this planner
       * @param  costmap A pointer to the ROS wrapper of the costmap to use
       */
      //RouteCalculation();


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

#endif // ROUTE_CALCULATION_H
