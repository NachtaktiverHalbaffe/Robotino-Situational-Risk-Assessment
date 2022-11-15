#ifndef SAFEGUARD_H
#define SAFEGUARD_H
#include <ros/ros.h>

namespace SafetyManager {
  /**
   * @class Safeguard
   * @brief 
   */
  class Safeguard {
    public:
      /**
       * @brief  Default constructor for the Safeguard object
       */
      Safeguard();
      ~Safeguard(){}

      /**
       * @brief  Constructor for the Safeguard object
       * @param  name The name of this planner
       * @param  costmap A pointer to the ROS wrapper of the costmap to use
       */
      //Safeguard();


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

#endif // SAFEGUARD_H
