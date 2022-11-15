#ifndef CO_SIM_IF_H
#define CO_SIM_IF_H

#include <ros/ros.h>

namespace VisualManager {
  /**
   * @class CoSimIF
   * @brief 
   */
  class CoSimIF {
    public:
      /**
       * @brief  Default constructor for the CoSimIF object
       */
      CoSimIF();
      ~CoSimIF(){}

      /**
       * @brief  Constructor for the CoSimIF object
       * @param  name The name of this planner
       * @param  costmap A pointer to the ROS wrapper of the costmap to use
       */
      //CoSimIF();


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

#endif // CO_SIM_IF_H
