#ifndef DEVIATION_DETECTOR_H
#define DEVIATION_DETECTOR_H
#include <ros/ros.h>

namespace SyncManager {
  /**
   * @class DeviationDetector
   * @brief 
   */
  class DeviationDetector {
    public:
      /**
       * @brief  Default constructor for the DeviationDetector object
       */
      DeviationDetector();
      ~DeviationDetector(){}

      /**
       * @brief  Constructor for the DeviationDetector object
       * @param  name The name of this planner
       * @param  costmap A pointer to the ROS wrapper of the costmap to use
       */
      //DeviationDetector();


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

#endif // DEVIATION_DETECTOR_H
