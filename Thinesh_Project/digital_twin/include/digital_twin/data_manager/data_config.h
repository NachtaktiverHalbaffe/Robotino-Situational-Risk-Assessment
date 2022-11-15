#ifndef DATA_CONFIG_H
#define DATA_CONFIG_H

#include <ros/ros.h>

namespace DataManager {
  /**
   * @class DataConfig
   * @brief 
   */
  class DataConfig {
    public:
      /**
       * @brief  Default constructor for the DataConfig object
       */
      DataConfig();
      ~DataConfig(){}

      /**
       * @brief  Constructor for the DataConfig object
       * @param  name The name of this planner
       * @param  costmap A pointer to the ROS wrapper of the costmap to use
       */
      //DataConfig();


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
#endif // DATA_CONFIG_H
