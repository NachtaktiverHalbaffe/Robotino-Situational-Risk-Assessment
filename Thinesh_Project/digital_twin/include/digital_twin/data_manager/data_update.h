#ifndef DATA_UPDATE_H
#define DATA_UPDATE_H
#include <ros/ros.h>

namespace DataManager {
  /**
   * @class DataUpdate
   * @brief 
   */
  class DataUpdate {
    public:
      /**
       * @brief  Default constructor for the DataUpdate object
       */
      DataUpdate();
      ~DataUpdate(){}

      /**
       * @brief  Constructor for the DataUpdate object
       * @param  name The name of this planner
       * @param  costmap A pointer to the ROS wrapper of the costmap to use
       */
      //DataUpdate();


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

#endif // DATA_UPDATE_H
