#ifndef DATA_UPLOAD_H
#define DATA_UPLOAD_H

#include <ros/ros.h>

namespace DataManager {
  /**
   * @class DataUpload
   * @brief 
   */
  class DataUpload {
    public:
      /**
       * @brief  Default constructor for the DataUpload object
       */
      DataUpload();
      ~DataUpload(){}

      /**
       * @brief  Constructor for the DataUpload object
       * @param  name The name of this planner
       * @param  costmap A pointer to the ROS wrapper of the costmap to use
       */
      //DataUpload();


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

#endif // DATA_UPLOAD_H
