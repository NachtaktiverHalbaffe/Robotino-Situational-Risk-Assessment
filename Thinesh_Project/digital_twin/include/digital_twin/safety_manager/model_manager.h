#ifndef MODEL_MANAGER_H
#define MODEL_MANAGER_H
#include <ros/ros.h>

namespace SafetyManager {
  /**
   * @class ModelManager
   * @brief 
   */
  class ModelManager {
    public:
      /**
       * @brief  Default constructor for the ModelManager object
       */
      ModelManager();
      ~ModelManager(){}

      /**
       * @brief  Constructor for the ModelManager object
       * @param  name The name of this planner
       * @param  costmap A pointer to the ROS wrapper of the costmap to use
       */
      //ModelManager();


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

#endif // MODEL_MANAGER_H
