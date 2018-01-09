#ifndef EFFORT_CONTROLLERS__UTILS
#define EFFORT_CONTROLLERS__UTILS

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <tinyxml.h>
#include <iostream>
#include <iostream>
#include <climits>

namespace custom_controller
{

const int NON_EXISTING = INT_MAX;

/** @brief this class provides mappings between dofs as defined in a controller
 * yaml file for loaded joints and loaded urdf file **/
class MapToUrdf
{

private:
  bool valid = false;
  /** mapping from controller to urdf, in a size of controller dofs.
   *  Elements defined in a controller and not defined in urdf are marked as
   */
  std::vector<int> id;
  int dofs; // controller dofs as defined in yaml file
  std::vector<std::string> controller_names;
  std::vector<std::string> urdf_names;
  void initCallback(const sensor_msgs::JointState::ConstPtr& msg);

public:
  MapToUrdf(std::string topic = "joint_states",
            std::string controller = "position_controller");
  ~MapToUrdf() {}
  std::vector<int> getMapping() { return id; }

  /**
   * @brief the function provides mapping between controller and
   *urdf(joint_states
   *feedback) values, as urdf vector may be bigger than urdf vector (flaoting
   *base
   *case), it doesn't overwrite values not present in urdf
   *
   * @note q_mapped must be of the proper size, it is not checked for
   *perfromance
   *purposes, if q_mapped is too small it may cause memory violation
   */
  template <typename Vector>
  void mapToController(Vector q_urdf, Vector& q_mapped)
  {

    for (int i = 0; i < dofs; i++)
    {
      if (id[i] != NON_EXISTING && id[i] < q_urdf.size())
        q_mapped[i] = q_urdf[id[i]];
    }
  }

  /**
   * @brief the function provides mapping between controller and
   *urdf(joint_states
   *feedback) values, as urdf vector may be bigger than urdf vector (flaoting
   *base
   *case), it doesn't overwrite values not present in controller
   *
   * @note q_urdf must be of the proper size, it is not checked for perfromance
   *purposes, if q_urdf is too small it may cause memory violation
   */
  template <typename Vector>
  void mapFromController(Vector q_controller, Vector& q_urdf)
  {

    for (int i = 0; i < dofs; i++)
    {
      if (id[i] != NON_EXISTING && id[i] < q_urdf.size())
        q_urdf[id[i]] = q_controller[i];
    }
  }

  std::vector<std::string> getControllerNames() { return controller_names; }
  std::vector<std::string> getUrdfNames() { return urdf_names; }
  int getControllerDofs() { return dofs; }
};

} // namespace

#endif
