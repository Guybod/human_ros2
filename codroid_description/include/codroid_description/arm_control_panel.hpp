#ifndef CODROID_DESCRIPTION__ARM_CONTROL_PANEL_HPP_
#define CODROID_DESCRIPTION__ARM_CONTROL_PANEL_HPP_

#include <array>
#include <memory>
#include <string>

#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QVBoxLayout>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace codroid_description
{

class ArmControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ArmControlPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void sendPose();
  void resetError();
  void cancelTrajectory();

private:
  void addToggleButton(
    const QString & label, const std::string & service,
    bool confirmation_required, QVBoxLayout * layout);
  void callSetBool(
    const std::string & service, bool value, QPushButton * button,
    bool confirmation_required);
  void callTrigger(const std::string & service, const QString & action);
  void setStatus(const QString & text, bool error = false);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_publisher_;

  QLabel * status_label_;
  QComboBox * arm_selector_;
  std::array<QDoubleSpinBox *, 7> pose_fields_;
};

}  // namespace codroid_description

#endif  // CODROID_DESCRIPTION__ARM_CONTROL_PANEL_HPP_
