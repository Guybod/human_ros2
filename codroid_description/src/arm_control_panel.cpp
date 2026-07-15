#include "codroid_description/arm_control_panel.hpp"

#include <functional>
#include <cmath>
#include <utility>

#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QMetaObject>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace codroid_description
{

ArmControlPanel::ArmControlPanel(QWidget * parent)
: rviz_common::Panel(parent), status_label_(new QLabel(tr("等待 RViz 初始化"))),
  arm_selector_(new QComboBox())
{
  auto * root = new QVBoxLayout(this);

  auto * lifecycle = new QGroupBox(tr("控制器与双臂"));
  auto * lifecycle_layout = new QVBoxLayout(lifecycle);
  addToggleButton(tr("UDP 连接"), "/codroid_arm_bridge/connect", false, lifecycle_layout);
  addToggleButton(tr("机器人使能"), "/codroid_arm_bridge/enable", true, lifecycle_layout);
  addToggleButton(tr("获取控制权"), "/codroid_arm_bridge/lock", true, lifecycle_layout);
  addToggleButton(tr("左臂实时控制"), "/codroid_arm_bridge/left_control", true, lifecycle_layout);
  addToggleButton(tr("右臂实时控制"), "/codroid_arm_bridge/right_control", true, lifecycle_layout);
  root->addWidget(lifecycle);

  auto * actions = new QHBoxLayout();
  auto * reset = new QPushButton(tr("错误复位"));
  auto * cancel = new QPushButton(tr("取消轨迹"));
  connect(reset, &QPushButton::clicked, this, &ArmControlPanel::resetError);
  connect(cancel, &QPushButton::clicked, this, &ArmControlPanel::cancelTrajectory);
  actions->addWidget(reset);
  actions->addWidget(cancel);
  root->addLayout(actions);

  auto * pose_group = new QGroupBox(tr("末端目标 Pose (base_link)"));
  auto * pose_layout = new QFormLayout(pose_group);
  arm_selector_->addItems({tr("右臂"), tr("左臂")});
  pose_layout->addRow(tr("机械臂"), arm_selector_);
  const std::array<QString, 7> labels = {"X (m)", "Y (m)", "Z (m)", "Qx", "Qy", "Qz", "Qw"};
  for (std::size_t index = 0; index < pose_fields_.size(); ++index) {
    auto * field = new QDoubleSpinBox();
    field->setDecimals(6);
    field->setSingleStep(index < 3 ? 0.01 : 0.05);
    field->setRange(index < 3 ? -2.0 : -1.0, index < 3 ? 2.0 : 1.0);
    pose_fields_[index] = field;
    pose_layout->addRow(labels[index], field);
  }
  pose_fields_[6]->setValue(1.0);
  auto * send_pose = new QPushButton(tr("发送 Pose 目标"));
  send_pose->setStyleSheet("font-weight: bold; padding: 6px;");
  connect(send_pose, &QPushButton::clicked, this, &ArmControlPanel::sendPose);
  pose_layout->addRow(send_pose);
  root->addWidget(pose_group);

  status_label_->setWordWrap(true);
  status_label_->setStyleSheet("padding: 5px; background: #303030;");
  root->addWidget(status_label_);
  root->addStretch();
}

void ArmControlPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    setStatus(tr("无法获取 RViz ROS 节点"), true);
    return;
  }
  node_ = abstraction->get_raw_node();
  left_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/codroid/left_arm/pose_target", 1);
  right_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/codroid/right_arm/pose_target", 1);
  setStatus(tr("面板就绪；发送真机指令前请确认急停可用"));
}

void ArmControlPanel::addToggleButton(
  const QString & label, const std::string & service,
  bool confirmation_required, QVBoxLayout * layout)
{
  auto * button = new QPushButton(label + tr("：关"));
  button->setCheckable(true);
  button->setProperty("base_label", label);
  connect(button, &QPushButton::toggled, this,
    [this, service, button, confirmation_required](bool checked) {
      callSetBool(service, checked, button, confirmation_required);
    });
  layout->addWidget(button);
}

void ArmControlPanel::callSetBool(
  const std::string & service, bool value, QPushButton * button,
  bool confirmation_required)
{
  if (!node_) {
    button->setChecked(!value);
    setStatus(tr("ROS 节点尚未初始化"), true);
    return;
  }
  if (value && confirmation_required) {
    const auto answer = QMessageBox::warning(
      this, tr("确认真机操作"),
      tr("即将执行：%1\n请确认机器人周围安全且急停可用。")
      .arg(button->property("base_label").toString()),
      QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel);
    if (answer != QMessageBox::Yes) {
      button->blockSignals(true);
      button->setChecked(false);
      button->blockSignals(false);
      return;
    }
  }

  auto client = node_->create_client<std_srvs::srv::SetBool>(service);
  if (!client->service_is_ready()) {
    button->blockSignals(true);
    button->setChecked(!value);
    button->blockSignals(false);
    setStatus(tr("服务不可用：%1").arg(QString::fromStdString(service)), true);
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = value;
  button->setEnabled(false);
  auto future = client->async_send_request(request,
    [this, button, value, client, service](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result) {
      const auto response = result.get();
      QMetaObject::invokeMethod(this, [this, button, value, response, service]() {
        button->setEnabled(true);
        const bool applied = response->success ? value : !value;
        button->blockSignals(true);
        button->setChecked(applied);
        button->setText(
          button->property("base_label").toString() + (applied ? tr("：开") : tr("：关")));
        button->blockSignals(false);
        setStatus(
          QString::fromStdString(response->message), !response->success);
      }, Qt::QueuedConnection);
    });
  (void)future;
  setStatus(tr("正在调用：%1").arg(QString::fromStdString(service)));
}

void ArmControlPanel::callTrigger(const std::string & service, const QString & action)
{
  if (!node_) {
    setStatus(tr("ROS 节点尚未初始化"), true);
    return;
  }
  auto client = node_->create_client<std_srvs::srv::Trigger>(service);
  if (!client->service_is_ready()) {
    setStatus(tr("服务不可用：%1").arg(QString::fromStdString(service)), true);
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(request,
    [this, client, action](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result) {
      const auto response = result.get();
      QMetaObject::invokeMethod(this, [this, response, action]() {
        setStatus(action + tr("：") + QString::fromStdString(response->message), !response->success);
      }, Qt::QueuedConnection);
    });
}

void ArmControlPanel::sendPose()
{
  if (!node_ || !left_pose_publisher_ || !right_pose_publisher_) {
    setStatus(tr("ROS 节点尚未初始化"), true);
    return;
  }
  const double qx = pose_fields_[3]->value();
  const double qy = pose_fields_[4]->value();
  const double qz = pose_fields_[5]->value();
  const double qw = pose_fields_[6]->value();
  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm < 1e-9) {
    setStatus(tr("四元数长度不能为零"), true);
    return;
  }
  if (QMessageBox::warning(
      this, tr("确认发送 Pose"), tr("将向所选机械臂发送新的末端目标，是否继续？"),
      QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Yes)
  {
    return;
  }
  geometry_msgs::msg::PoseStamped message;
  message.header.stamp = node_->now();
  message.header.frame_id = "base_link";
  message.pose.position.x = pose_fields_[0]->value();
  message.pose.position.y = pose_fields_[1]->value();
  message.pose.position.z = pose_fields_[2]->value();
  message.pose.orientation.x = qx / norm;
  message.pose.orientation.y = qy / norm;
  message.pose.orientation.z = qz / norm;
  message.pose.orientation.w = qw / norm;
  if (arm_selector_->currentIndex() == 0) {
    right_pose_publisher_->publish(message);
    setStatus(tr("已发送右臂 Pose 目标"));
  } else {
    left_pose_publisher_->publish(message);
    setStatus(tr("已发送左臂 Pose 目标"));
  }
}

void ArmControlPanel::resetError()
{
  callTrigger("/codroid_arm_bridge/reset_error", tr("错误复位"));
}

void ArmControlPanel::cancelTrajectory()
{
  callTrigger("/codroid_arm_trajectory/cancel", tr("取消轨迹"));
}

void ArmControlPanel::setStatus(const QString & text, bool error)
{
  status_label_->setText(text);
  status_label_->setStyleSheet(
    error ? "padding: 5px; background: #7a2020;" : "padding: 5px; background: #205020;");
}

}  // namespace codroid_description

PLUGINLIB_EXPORT_CLASS(codroid_description::ArmControlPanel, rviz_common::Panel)
