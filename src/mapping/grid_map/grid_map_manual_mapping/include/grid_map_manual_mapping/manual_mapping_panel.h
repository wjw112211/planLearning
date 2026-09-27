#ifndef GRID_MAP_MANUAL_MAPPING_MANUAL_MAPPING_PANEL_H_
#define GRID_MAP_MANUAL_MAPPING_MANUAL_MAPPING_PANEL_H_

#include <string>

#include <QLabel>
#include <QPushButton>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

namespace grid_map_manual_mapping
{

// RViz 里的"手动建图"面板：三个按钮 + 一行状态说明。
//   定位模式 : 往 mode_topic 发 "localization"（manual_mapping_node 会加载 maps/ 里最新的 pgm）
//   建图模式 : 往 mode_topic 发 "mapping"（清空地图，接着用键盘 wsad 边走边建）
//   保存地图 : 调 save_service（std_srvs/Trigger），写成 pgm + yaml
// 状态文字订阅 status_topic，显示在面板最下面。
class ManualMappingPanel : public rviz::Panel
{
  Q_OBJECT
public:
  explicit ManualMappingPanel(QWidget* parent = nullptr);
  ~ManualMappingPanel() override;

  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

private Q_SLOTS:
  void onLocalizationClicked();
  void onMappingClicked();
  void onSaveClicked();

private:
  void setupUi();
  void publishMode(const std::string& mode);
  void modeCallback(const std_msgs::String::ConstPtr& message);
  void statusCallback(const std_msgs::String::ConstPtr& message);

  ros::NodeHandle nodeHandle_;
  ros::Publisher modePublisher_;
  ros::Subscriber modeSubscriber_;
  ros::Subscriber statusSubscriber_;
  ros::ServiceClient saveClient_;
  ros::CallbackQueue callbackQueue_;
  ros::AsyncSpinner spinner_;

  QPushButton* localizationButton_;
  QPushButton* mappingButton_;
  QPushButton* saveButton_;
  QLabel* statusLabel_;
};

}  // namespace grid_map_manual_mapping

#endif  // GRID_MAP_MANUAL_MAPPING_MANUAL_MAPPING_PANEL_H_
