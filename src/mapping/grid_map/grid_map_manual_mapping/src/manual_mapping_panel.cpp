// RViz 面板插件：手动建图的三个按钮（定位模式 / 建图模式 / 保存地图）。
// 面板自己起一个独立的回调队列 + AsyncSpinner，不依赖 rviz 主循环是否在 spin，
// 所以订阅 /manual_mapping/status 和发 /manual_mapping/mode 都很稳。
#include "grid_map_manual_mapping/manual_mapping_panel.h"

#include <QButtonGroup>
#include <QFont>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.h>

#include "logManager/logManager.h"

namespace grid_map_manual_mapping
{

namespace
{
// 面板里用的名字默认和 config/manual_mapping.yaml 保持一致；
// 那份 yaml 改了名字的话，下面会从 /manual_mapping_node/* 读回来（节点名固定是 manual_mapping_node）。
const char* const kDefaultModeTopic = "/manual_mapping/mode";
const char* const kDefaultStatusTopic = "/manual_mapping/status";
const char* const kDefaultSaveService = "/manual_mapping/save_map";
}  // namespace

ManualMappingPanel::ManualMappingPanel(QWidget* parent)
    : rviz::Panel(parent),
      nodeHandle_(),
      spinner_(1, &callbackQueue_),
      localizationButton_(nullptr),
      mappingButton_(nullptr),
      saveButton_(nullptr),
      statusLabel_(nullptr)
{
    std::string modeTopic = kDefaultModeTopic;
    std::string statusTopic = kDefaultStatusTopic;
    std::string saveService = kDefaultSaveService;
    ros::NodeHandle globalNodeHandle;
    globalNodeHandle.param<std::string>("/manual_mapping_node/mode_topic", modeTopic, modeTopic);
    globalNodeHandle.param<std::string>("/manual_mapping_node/status_topic", statusTopic,
                                        statusTopic);
    globalNodeHandle.param<std::string>("/manual_mapping_node/save_service", saveService,
                                        saveService);

    setupUi();

    modePublisher_ = nodeHandle_.advertise<std_msgs::String>(modeTopic, 1);
    saveClient_ = nodeHandle_.serviceClient<std_srvs::Trigger>(saveService);

    ros::SubscribeOptions modeOptions = ros::SubscribeOptions::create<std_msgs::String>(
        modeTopic, 1,
        [this](const std_msgs::String::ConstPtr& message) { modeCallback(message); },
        ros::VoidConstPtr(), &callbackQueue_);
    modeSubscriber_ = nodeHandle_.subscribe(modeOptions);

    ros::SubscribeOptions statusOptions = ros::SubscribeOptions::create<std_msgs::String>(
        statusTopic, 1,
        [this](const std_msgs::String::ConstPtr& message) { statusCallback(message); },
        ros::VoidConstPtr(), &callbackQueue_);
    statusSubscriber_ = nodeHandle_.subscribe(statusOptions);

    spinner_.start();

    statusLabel_->setText(
        QString::fromStdString("就绪。mode: " + modeTopic + "   save: " + saveService));
    LOG(INFO) << "ManualMappingPanel up: mode topic " << modeTopic << ", save service "
              << saveService;
}

ManualMappingPanel::~ManualMappingPanel()
{
    spinner_.stop();
}

void ManualMappingPanel::load(const rviz::Config& config)
{
    rviz::Panel::load(config);
}

void ManualMappingPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
}

void ManualMappingPanel::setupUi()
{
    QVBoxLayout* layout = new QVBoxLayout;

    QLabel* title = new QLabel(QString::fromUtf8("手动建图 / 定位"));
    QFont titleFont = title->font();
    titleFont.setBold(true);
    title->setFont(titleFont);
    layout->addWidget(title);

    localizationButton_ = new QPushButton(QString::fromUtf8("定位模式"));
    mappingButton_ = new QPushButton(QString::fromUtf8("建图模式"));
    localizationButton_->setCheckable(true);
    mappingButton_->setCheckable(true);

    QButtonGroup* modeGroup = new QButtonGroup(this);
    modeGroup->setExclusive(true);
    modeGroup->addButton(localizationButton_);
    modeGroup->addButton(mappingButton_);

    QHBoxLayout* modeRow = new QHBoxLayout;
    modeRow->addWidget(localizationButton_);
    modeRow->addWidget(mappingButton_);
    layout->addLayout(modeRow);

    saveButton_ = new QPushButton(QString::fromUtf8("保存地图"));
    layout->addWidget(saveButton_);

    statusLabel_ = new QLabel(QString::fromUtf8("状态：等待 manual_mapping_node ..."));
    statusLabel_->setWordWrap(true);
    layout->addWidget(statusLabel_);

    layout->addStretch(1);
    setLayout(layout);

    localizationButton_->setChecked(true);

    QObject::connect(localizationButton_, &QPushButton::clicked, this,
                     &ManualMappingPanel::onLocalizationClicked);
    QObject::connect(mappingButton_, &QPushButton::clicked, this,
                     &ManualMappingPanel::onMappingClicked);
    QObject::connect(saveButton_, &QPushButton::clicked, this, &ManualMappingPanel::onSaveClicked);
}

void ManualMappingPanel::publishMode(const std::string& mode)
{
    std_msgs::String message;
    message.data = mode;
    modePublisher_.publish(message);
    LOG(INFO) << "面板按下按钮，发布模式 " << mode;
}

void ManualMappingPanel::onLocalizationClicked()
{
    publishMode("localization");
}

void ManualMappingPanel::onMappingClicked()
{
    publishMode("mapping");
}

void ManualMappingPanel::onSaveClicked()
{
    if (!saveClient_.waitForExistence(ros::Duration(1.0)))
    {
        statusLabel_->setText(
            QString::fromUtf8("保存失败：服务不可用，manual_mapping_node 是不是没在跑？"));
        return;
    }
    std_srvs::Trigger service;
    if (saveClient_.call(service) && service.response.success)
    {
        statusLabel_->setText(QString::fromStdString(service.response.message));
    }
    else
    {
        statusLabel_->setText(QString::fromUtf8("保存失败：") +
                              QString::fromStdString(service.response.message));
    }
}

void ManualMappingPanel::modeCallback(const std_msgs::String::ConstPtr& message)
{
    if (message->data == "mapping")
    {
        mappingButton_->setChecked(true);
    }
    else if (message->data == "localization")
    {
        localizationButton_->setChecked(true);
    }
}

void ManualMappingPanel::statusCallback(const std_msgs::String::ConstPtr& message)
{
    statusLabel_->setText(QString::fromStdString(message->data));
}

}  // namespace grid_map_manual_mapping

// 注册成 rviz 面板插件（没有这一句，rviz 会报
// "PluginlibFactory: ... failed to load. Could not create class of type ..."）
PLUGINLIB_EXPORT_CLASS(grid_map_manual_mapping::ManualMappingPanel, rviz::Panel)
