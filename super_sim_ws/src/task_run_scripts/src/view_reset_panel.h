#ifndef TASK_RUN_SCRIPTS_VIEW_RESET_PANEL_H
#define TASK_RUN_SCRIPTS_VIEW_RESET_PANEL_H

#include <rviz/panel.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>

namespace rviz { class ViewManager; class Property; }

namespace task_run_scripts
{

// 视角切换面板（无 UI 按钮，全部通过话题控制，平板端按钮发指令）
//   /rviz/reset_view  -> 切换到 FPV 视角（第一人称，~/.config/liyumen/rviz_fpv_view.json，启动时保存 rviz 配置预设视角，无则用默认）
//   /rviz/save_view   -> 切换到自由视角（自动记录的用户手动移动后的视角）
// 用户手动移动 rviz 视角时自动记录为自由视角；启动后自动全屏
class ViewResetPanel : public rviz::Panel
{
    Q_OBJECT
public:
    explicit ViewResetPanel(QWidget* parent = nullptr);
    void onInitialize() override;

private Q_SLOTS:
    void applyFreeView();
    void applyFpvView();
    void captureFreeView();
    void captureFpvView();

private:
    void onResetTopic(const std_msgs::EmptyConstPtr& msg);
    void onSaveTopic(const std_msgs::EmptyConstPtr& msg);
    void onViewChanged();
    void connectPropertyChanged(rviz::Property* prop);
    void makeFullscreen();
    void applyViewFromFile(const QString& path, const QJsonObject& fallback);
    void captureViewToFile(const QString& path, const QString& tag);

    QString freeViewFile() const;
    QString fpvViewFile() const;

    rviz::ViewManager* viewManager_ = nullptr;
    ros::Subscriber resetSub_;
    ros::Subscriber saveSub_;
    bool applying_ = false;
    bool viewChangedScheduled_ = false;
};

}  // namespace task_run_scripts

#endif