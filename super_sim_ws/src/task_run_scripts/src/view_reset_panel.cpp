#include "view_reset_panel.h"

#include <rviz/view_manager.h>
#include <rviz/view_controller.h>
#include <rviz/properties/property.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/vector_property.h>
#include <OgreVector3.h>

#include <QApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>
#include <QVariant>
#include <QStringList>

namespace task_run_scripts
{

ViewResetPanel::ViewResetPanel(QWidget* parent)
    : rviz::Panel(parent)
{
    // 无 UI 按钮：控制全部走话题（平板端按钮），这里只保留一个不可见的占位
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(new QLabel(QStringLiteral(""), this));
}

QString ViewResetPanel::freeViewFile() const
{
    return QDir::homePath() + QStringLiteral("/.config/liyumen/rviz_free_view.json");
}

QString ViewResetPanel::fpvViewFile() const
{
    return QDir::homePath() + QStringLiteral("/.config/liyumen/rviz_fpv_view.json");
}

void ViewResetPanel::onInitialize()
{
    viewManager_ = vis_manager_->getViewManager();
    ros::NodeHandle nh;
    resetSub_ = nh.subscribe("/rviz/reset_view", 1, &ViewResetPanel::onResetTopic, this);
    saveSub_ = nh.subscribe("/rviz/save_view", 1, &ViewResetPanel::onSaveTopic, this);

    // 监听视角变化（用户手动拖动），自动记录为自由视角
    // 注意：拖拽改变的是 Yaw/Pitch/Distance/FocalPoint 等子属性，需递归监听子属性 changed
    if (viewManager_) {
        rviz::ViewController* vc = viewManager_->getCurrent();
        if (vc) {
            connectPropertyChanged(vc);
        }
        connect(viewManager_, &rviz::ViewManager::currentChanged,
                this, [this]() {
                    onViewChanged();
                    rviz::ViewController* vc = viewManager_->getCurrent();
                    if (vc) connectPropertyChanged(vc);
                });
    }

    // rviz 加载完成后自动全屏
    QTimer::singleShot(3000, this, &ViewResetPanel::makeFullscreen);
    // 启动后把初始视角记录为自由视角（用户可在此基础上移动）
    QTimer::singleShot(4000, this, &ViewResetPanel::captureFreeView);
    // 启动后把 rviz 配置的预设视角保存为 FPV（平板"FPV 第一视角"按钮恢复它）
    QTimer::singleShot(4000, this, &ViewResetPanel::captureFpvView);
}

void ViewResetPanel::makeFullscreen()
{
    for (QWidget* w : QApplication::topLevelWidgets()) {
        if (w->inherits("QMainWindow")) {
            w->showFullScreen();
            ROS_INFO("[ViewReset] rviz fullscreen enabled");
            break;
        }
    }
}

void ViewResetPanel::onResetTopic(const std_msgs::EmptyConstPtr& msg)
{
    QMetaObject::invokeMethod(this, "applyFpvView", Qt::QueuedConnection);
}

void ViewResetPanel::onSaveTopic(const std_msgs::EmptyConstPtr& msg)
{
    QMetaObject::invokeMethod(this, "applyFreeView", Qt::QueuedConnection);
}

void ViewResetPanel::connectPropertyChanged(rviz::Property* prop)
{
    if (!prop) return;
    connect(prop, &rviz::Property::changed, this, &ViewResetPanel::onViewChanged);
    for (int i = 0; i < prop->numChildren(); ++i) {
        connectPropertyChanged(prop->childAt(i));
    }
}

void ViewResetPanel::onViewChanged()
{
    if (applying_) return;
    if (viewChangedScheduled_) return;
    viewChangedScheduled_ = true;
    QTimer::singleShot(300, this, &ViewResetPanel::captureFreeView);
}

void ViewResetPanel::captureViewToFile(const QString& path, const QString& tag)
{
    if (!viewManager_) return;
    rviz::ViewController* vc = viewManager_->getCurrent();
    if (!vc) return;

    QJsonObject obj;
    obj[QStringLiteral("Class")] = vc->getClassId();
    obj[QStringLiteral("Yaw")] = vc->subProp(QStringLiteral("Yaw"))->getValue().toDouble();
    obj[QStringLiteral("Pitch")] = vc->subProp(QStringLiteral("Pitch"))->getValue().toDouble();
    obj[QStringLiteral("Distance")] = vc->subProp(QStringLiteral("Distance"))->getValue().toDouble();
    obj[QStringLiteral("Field of View")] = vc->subProp(QStringLiteral("Field of View"))->getValue().toDouble();
    obj[QStringLiteral("Target Frame")] = vc->subProp(QStringLiteral("Target Frame"))->getValue().toString();

    rviz::VectorProperty* fp = dynamic_cast<rviz::VectorProperty*>(vc->subProp(QStringLiteral("Focal Point")));
    if (fp) {
        const Ogre::Vector3 v = fp->getVector();
        QJsonArray arr = { v.x, v.y, v.z };
        obj[QStringLiteral("Focal Point")] = arr;
    }

    QDir().mkpath(QFileInfo(path).absolutePath());
    QFile f(path);
    if (f.open(QIODevice::WriteOnly)) {
        f.write(QJsonDocument(obj).toJson(QJsonDocument::Indented));
        ROS_INFO("[ViewReset] %s view captured", qPrintable(tag));
    }
}

void ViewResetPanel::captureFreeView()
{
    viewChangedScheduled_ = false;
    captureViewToFile(freeViewFile(), QStringLiteral("free"));
}

void ViewResetPanel::captureFpvView()
{
    captureViewToFile(fpvViewFile(), QStringLiteral("FPV"));
}

void ViewResetPanel::applyFreeView()
{
    ROS_INFO("[ViewReset] apply free view");
    QJsonObject fallback;
    applyViewFromFile(freeViewFile(), fallback);
}

void ViewResetPanel::applyFpvView()
{
    ROS_INFO("[ViewReset] apply FPV view");
    QJsonObject fallback;
    fallback[QStringLiteral("Class")] = QStringLiteral("rviz/FPS");
    fallback[QStringLiteral("Yaw")] = 0.0;
    fallback[QStringLiteral("Pitch")] = 0.0;
    fallback[QStringLiteral("Field of View")] = 1.0;
    fallback[QStringLiteral("Position")] = QJsonArray({ 0, 0, 0 });
    // 保留当前 Target Frame，跟随当前坐标系
    if (viewManager_) {
        rviz::ViewController* cur = viewManager_->getCurrent();
        if (cur) fallback[QStringLiteral("Target Frame")] = cur->subProp(QStringLiteral("Target Frame"))->getValue().toString();
    }
    applyViewFromFile(fpvViewFile(), fallback);
}

void ViewResetPanel::applyViewFromFile(const QString& path, const QJsonObject& fallback)
{
    if (!viewManager_) return;

    QJsonObject obj;
    QFile f(path);
    if (f.open(QIODevice::ReadOnly)) {
        obj = QJsonDocument::fromJson(f.readAll()).object();
    }
    if (obj.isEmpty()) {
        obj = fallback;
        if (obj.isEmpty()) {
            ROS_WARN("[ViewReset] no view to apply");
            return;
        }
    }

    applying_ = true;
    rviz::ViewController* vc = viewManager_->getCurrent();
    if (!vc) { applying_ = false; return; }

    const QString cls = obj.value(QStringLiteral("Class")).toString();
    if (!cls.isEmpty() && vc->getClassId() != cls) {
        viewManager_->setCurrentViewControllerType(cls);
        vc = viewManager_->getCurrent();
        if (!vc) { applying_ = false; return; }
    }

    auto setProp = [vc](const QString& key, const QVariant& val) {
        rviz::Property* p = vc->subProp(key);
        if (p) p->setValue(val);
    };

    if (obj.contains(QStringLiteral("Yaw"))) setProp(QStringLiteral("Yaw"), obj.value(QStringLiteral("Yaw")).toDouble());
    if (obj.contains(QStringLiteral("Pitch"))) setProp(QStringLiteral("Pitch"), obj.value(QStringLiteral("Pitch")).toDouble());
    if (obj.contains(QStringLiteral("Distance"))) setProp(QStringLiteral("Distance"), obj.value(QStringLiteral("Distance")).toDouble());
    if (obj.contains(QStringLiteral("Field of View"))) setProp(QStringLiteral("Field of View"), obj.value(QStringLiteral("Field of View")).toDouble());
    if (obj.contains(QStringLiteral("Target Frame"))) setProp(QStringLiteral("Target Frame"), obj.value(QStringLiteral("Target Frame")).toString());

    const QStringList vecKeys = { QStringLiteral("Focal Point"), QStringLiteral("Position") };
    for (const QString& key : vecKeys) {
        if (!obj.contains(key)) continue;
        rviz::VectorProperty* vp = dynamic_cast<rviz::VectorProperty*>(vc->subProp(key));
        if (vp) {
            const QJsonArray arr = obj.value(key).toArray();
            if (arr.size() >= 3) {
                vp->setVector(Ogre::Vector3(arr.at(0).toDouble(), arr.at(1).toDouble(), arr.at(2).toDouble()));
            }
        }
    }
    applying_ = false;
    ROS_INFO("[ViewReset] view applied from %s", qPrintable(path));
}

}  // namespace task_run_scripts

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_run_scripts::ViewResetPanel, rviz::Panel)
