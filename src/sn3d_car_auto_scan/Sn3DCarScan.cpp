#include "mainwindow.h"
#include <QApplication>
#include<iostream>
#include<ros/ros.h>
#include <QLoggingCategory>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QVulkanInstance inst;
    const bool dbg = qEnvironmentVariableIntValue("QT_VK_DEBUG");
    if (dbg) {
        QLoggingCategory::setFilterRules(QStringLiteral("qt.vulkan=true"));

#ifndef Q_OS_ANDROID
        inst.setLayers(QByteArrayList() << "VK_LAYER_LUNARG_standard_validation");
#else
        inst.setLayers(QByteArrayList()
                       << "VK_LAYER_GOOGLE_threading"
                       << "VK_LAYER_LUNARG_parameter_validation"
                       << "VK_LAYER_LUNARG_object_tracker"
                       << "VK_LAYER_LUNARG_core_validation"
                       << "VK_LAYER_LUNARG_image"
                       << "VK_LAYER_LUNARG_swapchain"
                       << "VK_LAYER_GOOGLE_unique_objects");
#endif
    }

    if (!inst.create())
        qFatal("Failed to create Vulkan instance: %d", inst.errorCode());

    VulkanWindow *vulkanWindow = new VulkanWindow();
    vulkanWindow->setVulkanInstance(&inst);

    MainWindow w(argc,argv,vulkanWindow);
    w.show();
    return a.exec();
}