#ifndef VULKANWINDOW_H
#define VULKANWINDOW_H
#include<QVulkanWindow>
#include"vulkan_window/vulkanrender.h"

class VulkanWindow : public QVulkanWindow
{
public:
    QVulkanWindowRenderer *createRenderer() override;
};

#endif