#ifndef VULKANMODELRENDER_H
#define VULKANMDEOLRENDER_H

#include"vulkanwindow.h"
#include "OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh"
#include"OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh"
#include<qfuture.h>
#include<QFutureWatcher>
#include<shader.h>
#include"orthoCamera.h"

typedef OpenMesh::TriMesh_ArrayKernelT<OpenMesh::DefaultTraits> TriMesh;

class VulkanModelRenderer : public QVulkanWindowRenderer
{
public:
    VulkanModelRenderer(VulkanWindow *w);

    void preInitResources() override;
    void initResources() override;
    void initSwapChainResources() override;
    void releaseSwapChainResources() override;
    void releaseResources() override;
    void startNextFrame() override;
 
    void setMesh(const TriMesh& mesh);
    void markViewProjDirty() { vpDirty_ = window_->concurrentFrameCount(); }
public:
    OrthoCamera cam_;
private:
    void createPipelines();
    void ensureBuffers();
    void getMatrices(QMatrix4x4 *mvp, QMatrix4x4 *model,  QVector3D *eyePos);
    void buildFrame();
    void buildDrawCallsForModel();

    VulkanWindow *window_;
    QVulkanDeviceFunctions *devFuncs_;

    TriMesh mesh_;
    bool isUpdata_;
    struct {
        VkDeviceSize vertUniSize;
        VkDeviceSize fragUniSize;
        VkDeviceSize uniMemStartOffset;
        Shader vs;
        Shader gs;
        Shader fs;
        VkDescriptorPool descPool = VK_NULL_HANDLE;
        VkDescriptorSetLayout descSetLayout = VK_NULL_HANDLE;
        VkDescriptorSet descSet;
        VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
        VkPipeline pipeline = VK_NULL_HANDLE;
    } meshMaterial_;

    VkDeviceMemory bufMem_ = VK_NULL_HANDLE;
    VkBuffer meshVertexBuf_ = VK_NULL_HANDLE;
    VkBuffer meshIndexBuf_ = VK_NULL_HANDLE;
    VkBuffer uniBuf_ = VK_NULL_HANDLE;

    VkPipelineCache pipelineCache_ = VK_NULL_HANDLE;
    QFuture<void> pipelinesFuture_;

    QVector3D lightPos_;

    int vpDirty_ = 0;

    QFutureWatcher<void> frameWatcher_;
    QMutex guiMutex_;
    bool framePending_;
};

#endif