#ifndef VULKANWINDOW_H
#define VULKANWINDOW_H
#include<QVulkanWindow>
#include "OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh"
#include"OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh"
#include<QMouseEvent>

class VulkanModelRenderer;

typedef OpenMesh::TriMesh_ArrayKernelT<OpenMesh::DefaultTraits> TriMesh;
class VulkanWindow : public QVulkanWindow
{
public:
    VulkanWindow();
    QVulkanWindowRenderer *createRenderer() override;
protected:
    void mousePressEvent(QMouseEvent *)  override;
    void mouseReleaseEvent(QMouseEvent *) override;
    void mouseMoveEvent(QMouseEvent *) override;
    void wheelEvent(QWheelEvent *) override;
private:
    VulkanModelRenderer* renderer_;
public slots:
    void slotGetUpateMesh(const TriMesh& mesh);
    bool  isLeftDown_, isRightDown_ ;
    QPoint leftPos_,rightPos_;
};

#endif