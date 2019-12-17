#include "vulkanwindow.h"
#include"vulkanrender.h"
#include"vulkanmodelrender.h"

QVulkanWindowRenderer *VulkanWindow::createRenderer()
{
    renderer_=  new VulkanModelRenderer(this);
    return renderer_;
}

VulkanWindow::VulkanWindow():
    QVulkanWindow(),isLeftDown_(false),isRightDown_(false)
{}

void VulkanWindow::slotGetUpateMesh(const TriMesh& mesh)
{
    renderer_->setMesh(mesh);
}

void VulkanWindow::mousePressEvent(QMouseEvent *event) 
{
    if(event->button()==Qt::LeftButton)
    {
        leftPos_ = event->pos();
        isLeftDown_ = true;
    }
       
    if(event->button() == Qt::RightButton)
    {
        rightPos_  = event -> pos(); 
        isRightDown_ = true;
    } 
}
void VulkanWindow::mouseReleaseEvent(QMouseEvent *event)
{
    if(event->button() == Qt::LeftButton)
        isLeftDown_ = false;
    if(event->button() == Qt::RightButton)
        isRightDown_ = false;
}

void VulkanWindow::mouseMoveEvent(QMouseEvent *event) 
{
    QPoint mousepos =  event->pos();
    int movex,movey;
    if(isLeftDown_)
    {
        movex = mousepos.rx() - leftPos_.rx();
        movey = leftPos_.ry() - mousepos.ry();
        leftPos_ = mousepos;

        if(movex > 3)
        {
            renderer_->cam_.rotata_view(3,1);
        }  
        else if(movex < -3)
        {
            renderer_->cam_.rotata_view(-3,1);
        }

        if(movey > 3)
        {
            renderer_->cam_.rotata_view(3,0);
        }
        else if(movey < -3)
        {
            renderer_->cam_.rotata_view(-3,0);
        }
        renderer_->markViewProjDirty();
    }
    else if( isRightDown_)
    {
        movex = mousepos.rx() - rightPos_.rx();
        movey = rightPos_.ry() - mousepos.ry();
        rightPos_ =  mousepos;
        renderer_->cam_.move_view(0.001*movex,0.001*movey);
        renderer_->markViewProjDirty();
    }
}

void VulkanWindow::wheelEvent(QWheelEvent *event) 
{
    if(event->delta()>2)
    {
        renderer_->cam_.zoom(0.9);
        renderer_->markViewProjDirty();
    }
    else
    {
        renderer_->cam_.zoom(1.1);
        renderer_->markViewProjDirty();
    }   
}