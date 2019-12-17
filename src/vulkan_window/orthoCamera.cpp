#include"orthoCamera.h"



OrthoCamera::OrthoCamera()
{
    init();
}

OrthoCamera::~OrthoCamera()
{

}

void OrthoCamera::init()
{
    eye_ = QVector3D(5.0f,2.5f,1000.0f);
    ref_   = QVector3D(5.0f,2.5f,0.0f);
    vecUp_ = QVector3D(0.0f,-1.0f,0.0f);

    far_ = 10000.0f;
    near_ = 0;
    width_ = 10;
    height_ = 10;

    screen_[0] = screen_[1] = 10;
}


void OrthoCamera::projection(const QMatrix4x4& poj)
{
    float left           = -width_/2.0f;
    float right        = width_/2.0f;
    float  bottom = -height_/2.0f;
    double top     = height_/2.0f;

    qpoj_ = poj;
    qpoj_.ortho(left,right,bottom, top,near_,far_);

    qview_.setToIdentity();
    qview_.lookAt(eye_,ref_,vecUp_);
}

void OrthoCamera::set_sreen(int x,int y)
{
    if(y==0) y =1;
    float ratio = float(x)/float(y);
    width_ *= float(x) / screen_[0];
    height_ *= float(y) /screen_[1];
    screen_[0] = x;
    screen_[1] = y;
}

void OrthoCamera::set_eye(const QVector3D& eye)
{
    eye_ = eye;
}

void OrthoCamera::set_ref(const QVector3D& ref)
{
    ref_ = ref;
}

void OrthoCamera::set_vecUp(const QVector3D& vecUp)
{
    vecUp_ = vecUp;
}

void OrthoCamera::set_view_rect(const QVector2D& rect)
{
    width_ = rect[0];
    height_ = rect[1];
    float aspect = screen_[0]/screen_[1];
    width_ = height_ * aspect;
}

void OrthoCamera::get_view_rect(QVector2D& rect)
{
    rect.setX(width_) ;
    rect.setY(height_);
}

void OrthoCamera::zoom(float scale)
{
    if(scale > 0)
    {
        width_ *= scale;
        height_ *=scale;
        scale_ = scale;
        if(scale_ >1)
            scale_ = 1;
    }
}

void OrthoCamera::update_upVec()
{
    QVector3D vec = ref_ - eye_;
    QVector3D zVec(0.0f,0.0f,1.0f);
    QVector3D vec0;

    vec.normalize();
    vec0 = QVector3D::crossProduct(vec,zVec);
    vecUp_ = QVector3D::crossProduct(vec0,vec);
}

void OrthoCamera::move_view(float  dpx, float dpy)
{
    QVector3D vec;
    QVector3D xUp, yUp;

    vec = ref_ - eye_;
    vec.normalize();
    xUp = QVector3D::crossProduct(vec,vecUp_);
    yUp = QVector3D::crossProduct(xUp, vec);

    eye_ -= xUp*width_*dpx + yUp*height_*dpy;
    ref_ -= xUp*width_*dpx + yUp*height_*dpy;
}

void OrthoCamera::rotata_view(float dRot, short snXyz)
{
    QVector3D rotVec;
    QVector3D origVec;
    QMatrix4x4 rotMat;

    origVec = eye_ - ref_;

    switch (snXyz)
    {
    case 0:
        rotVec = QVector3D::crossProduct(vecUp_ , origVec);
        rotMat.rotate(dRot, rotVec);

        vecUp_ = rotMat * vecUp_;
        origVec = rotMat * origVec;
        eye_ = ref_ + origVec; 
        break;

    case 1:
        rotMat.rotate(dRot,vecUp_);
        origVec = rotMat*origVec;
        eye_ = ref_ + origVec;   
        break;

    case 2:
        rotMat.rotate(dRot,origVec);
        vecUp_ = rotMat * vecUp_;
        break;
    default:
        break;
    }
}

void OrthoCamera::get_eye(QVector3D& eye)
{
    eye = eye_;
}

void OrthoCamera::get_ref(QVector3D& ref)
{
    ref = ref_;
}

void OrthoCamera::get_vecUp(QVector3D& vecUp)
{
    vecUp = vecUp_;
}