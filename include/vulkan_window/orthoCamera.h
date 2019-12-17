#ifndef ORTHOCAMERA_H
#define ORTHOCAMERA_H
#include<QVector3D>
#include<QVector2D>
#include<QMatrix4x4>
#include<QRect>


#define CAD_ZERO                        1.0E-6
#define IS_ZERO                             (fabs(x)<=CAD_ZERO)


enum view_type
{
    VIEW_FRONT                    = 0,
    VIEW_BACK                       = 1,
    VIEW_TOP                          = 2,
    VIEW_BOTTOM                = 3,
    VIEW_RIGHT                     = 4,
    VIEW_LEFT                        = 5,
    VIEW_SW_ISOMETRIC  = 6,
    VIEW_SE_ISOMETRIC   = 7,
    VIEW_NE_ISOMETRIC  = 8,
    VIEW_NW_ISOMETRIC = 9,

    ZOOM_ALL                        = 10,
    ZOOM_IN                           = 11,
    ZOOM_OUT                      = 12
};


class OrthoCamera
{
public:
    QVector3D    eye_;
    QVector3D    ref_;
    QVector3D    vecUp_;

    float             far_, near_;
    float             width_, height_;
    
    float             scale_;

    float             screen_[2];
public:
    OrthoCamera();
    ~OrthoCamera();

    void init();

    void projection(const QMatrix4x4& poj);

    void zoom(float scale);

    void zoom_all(const QVector3D& min_pos, const QVector3D& max_pos );

    void move_view(float dpx, float dpy);

    void set_view_type(int type);

    void set_sreen(int x,int y);

    void set_eye(const QVector3D& eye);
    void set_ref(const QVector3D& ref);
    void set_vecUp(const QVector3D& vecUp);

    void get_eye(QVector3D& eye);
    void get_ref(QVector3D& ref);
    void get_vecUp(QVector3D& vecUp);

    void set_view_rect(const QVector2D& rect);
    void get_view_rect(QVector2D& rect);
    void rotata_view(float dRot, short snXyz);

    QMatrix4x4 qview_, qpoj_;
protected:
    void update_upVec();
};

#endif