#include <algorithm>
#include <chrono>
#include <map>
#include <numeric>
#include <thread>

#include <pangolin/display/display.h>
#include <pangolin/display/opengl_render_state.h>

#include "visualizer.h"

void Visualizer::run()
{
    pangolin::CreateWindowAndBind("Main", 640, 480);
    //启动深度测试
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 320, 0.2, 100),
                                      //对应的是gluLookAt,摄像机位置,参考点位置,up vector(上向量)
                                      pangolin::ModelViewLookAt(20, 0, 30, 0, 0, 0, pangolin::AxisZ));

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    // setBounds 跟opengl的viewport 有关
    //看SimpleDisplay中边界的设置就知道
    pangolin::View& d_cam =
        pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f).SetHandler(&handler);

    while (!pangolin::ShouldQuit())
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        drawAxis();

        drawGTOdometry();

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }
}

void Visualizer::SetGroundTruth(const std::vector<IMU>& imus)
{
    std::unique_lock<std::mutex> lock(imu_data_mutex_);
    ground_truth_imus_ = imus;
}

void Visualizer::drawAxis()
{
    glLineWidth(3);
    glBegin(GL_LINES);
    glColor3f(1.0, 0, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(2, 0, 0);

    glColor3f(0, 1.0, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 2, 0);

    glColor3f(0, 0, 1);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 2);

    glEnd();
}

void Visualizer::drawGTOdometry()
{
    std::unique_lock<std::mutex> lock(imu_data_mutex_);
    for (size_t i = 1; i < ground_truth_imus_.size(); ++i)
    {
        IMU             imu_data_1 = ground_truth_imus_.at(i - 1);
        Vector3d        twb_1      = imu_data_1.twb;
        IMU             imu_data_2 = ground_truth_imus_.at(i);
        Vector3d        twb_2      = imu_data_2.twb;

        glLineWidth(2);
        glBegin(GL_LINES);
        glColor3f(1.0, 0, 0);
        glVertex3d(twb_1.x(), twb_1.y(), twb_1.z());
        glVertex3d(twb_2.x(), twb_2.y(), twb_2.z());
        glEnd();
    }

    if (!ground_truth_imus_.empty())
    {
        const auto imu_data = ground_truth_imus_.back();
        drawCamera(imu_data);
    }
}

void Visualizer::drawCamera(const IMU& imu_data)
{
    Eigen::Matrix3d Rwb = imu_data.Rwb;
    Eigen::Vector3d twb = imu_data.twb;
    /*
    cam.Rwb             = imu.Rwb * params.R_bc;            // cam frame in world frame
    cam.twb             = imu.twb + imu.Rwb * params.t_bc;  //  Tcw = Twb * Tbc ,  t = Rwb * tbc + twb
    */

    drawAxis();
    glPushMatrix();
    std::vector<GLdouble> Twb = {Rwb(0, 0), Rwb(1, 0), Rwb(2, 0), 0., Rwb(0, 1), Rwb(1, 1), Rwb(2, 1), 0.,
                                 Rwb(0, 2), Rwb(1, 2), Rwb(2, 2), 0., twb.x(),   twb.y(),   twb.z(),   1.};
    glMultMatrixd(Twb.data());

    const float w = 2;
    const float h = w * 0.75;
    const float z = w * 0.6;

    glLineWidth(2);

    glBegin(GL_LINES);

    drawAxis();

    glEnd();

    glPopMatrix();
}

