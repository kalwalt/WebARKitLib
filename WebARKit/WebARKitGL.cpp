#include <WebARKitGL.h>

namespace webarkit {

void arglCameraViewRHf(float para[3][4], float m_modelview[16], const float scale) {
    m_modelview[0 + 0 * 4] = para[0][0]; // R1C1
    m_modelview[0 + 1 * 4] = para[0][1]; // R1C2
    m_modelview[0 + 2 * 4] = para[0][2];
    m_modelview[0 + 3 * 4] = para[0][3];
    m_modelview[1 + 0 * 4] = -para[1][0]; // R2
    m_modelview[1 + 1 * 4] = -para[1][1];
    m_modelview[1 + 2 * 4] = -para[1][2];
    m_modelview[1 + 3 * 4] = -para[1][3];
    m_modelview[2 + 0 * 4] = -para[2][0]; // R3
    m_modelview[2 + 1 * 4] = -para[2][1];
    m_modelview[2 + 2 * 4] = -para[2][2];
    m_modelview[2 + 3 * 4] = -para[2][3];
    m_modelview[3 + 0 * 4] = 0.0f;
    m_modelview[3 + 1 * 4] = 0.0f;
    m_modelview[3 + 2 * 4] = 0.0f;
    m_modelview[3 + 3 * 4] = 1.0f;
    if (scale != 0.0f) {
        m_modelview[12] *= scale;
        m_modelview[13] *= scale;
        m_modelview[14] *= scale;
    }
}

void arglCameraViewRHf(cv::Mat para, std::array<double, 16>& m_modelview, const double scale) {
    m_modelview[0 + 0 * 4] = para.at<double>(0, 0); // R1C1
    m_modelview[0 + 1 * 4] = para.at<double>(0, 1); // R1C2
    m_modelview[0 + 2 * 4] = para.at<double>(0, 2);
    m_modelview[0 + 3 * 4] = para.at<double>(0, 3);
    m_modelview[1 + 0 * 4] = -para.at<double>(1, 0); // R2
    m_modelview[1 + 1 * 4] = -para.at<double>(1, 1);
    m_modelview[1 + 2 * 4] = -para.at<double>(1, 2);
    m_modelview[1 + 3 * 4] = -para.at<double>(1, 3);
    m_modelview[2 + 0 * 4] = -para.at<double>(2, 0); // R3
    m_modelview[2 + 1 * 4] = -para.at<double>(2, 1);
    m_modelview[2 + 2 * 4] = -para.at<double>(2, 2);
    m_modelview[2 + 3 * 4] = -para.at<double>(2, 3);
    m_modelview[3 + 0 * 4] = 0.0f;
    m_modelview[3 + 1 * 4] = 0.0f;
    m_modelview[3 + 2 * 4] = 0.0f;
    m_modelview[3 + 3 * 4] = 1.0f;
    if (scale != 0.0f) {
        m_modelview[12] *= scale;
        m_modelview[13] *= scale;
        m_modelview[14] *= scale;
    }
}

void cameraProjectionMatrix(const std::array<double, 9>& calibration, double nearPlane, double farPlane, int screenWidth, int screenHeight, std::array<double, 16>& projectionMatrix)
{
  // Camera parameters
  double f_x = calibration.at(0); // Focal length in x axis
  double f_y = calibration.at(4); // Focal length in y axis (usually the same?)
  double c_x = calibration.at(2); // Camera primary point x
  double c_y = calibration.at(5); // Camera primary point y

  projectionMatrix[0] = -2.0f * f_x / screenWidth;
  projectionMatrix[1] = 0.0f;
  projectionMatrix[2] = 0.0f;
  projectionMatrix[3] = 0.0f;

  projectionMatrix[4] = 0.0f;
  projectionMatrix[5] = 2.0f * f_y / screenHeight;
  projectionMatrix[6] = 0.0f;
  projectionMatrix[7] = 0.0f;

  projectionMatrix[8] = 2.0f * c_x / screenWidth - 1.0f;
  projectionMatrix[9] = 2.0f * c_y / screenHeight - 1.0f;    
  projectionMatrix[10] = -( farPlane + nearPlane) / ( farPlane - nearPlane );
  projectionMatrix[11] = -1.0f;

  projectionMatrix[12] = 0.0f;
  projectionMatrix[13] = 0.0f;
  projectionMatrix[14] = -2.0f * farPlane * nearPlane / ( farPlane - nearPlane );        
  projectionMatrix[15] = 0.0f;
}
} // namespace webarkit