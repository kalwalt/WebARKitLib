#include <iostream>
static cv::Mat im_gray(uchar data[], size_t cols, size_t rows) {
    std::cout << data << std::endl;
    uint32_t idx;
    uchar gray[rows][cols];
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            idx = (i * cols * 4) + j * 4;

            // rgba to rgb
            uchar r = data[idx];
            uchar g = data[idx + 1];
            uchar b = data[idx + 2];
            // uchar a = data[idx + 3];

            // turn frame image to gray scale
            gray[i][j] = (0.30 * r) + (0.59 * g) + (0.11 * b);
            std::cout << gray[i][j] << std::endl;
        }
    }

    return cv::Mat(rows, cols, CV_8UC1, gray);
}