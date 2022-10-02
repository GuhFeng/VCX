#include <random>

#include <spdlog/spdlog.h>

#include "Labs/1-Drawing2D/tasks.h"

#include <math.h>

using VCX::Labs::Common::ImageRGB;

float *** alloc_buff(int w, int h) {
    float *** p = new float **[w];
    for (int i = 0; i < w; i++) {
        p[i] = new float *[h];
        for (int j = 0; j < h; j++) {
            p[i][j]    = new float[3];
            p[i][j][0] = p[i][j][1] = p[i][j][2] = 0;
        }
    }
    return p;
}

void delete_buff(float *** p, int w, int h) {
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++)
            delete p[i][j];
        delete p[i];
    }
    delete p;
}

namespace VCX::Labs::Drawing2D {
    /******************* 1.Image Dithering *****************/
    void DitheringThreshold(
        ImageRGB &       output,
        ImageRGB const & input) {
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input[{ x, y }];
                output.SetAt({ x, y }, {
                                           color.r > 0.5 ? 1 : 0,
                                           color.g > 0.5 ? 1 : 0,
                                           color.b > 0.5 ? 1 : 0,
                                       });
            }
    }

    void DitheringRandomUniform(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        srand(time(NULL));

        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                float     noise = rand() % (10000) / (float) (10000);
                glm::vec3 color = input[{ x, y }];
                output.SetAt({ x, y }, {
                                           color.r + noise > 1 ? 1 : 0,
                                           color.g + noise > 1 ? 1 : 0,
                                           color.b + noise > 1 ? 1 : 0,
                                       });
            }
    }

    void DitheringRandomBlueNoise(
        ImageRGB &       output,
        ImageRGB const & input,
        ImageRGB const & noise) {
        // your code here:
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color       = input[{ x, y }];
                glm::vec3 noise_color = noise[{ x, y }];
                output.SetAt({ x, y }, {
                                           color.r + noise_color.r > 1 ? 1 : 0,
                                           color.g + noise_color.g > 1 ? 1 : 0,
                                           color.b + noise_color.b > 1 ? 1 : 0,
                                       });
            }
    }

    void DitheringOrdered(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color            = input[{ x, y }];
                glm::vec3 color_buff[3][3] = {};
                switch (int(10 * color.r - 0.0000001)) {
                case 9:
                    color_buff[0][1].r = 1;
                case 8:
                    color_buff[2][2].r = 1;
                case 7:
                    color_buff[0][0].r = 1;
                case 6:
                    color_buff[2][0].r = 1;
                case 5:
                    color_buff[0][2].r = 1;
                case 4:
                    color_buff[1][2].r = 1;
                case 3:
                    color_buff[2][1].r = 1;
                case 2:
                    color_buff[1][0].r = 1;
                case 1:
                    color_buff[1][1].r = 1;
                default:
                    break;
                }
                switch (int(10 * color.g - 0.0000001)) {
                case 9:
                    color_buff[0][1].g = 1;
                case 8:
                    color_buff[2][2].g = 1;
                case 7:
                    color_buff[0][0].g = 1;
                case 6:
                    color_buff[2][0].g = 1;
                case 5:
                    color_buff[0][2].g = 1;
                case 4:
                    color_buff[1][2].g = 1;
                case 3:
                    color_buff[2][1].g = 1;
                case 2:
                    color_buff[1][0].g = 1;
                case 1:
                    color_buff[1][1].g = 1;
                default:
                    break;
                }
                switch (int(10 * color.b - 0.0000001)) {
                case 9:
                    color_buff[0][1].b = 1;
                case 8:
                    color_buff[2][2].b = 1;
                case 7:
                    color_buff[0][0].b = 1;
                case 6:
                    color_buff[2][0].b = 1;
                case 5:
                    color_buff[0][2].b = 1;
                case 4:
                    color_buff[1][2].b = 1;
                case 3:
                    color_buff[2][1].b = 1;
                case 2:
                    color_buff[1][0].b = 1;
                case 1:
                    color_buff[1][1].b = 1;
                default:
                    break;
                }
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        output.SetAt({ 3 * x + i, 3 * y + j }, { color_buff[i][j].r, color_buff[i][j].g, color_buff[i][j].b });
                    }
                }
            }
    }

    void DitheringErrorDiffuse(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        int       w = input.GetSizeX(), h = input.GetSizeY();
        float *** error = alloc_buff(w + 5, h + 5);
        for (std::size_t y = 0; y < input.GetSizeY(); ++y)
            for (std::size_t x = 0; x < input.GetSizeX(); ++x) {
                glm::vec3 color = input[{ x, y }];
                color.r         = color.r - error[x + 1][y + 1][0];
                color.g         = color.g - error[x + 1][y + 1][1];
                color.b         = color.b - error[x + 1][y + 1][2];
                output.SetAt({ x, y }, {
                                           color.r > 0.5 ? 1 : 0,
                                           color.g > 0.5 ? 1 : 0,
                                           color.b > 0.5 ? 1 : 0,
                                       });
                float tmp_err[3];
                tmp_err[0] = (color.r > 0.5 ? 1 : 0) - color.r;
                tmp_err[1] = (color.g > 0.5 ? 1 : 0) - color.g;
                tmp_err[2] = (color.b > 0.5 ? 1 : 0) - color.b;
                for (int i = 0; i < 3; i++) {
                    error[x + 2][y + 1][i] += tmp_err[i] * 7 / 16;
                    error[x + 1][y + 2][i] += tmp_err[i] * 5 / 16;
                    error[x + 2][y + 2][i] += tmp_err[i] * 1 / 16;
                    error[x][y + 2][i] += tmp_err[i] * 3 / 16;
                }
            }
        delete_buff(error, w + 5, h + 5);
    }

    /******************* 2.Image Filtering *****************/
    void Blur(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        int       w = input.GetSizeX(), h = input.GetSizeY();
        float *** Color_Buf_Img = alloc_buff(w + 5, h + 5);
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color                = input[{ x, y }];
                Color_Buf_Img[x + 1][y + 1][0] = color.r;
                Color_Buf_Img[x + 1][y + 1][1] = color.g;
                Color_Buf_Img[x + 1][y + 1][2] = color.b;
            }
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                float color_tmp[3] = {};
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++)
                        for (int k = 0; k < 3; k++) {
                            color_tmp[i] += Color_Buf_Img[x + j][y + k][i];
                        }
                    color_tmp[i] /= 9;
                }
                output.SetAt({ x, y }, { color_tmp[0], color_tmp[1], color_tmp[2] });
            }
        delete_buff(Color_Buf_Img, w + 5, h + 5);
    }

    void Edge(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        int       w = input.GetSizeX(), h = input.GetSizeY();
        float *** Color_Buf_Img = alloc_buff(w + 5, h + 5);
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color                = input[{ x, y }];
                Color_Buf_Img[x + 1][y + 1][0] = color.r;
                Color_Buf_Img[x + 1][y + 1][1] = color.g;
                Color_Buf_Img[x + 1][y + 1][2] = color.b;
            }
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                float color_tmp[3][2] = {};
                float tmp[3]          = {};
                for (int i = 0; i < 3; i++) {
                    color_tmp[i][0] = Color_Buf_Img[x + 2][y + 2][i] - Color_Buf_Img[x][y + 2][i] + 2 * (Color_Buf_Img[x + 2][y + 1][i] - Color_Buf_Img[x][y + 1][i]) + Color_Buf_Img[x + 2][y][i] - Color_Buf_Img[x][y][i];
                    color_tmp[i][1] = Color_Buf_Img[x + 2][y + 2][i] - Color_Buf_Img[x + 2][y][i] + 2 * (Color_Buf_Img[x + 1][y + 2][i] - Color_Buf_Img[x + 1][y][i]) + Color_Buf_Img[x][y + 2][i] - Color_Buf_Img[x][y][i];
                    tmp[i]          = sqrt(color_tmp[i][0] * color_tmp[i][0] + color_tmp[i][1] * color_tmp[i][1]);
                }
                output.SetAt({ x, y }, { tmp[0], tmp[1], tmp[2] });
            }
        delete_buff(Color_Buf_Img, w + 5, h + 5);
    }

    /******************* 3. Image Inpainting *****************/
    void Inpainting(
        ImageRGB &         output,
        ImageRGB const &   inputBack,
        ImageRGB const &   inputFront,
        const glm::ivec2 & offset) {
        output             = inputBack;
        size_t      width  = inputFront.GetSizeX();
        size_t      height = inputFront.GetSizeY();
        glm::vec3 * g      = new glm::vec3[width * height];
        memset(g, 0, sizeof(glm::vec3) * width * height);
        // set boundary condition
        for (std::size_t y = 0; y < height; ++y) {
            // set boundary for (0, y), your code: g[y * width] = ?
            glm::vec3 color1 = inputBack[{ (std::size_t)(offset[0]), (std::size_t)(offset[1] + y) }];
            g[y * width]     = color1;
            glm::vec3 color2 = inputBack[{ (std::size_t)(offset[0] + width - 1), (std::size_t)(offset[1] + y) }];
            // set boundary for (width - 1, y), your code: g[y * width + width - 1] = ?
            g[y * width + width - 1] = color2;
        }
        for (std::size_t x = 0; x < width; ++x) {
            // set boundary for (x, 0), your code: g[x] = ?
            glm::vec3 color1 = inputBack[{ (std::size_t)(offset[0] + x), (std::size_t)(offset[1]) }];
            g[x]             = color1;
            // set boundary for (x, height - 1), your code: g[(height - 1) * width + x] = ?
            glm::vec3 color2            = inputBack[{ (std::size_t)(offset[0] + x), (std::size_t)(offset[1] + height - 1) }];
            g[(height - 1) * width + x] = color2;
        }

        // Jacobi iteration, solve Ag = b
        for (int iter = 0; iter < 8000; ++iter) {
            for (std::size_t y = 1; y < height - 1; ++y)
                for (std::size_t x = 1; x < width - 1; ++x) {
                    glm::vec3 div_front = inputFront[{ x + 1, y }] + inputFront[{ x - 1, y }] + inputFront[{ x, y + 1 }] + inputFront[{ x, y - 1 }];
                    div_front           = div_front - inputFront[{ x, y }] * glm::vec3(4);
                    g[y * width + x]    = (g[(y - 1) * width + x] + g[(y + 1) * width + x] + g[y * width + x - 1] + g[y * width + x + 1]);
                    g[y * width + x]    = (g[y * width + x] - div_front) * glm::vec3(0.25);
                }
        }

        for (std::size_t y = 0; y < inputFront.GetSizeY(); ++y)
            for (std::size_t x = 0; x < inputFront.GetSizeX(); ++x) {
                glm::vec3 color = g[y * width + x];
                output.SetAt({ x + offset.x, y + offset.y }, color);
            }
        delete[] g;
    }

    /******************* 4. Line Drawing *****************/
    void DrawLine(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1) {
        // your code here:
        int        x, y, sg, dx, dy, dydx, F;
        glm::ivec2 ponits[2];
        switch (abs(p0[0] - p1[0]) >= abs(p0[1] - p1[1])) {
        case true:
            if (p0[0] > p1[0]) {
                ponits[0] = p1;
                ponits[1] = p0;
            } else {
                ponits[1] = p1;
                ponits[0] = p0;
            }
            x, y = ponits[0][1];
            if (ponits[1][1] == ponits[0][1])
                sg = 0;
            else
                sg = (ponits[1][1] - ponits[0][1]) / abs(ponits[1][1] - ponits[0][1]);
            dx = 2 * (ponits[1][0] - ponits[0][0]), dy = 2 * abs(ponits[1][1] - ponits[0][1]);
            dydx = dy - dx, F = dy - dx / 2;
            for (x = ponits[0][0]; x <= ponits[1][0]; x++) {
                canvas.SetAt({ (std::size_t) x, (std::size_t) y }, color);
                if (F < 0) F += dy;
                else {
                    y += sg;
                    F += dydx;
                }
            }
            break;

        default:
            if (p0[1] > p1[1]) {
                ponits[0] = p1;
                ponits[1] = p0;
            } else {
                ponits[1] = p1;
                ponits[0] = p0;
            }
            x = ponits[0][0], y;
            if (ponits[1][0] == ponits[0][0])
                sg = 0;
            else
                sg = (ponits[1][0] - ponits[0][0]) / abs(ponits[1][0] - ponits[0][0]);
            dx = 2 * abs(ponits[1][0] - ponits[0][0]), dy = 2 * (ponits[1][1] - ponits[0][1]);
            dydx = dx - dy, F = dx - dy / 2;
            for (y = ponits[0][1]; y <= ponits[1][1]; y++) {
                canvas.SetAt({ (std::size_t) x, (std::size_t) y }, color);
                if (F < 0) F += dx;
                else {
                    x += sg;
                    F += dydx;
                }
            }
            break;
        }
    }

    /******************* 5. Triangle Drawing *****************/
    void DrawTriangleFilled(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1,
        glm::ivec2 const p2) {
        // your code here:
        glm::ivec2 P[3];
        if (p0[1] >= p1[1] && p0[1] >= p2[1]) {
            P[0] = p0;
            if (p1[1] >= p2[1]) {
                P[1] = p1;
                P[2] = p2;
            } else {
                P[2] = p1;
                P[1] = p2;
            }
        } else if (p1[1] >= p0[1] && p1[1] >= p2[1]) {
            P[0] = p1;
            if (p0[1] >= p2[1]) {
                P[1] = p0;
                P[2] = p2;
            } else {
                P[2] = p0;
                P[1] = p2;
            }
        } else if (p2[1] >= p0[1] && p2[1] >= p1[1]) {
            P[0] = p2;
            if (p0[1] >= p1[1]) {
                P[1] = p0;
                P[2] = p1;
            } else {
                P[2] = p0;
                P[1] = p1;
            }
        }
        for (int y = P[1][1]; y < P[0][1]; y++) {
            int x1 = round(1.0 * ((y - P[1][1]) * (P[0][0] - P[1][0])) / (P[0][1] - P[1][1])) + P[1][0];
            int x2 = round(1.0 * (y - P[2][1]) * (P[0][0] - P[2][0]) / (P[0][1] - P[2][1])) + P[2][0];
            if (x1 > x2) {
                int tmp = x1;
                x1      = x2;
                x2      = tmp;
            }
            for (int x = x1; x <= x2; x++) {
                canvas.SetAt({ (std::size_t) x, (std::size_t) y }, color);
            }
        }
        for (int y = P[2][1]; y < P[1][1]; y++) {
            int x1 = round(1.0 * (y - P[2][1]) * (P[0][0] - P[2][0]) / (P[0][1] - P[2][1])) + P[2][0];
            int x2 = round(1.0 * (y - P[2][1]) * (P[1][0] - P[2][0]) / (P[1][1] - P[2][1])) + P[2][0];
            if (x1 > x2) {
                int tmp = x1;
                x1      = x2;
                x2      = tmp;
            }
            for (int x = x1; x <= x2; x++) {
                canvas.SetAt({ (std::size_t) x, (std::size_t) y }, color);
            }
        }
        for (int i = 0; i < 3; i++) {
            canvas.SetAt({ (std::size_t) P[i][0], (std::size_t) P[i][1] }, color);
        }
        if (P[0][1] == P[1][1]) {
            int x1 = P[0][0];
            int x2 = P[1][0];
            int y  = P[0][1];
            if (x1 > x2) {
                int tmp = x1;
                x1      = x2;
                x2      = tmp;
            }
            for (int x = x1; x <= x2; x++) {
                canvas.SetAt({ (std::size_t) x, (std::size_t) y }, color);
            }
        }
        if (P[2][1] == P[1][1]) {
            int x1 = P[2][0];
            int x2 = P[1][0];
            int y  = P[1][1];
            if (x1 > x2) {
                int tmp = x1;
                x1      = x2;
                x2      = tmp;
            }
            for (int x = x1; x <= x2; x++) {
                canvas.SetAt({ (std::size_t) x, (std::size_t) y }, color);
            }
        }
    }

    /******************* 6. Image Supersampling *****************/
    void Supersample(
        ImageRGB &       output,
        ImageRGB const & input,
        int              rate) {
        // your code here:
    }

    /******************* 7. Bezier Curve *****************/
    glm::vec2 CalculateBezierPoint(
        std::span<glm::vec2> points,
        float const          t) {
        if (points.size() == 4) {
            for (int i = 0; i < points.size(); i++) {
                printf("%d  %d:%d\n", points.size(), points[i].r, points[i][1]);
            }
        }
        // your code here:
        std::vector<glm::vec2> p;
        for (int i = 0; i < points.size() - 1; i++) {
            p.push_back(points[i]);
            p[i][0] = (1 - t) * points[i][0] + t * points[i][0];
            p[i][1] = (1 - t) * points[i][1] + t * points[i][1];
        }
        if (points.size() == 2) {
            return p[0];
        } else {
            std::span<glm::vec2> tmp(p);
            return CalculateBezierPoint(tmp, t);
        }
    }
} // namespace VCX::Labs::Drawing2D