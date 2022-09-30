#include <random>

#include <spdlog/spdlog.h>

#include "Labs/1-Drawing2D/tasks.h"

using VCX::Labs::Common::ImageRGB;

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
        float error[190][220][3] = {};
        std::memset(error, 0, sizeof(error));
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

                // std::printf("%f:%f:%f\n", error[x + 1][y + 1][0], error[x + 1][y + 1][1], error[x + 1][y + 1][2]);
            }
    }

    /******************* 2.Image Filtering *****************/
    void Blur(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
    }

    void Edge(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
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
            // set boundary for (width - 1, y), your code: g[y * width + width - 1] = ?
        }
        for (std::size_t x = 0; x < width; ++x) {
            // set boundary for (x, 0), your code: g[x] = ?
            // set boundary for (x, height - 1), your code: g[(height - 1) * width + x] = ?
        }

        // Jacobi iteration, solve Ag = b
        for (int iter = 0; iter < 8000; ++iter) {
            for (std::size_t y = 1; y < height - 1; ++y)
                for (std::size_t x = 1; x < width - 1; ++x) {
                    g[y * width + x] = (g[(y - 1) * width + x] + g[(y + 1) * width + x] + g[y * width + x - 1] + g[y * width + x + 1]);
                    g[y * width + x] = g[y * width + x] * glm::vec3(0.25);
                }
        }

        for (std::size_t y = 0; y < inputFront.GetSizeY(); ++y)
            for (std::size_t x = 0; x < inputFront.GetSizeX(); ++x) {
                glm::vec3 color = g[y * width + x] + inputFront.GetAt({ x, y });
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
    }

    /******************* 5. Triangle Drawing *****************/
    void DrawTriangleFilled(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1,
        glm::ivec2 const p2) {
        // your code here:
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
        // your code here:
        return glm::vec2 { 0, 0 };
    }
} // namespace VCX::Labs::Drawing2D