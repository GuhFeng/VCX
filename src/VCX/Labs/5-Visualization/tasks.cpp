#include "Labs/5-Visualization/tasks.h"

#include <numbers>

using VCX::Labs::Common::ImageRGB;
namespace VCX::Labs::Visualization {
#define CAST_INT(a, i)   (*(((int *) &a) + i))
#define CAST_FLOAT(a, i) (*(((float *) &a) + i))
    struct CoordinateStates {
        // your code here
    };

    bool PaintParallelCoordinates(Common::ImageRGB & input, InteractProxy const & proxy, std::vector<Car> const & data, bool force) {
        // your code here
        SetBackGround(input, glm::vec4(1));
        float datarange[2][7] = {
            { 5, 2,  29,  27, 1260,  6, 68},
            {51, 9, 494, 249, 5493, 27, 84}
        };
        const char * datarange_str[2][7] = {
            { "5", "2",  "29",  "27", "1260",  "6", "68"},
            {"51", "9", "494", "249", "5493", "27", "84"}
        };
        const int    if_int[7] = { 0, 1, 0, 0, 0, 0, 1 };
        const char * text[7]   = { "mileage", "cylinders", "displacement", "horsepower", "weight", "acceleration(0-60mph)", "year" };
        for (int i = 0; i < 7; i++) {
            DrawFilledRect(input, glm::vec4(0, 0, 0, 0.05), glm::vec2(0.06 + i * 0.14, 0.1), glm::vec2(0.04, 0.8));
            DrawLine(input, glm::vec4(0, 0, 0, 1), glm::vec2(0.08 + i * 0.14, 0.1), glm::vec2(0.08 + i * 0.14, 0.9), 1);
            PrintText(input, glm::vec4(0, 0, 0, 1), glm::vec2(0.08 + i * 0.14, 0.03), 0.02, text[i]);
            PrintText(input, glm::vec4(0, 0, 0, 1), glm::vec2(0.08 + i * 0.14, 0.08), 0.02, datarange_str[1][i]);
            PrintText(input, glm::vec4(0, 0, 0, 1), glm::vec2(0.08 + i * 0.14, 0.92), 0.02, datarange_str[0][i]);
        }
        for (int i = 0; i < data.size(); i++) {
            Car now_car = data[i];
            for (int j = 0; j < 6; j++) {
                float y1, y2;
                if (if_int[j])
                    y1 = 0.1 + 0.8 * (CAST_INT(now_car, j) - datarange[0][j]) / (datarange[1][j] - datarange[0][j]);
                else
                    y1 = 0.1 + 0.8 * (CAST_FLOAT(now_car, j) - datarange[0][j]) / (datarange[1][j] - datarange[0][j]);
                if (if_int[j + 1])
                    y2 = 0.1 + 0.8 * (CAST_INT(now_car, j + 1) - datarange[0][j + 1]) / (datarange[1][j + 1] - datarange[0][j + 1]);
                else
                    y2 = 0.1 + 0.8 * (CAST_FLOAT(now_car, j + 1) - datarange[0][j + 1]) / (datarange[1][j + 1] - datarange[0][j + 1]);
                DrawLine(input, glm::vec4(0.8 * i / data.size(), 0.1, 0.8 - 1.0 * i / data.size(), 0.2), glm::vec2(0.08 + j * 0.14, y1), glm::vec2(0.22 + j * 0.14, y2), 1.8);
            }
        }

        return true;
    }

    void LIC(ImageRGB & output, Common::ImageRGB const & noise, VectorField2D const & field, int const & step) {
        // your code here
    }
}; // namespace VCX::Labs::Visualization