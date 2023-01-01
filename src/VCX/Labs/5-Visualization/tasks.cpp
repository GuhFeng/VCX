#include "Labs/5-Visualization/tasks.h"

#include <numbers>

using VCX::Labs::Common::ImageRGB;
namespace VCX::Labs::Visualization {
    struct CoordinateStates {
        // your code here
    };
    float data_i(Car c, int i) {
        switch (i) {
        case 0:
            return c.cylinders;
        case 1:
            return c.displacement;
        case 2:
            return c.weight;
        case 3:
            return c.horsepower;
        case 4:
            return c.acceleration;
        case 5:
            return c.mileage;
        case 6:
            return c.year;
        default:
            return 0;
        }
    }
    bool PaintParallelCoordinates(Common::ImageRGB & input, InteractProxy const & proxy, std::vector<Car> const & data, bool force) {
        // your code here
        SetBackGround(input, glm::vec4(1));
        float datarange[2][7] = {
            {2,  29, 1260,  27,  6,  5, 68},
            {9, 494, 5493, 249, 27, 51, 84}
        };
        const char * datarange_str[2][7] = {
            {"2",  "29", "1260",  "27",  "6",  "5", "68"},
            {"9", "494", "5493", "249", "27", "51", "84"}
        };
        const char * text[7] = { "cylinders", "displacement", "weight", "horsepower", "acceleration(0-60mph)", "mileage", "year" };
        for (int i = 0; i < 7; i++) {
            DrawFilledRect(input, glm::vec4(0.2, 0.2, 0.2, 0.3), glm::vec2(0.06 + i * 0.14, 0.1), glm::vec2(0.04, 0.8));
            DrawFilledRect(input, glm::vec4(0.5, 0.5, 0.5, 0.5), glm::vec2(0.075 + i * 0.14, 0.1), glm::vec2(0.01, 0.8));
            PrintText(input, glm::vec4(0, 0, 0, 1), glm::vec2(0.08 + i * 0.14, 0.06), 0.015, text[i]);
            PrintText(input, glm::vec4(0, 0, 0, 1), glm::vec2(0.08 + i * 0.14, 0.92), 0.015, datarange_str[0][i]);
            PrintText(input, glm::vec4(0, 0, 0, 1), glm::vec2(0.08 + i * 0.14, 0.08), 0.015, datarange_str[1][i]);
        }
        for (int i = 0; i < data.size(); i++) {
            Car now_car = data[i];
            for (int j = 0; j < 6; j++) {
                float y1, y2;
                y1 = 0.9 - 0.8 * (data_i(now_car, j) - datarange[0][j]) / (datarange[1][j] - datarange[0][j]);
                y2 = 0.9 - 0.8 * (data_i(now_car, j + 1) - datarange[0][j + 1]) / (datarange[1][j + 1] - datarange[0][j + 1]);
                DrawLine(input, glm::vec4(0.8 * i / data.size(), 0.1, 0.8 - 1.0 * i / data.size(), 0.2), glm::vec2(0.08 + j * 0.14, y1), glm::vec2(0.22 + j * 0.14, y2), 1.2);
            }
        }

        return true;
    }

    void LIC(ImageRGB & output, Common::ImageRGB const & noise, VectorField2D const & field, int const & step) {
        // your code here
    }
}; // namespace VCX::Labs::Visualization