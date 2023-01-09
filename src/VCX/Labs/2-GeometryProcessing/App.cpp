#include "Labs/2-GeometryProcessing/App.h"
#include "Engine/loader.h"

namespace VCX::Labs::GeometryProcessing {
    using namespace Assets;

    App::App():
        _caseSubdivision(
            _viewer,
            { ExampleModel::Cube,
              ExampleModel::Block,
              ExampleModel::Dinosaur,
              ExampleModel::Fandisk,
              ExampleModel::Rocker,
              ExampleModel::Arma }),
        _caseParameterization(_viewer), _caseSimplification(
                                            _viewer,
                                            { ExampleModel::Sphere,
                                              ExampleModel::Block,
                                              ExampleModel::Dinosaur,
                                              ExampleModel::Fandisk,
                                              ExampleModel::Rocker,
                                              ExampleModel::Arma }),
        _caseSmoothing(_viewer, { ExampleModel::Block, ExampleModel::Dinosaur }),
        _caseMarchingCubes(_viewer), _casePointCloud(
                                         _viewer,
                                         {
                                             ExampleModel::Point_Cloud,
                                         }),
        _ui(Labs::Common::UIOptions {}) {}

    void App::OnFrame() { _ui.Setup(_cases, _caseId); }
}
