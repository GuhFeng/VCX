
#include <algorithm>
#include <array>

#include "Labs/2-GeometryProcessing/CasePointCloud.h"
#include "Labs/2-GeometryProcessing/tasks.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::GeometryProcessing {
    CasePointCloud::CasePointCloud(
        Viewer & viewer, std::initializer_list<Assets::ExampleModel> && models):
        _models(models),
        _viewer(viewer) {
        _cameraManager.EnablePan       = false;
        _cameraManager.AutoRotateSpeed = 0.f;
        _options.LightDirection        = glm::vec3(
            glm::cos(glm::radians(_options.LightDirScalar)),
            -1.0f,
            glm::sin(glm::radians(_options.LightDirScalar)));
    }

    void CasePointCloud::OnSetupPropsUI() {
        if (ImGui::BeginCombo("Model", GetModelName(_modelIdx))) {
            for (std::size_t i = 0; i < _models.size(); ++i) {
                bool selected = i == _modelIdx;
                if (ImGui::Selectable(GetModelName(i), selected)) {
                    if (! selected) {
                        _modelIdx  = i;
                        _recompute = true;
                    }
                }
            }
            ImGui::EndCombo();
        }
        Common::ImGuiHelper::SaveImage(_viewer.GetTexture(), _viewer.GetSize(), true);
        ImGui::Spacing();
        _recompute |= ImGui::SliderInt("Radii", &_radii, 0, 3);
        if (_running) {
            static const std::string t = "Running.....";
            ImGui::Text(t.substr(0, 7 + (static_cast<int>(ImGui::GetTime() / 0.1f) % 6)).c_str());
        } else ImGui::NewLine();
        ImGui::Spacing();
        Viewer::SetupRenderOptionsUI(_options, _cameraManager);
    }

    Common::CaseRenderResult
        CasePointCloud::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (_recompute) {
            _recompute = false;
            _task.Emplace([&]() {
                Engine::SurfaceMesh emptyMesh;
                PointCloud(GetModelMesh(_modelIdx), emptyMesh, _radii);
                return emptyMesh;
            });
            _running = true;
        }
        if (_running && _task.HasValue()) {
            _running = false;
            _modelObject.ReplaceMesh(_task.Value());
        }
        return _viewer.Render(_options, _modelObject, _camera, _cameraManager, desiredSize);
    }

    void CasePointCloud::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }
} // namespace VCX::Labs::GeometryProcessing
