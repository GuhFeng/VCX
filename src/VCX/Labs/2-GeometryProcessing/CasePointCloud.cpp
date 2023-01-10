
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
        static char pc_path[128] = "assets/point_cloud/car6.pcd";
        ImGui::InputTextWithHint("", "", pc_path, IM_ARRAYSIZE(pc_path));
        bool b_pc = ImGui::Button("Load Point Cloud File");
        _path     = pc_path;
        ImGui::Spacing();
        if (b_pc) { _recompute |= b_pc; }

        Viewer::SetupRenderOptionsUI(_options, _cameraManager);
    }

    Common::CaseRenderResult
        CasePointCloud::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (_recompute) {
            _recompute = false;
            _task.Emplace([&]() {
                Engine::SurfaceMesh emptyMesh;
                PointCloud(GetModelMesh(_modelIdx), emptyMesh, _path);
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
