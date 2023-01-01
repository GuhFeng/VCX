#include "Labs/4-Animation/tasks.h"
#include "CustomFunc.inl"
#include "IKSystem.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <spdlog/spdlog.h>
typedef Eigen::SparseMatrix<float> SPM;
typedef Eigen::Triplet<float>      TRP;
typedef Eigen::VectorXf            VECXF;
typedef Eigen::SimplicialLLT<SPM>  LLT;
namespace VCX::Labs::Animation {

    void partial_g(MassSpringSystem & system, std::vector<glm::vec3> & x, std::vector<glm::vec3> & y, std::vector<glm::vec3> & b, float h) {
        for (int i = 0; i < x.size(); i++) {
            b[i] = system.Mass * (x[i] - y[i]) / (h * h);
        }
        for (auto const spring : system.Springs) {
            auto const      p0  = spring.AdjIdx.first;
            auto const      p1  = spring.AdjIdx.second;
            glm::vec3 const x01 = x[p1] - x[p0];
            glm::vec3 const v01 = ((x[p1] - system.Positions[p1]) - (x[p0] - system.Positions[p0])) / h;
            glm::vec3 const e01 = glm::normalize(x01);
            glm::vec3       f   = (system.Stiffness * (glm::length(x01) - spring.RestLength) + system.Damping * glm::dot(v01, e01)) * e01;
            b[p0] -= f;
            b[p1] += f;
        }
    }
    void partial2_g(MassSpringSystem & system, std::vector<glm::vec3> & x, std::vector<glm::vec3> & y, std::vector<TRP> & A, float h) {
        A.clear();
        std::vector<glm::mat3x3> diag;
        for (int i = 0; i < x.size(); i++) {
            glm::mat3x3 Mi(0);
            for (int j = 0; j < 3; j++) Mi[j][j] = system.Mass / (h * h);
            diag.push_back(Mi);
        }
        for (auto const spring : system.Springs) {
            auto const      p0   = spring.AdjIdx.first;
            auto const      p1   = spring.AdjIdx.second;
            glm::vec3 const x01  = x[p1] - x[p0];
            glm::vec3 const x01_ = system.Positions[p1] - system.Positions[p0];
            float           d01 = glm::length(x01), d01_ = glm::length(x01_);
            glm::vec3 const v01 = ((x[p1] - system.Positions[p1]) - (x[p0] - system.Positions[p0])) / h;
            glm::vec3 const e01 = glm::normalize(x01);
            glm::mat3x3     Mij(0);
            for (int i = 0; i < 3; i++) {
                Mij[i][i] += -(system.Stiffness + system.Damping / h) + system.Stiffness * spring.RestLength * (glm::length2(x01) - x01[i] * x01[i]) / (glm::length2(x01) * glm::length(x01));
                Mij[i][i] -= system.Damping / h * (((glm::dot(x01, x01_) + x01[i] * x01_[i]) * d01 * d01 - 2 * glm::dot(x01, x01_) * x01[i] * x01[i]) / (glm::length2(x01) * glm::length2(x01)));
                diag[p0][i][i] += (system.Stiffness + system.Damping / h) - system.Stiffness * spring.RestLength * (glm::length2(x01) - x01[i] * x01[i]) / (glm::length2(x01) * glm::length(x01));
                diag[p0][i][i] += system.Damping / h * (((glm::dot(x01, x01_) + x01[i] * x01_[i]) * d01 * d01 - 2 * glm::dot(x01, x01_) * x01[i] * x01[i]) / (glm::length2(x01) * glm::length2(x01)));
                diag[p1][i][i] += (system.Stiffness + system.Damping / h) - system.Stiffness * spring.RestLength * (glm::length2(x01) - x01[i] * x01[i]) / (glm::length2(x01) * glm::length(x01));
                diag[p1][i][i] += system.Damping / h * (((glm::dot(x01, x01_) + x01[i] * x01_[i]) * d01 * d01 - 2 * glm::dot(x01, x01_) * x01[i] * x01[i]) / (glm::length2(x01) * glm::length2(x01)));

                for (int j = 0; j < 3; j++) {
                    if (j == i) continue;
                    Mij[i][j] += -2 * system.Stiffness * spring.RestLength * (x01[i] * x01[j]) / (glm::length2(x01) * glm::length(x01));
                    diag[p0][i][j] += 2 * system.Stiffness * spring.RestLength * (x01[i] * x01[j]) / (glm::length2(x01) * glm::length(x01));
                    diag[p1][i][j] += 2 * system.Stiffness * spring.RestLength * (x01[i] * x01[j]) / (glm::length2(x01) * glm::length(x01));
                    Mij[i][j] += -system.Damping / h / (glm::length2(x01) * glm::length2(x01)) * (x01[i] * x01_[j] * glm::length2(x01) - x01[i] * glm::dot(x01, x01_) * 2 * x01[j]);
                    diag[p0][i][j] += system.Damping / h / (glm::length2(x01) * glm::length2(x01)) * (x01[i] * x01_[j] * glm::length2(x01) - x01[i] * glm::dot(x01, x01_) * 2 * x01[j]);
                    diag[p1][i][j] += system.Damping / h / (glm::length2(x01) * glm::length2(x01)) * (x01[i] * x01_[j] * glm::length2(x01) - x01[i] * glm::dot(x01, x01_) * 2 * x01[j]);
                }
            }
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    A.emplace_back(p0 * 3 + i, p1 * 3 + j, Mij[i][j]);
                    A.emplace_back(p1 * 3 + j, p0 * 3 + i, Mij[i][j]);
                }
            }
        }
        for (int i = 0; i < x.size(); i++) {
            for (int a = 0; a < 3; a++) {
                for (int b = 0; b < 3; b++) {
                    A.emplace_back(i * 3 + a, i * 3 + b, diag[i][a][b]);
                }
            }
        }
    }

    void ForwardKinematics(IKSystem & ik, int StartIndex) {
        if (StartIndex == 0) {
            ik.JointGlobalRotation[0] = ik.JointLocalRotation[0];
            ik.JointGlobalPosition[0] = ik.JointLocalOffset[0];
            StartIndex                = 1;
        }
        for (int i = StartIndex; i < ik.JointLocalOffset.size(); i++) {
            // your code here: forward kinematics
            ik.JointGlobalPosition[i] = ik.JointGlobalPosition[i - 1] + glm::mat3_cast(ik.JointLocalRotation[i]) * ik.JointLocalOffset[i];
            ik.JointGlobalRotation[i] = ik.JointLocalRotation[i] * ik.JointGlobalRotation[i - 1];
        }
    }

    void InverseKinematicsCCD(IKSystem & ik, const glm::vec3 & EndPosition, int maxCCDIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        // These functions will be useful: glm::normalize, glm::rotation, glm::quat * glm::quat
        for (int CCDIKIteration = 0; CCDIKIteration < maxCCDIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; CCDIKIteration++) {
            // your code here: ccd ik
            for (int i = ik.JointLocalOffset.size() - 1; i > 0; i--) {
                glm::vec3 d1             = glm::normalize(EndPosition - ik.JointGlobalPosition[i - 1]);
                glm::vec3 d2             = glm::normalize(ik.JointGlobalPosition[ik.JointLocalOffset.size() - 1] - ik.JointGlobalPosition[i - 1]);
                ik.JointLocalRotation[i] = glm::rotation(d2, d1) * ik.JointLocalRotation[i];
                ForwardKinematics(ik, i - 1);
            }
        }
    }

    void InverseKinematicsFABR(IKSystem & ik, const glm::vec3 & EndPosition, int maxFABRIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        int                    nJoints = ik.NumJoints();
        std::vector<glm::vec3> backward_positions(nJoints, glm::vec3(0, 0, 0)), forward_positions(nJoints, glm::vec3(0, 0, 0));
        for (int IKIteration = 0; IKIteration < maxFABRIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; IKIteration++) {
            // task: fabr ik
            // backward update
            glm::vec3 next_position         = EndPosition;
            backward_positions[nJoints - 1] = EndPosition;

            for (int i = nJoints - 2; i >= 0; i--) {
                // your code here
                glm::vec3 back_dir    = glm::normalize(ik.JointGlobalPosition[i] - next_position);
                backward_positions[i] = backward_positions[i + 1] + back_dir * ik.JointOffsetLength[i + 1];
                next_position         = backward_positions[i];
            }

            // forward update
            glm::vec3 now_position = ik.JointGlobalPosition[0];
            forward_positions[0]   = ik.JointGlobalPosition[0];
            for (int i = 0; i < nJoints - 1; i++) {
                // your code here
                glm::vec3 forward_dir    = glm::normalize(backward_positions[i + 1] - now_position);
                forward_positions[i + 1] = forward_positions[i] + forward_dir * ik.JointOffsetLength[i + 1];
                now_position             = forward_positions[i + 1];
            }
            ik.JointGlobalPosition = forward_positions; // copy forward positions to joint_positions
        }

        // Compute joint rotation by position here.
        for (int i = 0; i < nJoints - 1; i++) {
            ik.JointGlobalRotation[i] = glm::rotation(glm::normalize(ik.JointLocalOffset[i + 1]), glm::normalize(ik.JointGlobalPosition[i + 1] - ik.JointGlobalPosition[i]));
        }
        ik.JointLocalRotation[0] = ik.JointGlobalRotation[0];
        for (int i = 1; i < nJoints - 1; i++) {
            ik.JointLocalRotation[i] = glm::inverse(ik.JointGlobalRotation[i - 1]) * ik.JointGlobalRotation[i];
        }
        ForwardKinematics(ik, 0);
    }

    IKSystem::Vec3ArrPtr IKSystem::BuildCustomTargetPosition() {
        // get function from https://www.wolframalpha.com/input/?i=Albert+Einstein+curve
        int nums      = 5000;
        using Vec3Arr = std::vector<glm::vec3>;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(nums));
        int                      index = 0;
        for (int i = 0; i < nums; i++) {
            float x_val = 1.5e-3f * custom_x(92 * glm::pi<float>() * i / nums);
            float y_val = 1.5e-3f * custom_y(92 * glm::pi<float>() * i / nums);
            if (std::abs(x_val) < 1e-3 || std::abs(y_val) < 1e-3) continue;
            (*custom)[index++] = glm::vec3(1.6f - x_val, 0.0f, y_val - 0.2f);
        }
        custom->resize(index);
        return custom;
    }

    void AdvanceMassSpringSystem(MassSpringSystem & system, float const dt) {
        // your code here: rewrite following code
        int const   steps = 1000;
        float const ddt   = dt / steps;
        for (std::size_t s = 0; s < steps; s++) {
            auto                       y(system.Positions);
            Eigen::SparseMatrix<float> M(3 * system.Positions.size(), 3 * system.Positions.size());
            std::vector<TRP>           A;
            std::vector<glm::vec3>     b(system.Positions);
            for (std::size_t i = 0; i < system.Positions.size(); i++) {
                y[i] = system.Positions[i] + ddt * system.Velocities[i] + ddt * ddt * glm::vec3(0, -system.Gravity, 0);
            }
            partial_g(system, system.Positions, y, b, ddt);
            partial2_g(system, system.Positions, y, A, ddt);
            M.setFromTriplets(A.begin(), A.end());
            VECXF b_v(3 * system.Positions.size());
            for (int i = 0; i < system.Positions.size(); i++) {
                for (int j = 0; j < 3; j++) b_v[i * 3 + j] = b[i][j];
            }
            LLT   solver(M);
            VECXF x_v = solver.solve(b_v);
            for (int i = 0; i < system.Positions.size(); i++) {
                for (int j = 0; j < 3; j++) {
                    system.Velocities[i][j] = -x_v[i * 3 + j] / ddt;
                    system.Positions[i][j]  = system.Positions[i][j] - x_v[i * 3 + j];
                }
            }
        }
    }
} // namespace VCX::Labs::Animation
