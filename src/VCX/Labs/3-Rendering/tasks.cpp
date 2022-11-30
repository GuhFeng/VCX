#include "Labs/3-Rendering/tasks.h"

namespace VCX::Labs::Rendering {

    float cos_vec(glm::vec3 d1, glm::vec3 d2) {
        float product = glm::dot(d1, d2);
        if (product < 0) return 0;
        float len_d1 = glm::dot(d1, d1);
        float len_d2 = glm::dot(d2, d2);
        return sqrt(product * product / (len_d1 * len_d2));
    }

    glm::vec4 GetTexture(Engine::Texture2D<Engine::Formats::RGBA8> const & texture, glm::vec2 const & uvCoord) {
        if (texture.GetSizeX() == 1 || texture.GetSizeY() == 1) return texture.At(0, 0);
        glm::vec2 uv      = glm::fract(uvCoord);
        uv.x              = uv.x * texture.GetSizeX() - .5f;
        uv.y              = uv.y * texture.GetSizeY() - .5f;
        std::size_t xmin  = std::size_t(glm::floor(uv.x) + texture.GetSizeX()) % texture.GetSizeX();
        std::size_t ymin  = std::size_t(glm::floor(uv.y) + texture.GetSizeY()) % texture.GetSizeY();
        std::size_t xmax  = (xmin + 1) % texture.GetSizeX();
        std::size_t ymax  = (ymin + 1) % texture.GetSizeY();
        float       xfrac = glm::fract(uv.x), yfrac = glm::fract(uv.y);
        return glm::mix(glm::mix(texture.At(xmin, ymin), texture.At(xmin, ymax), yfrac), glm::mix(texture.At(xmax, ymin), texture.At(xmax, ymax), yfrac), xfrac);
    }

    glm::vec4 GetAlbedo(Engine::Material const & material, glm::vec2 const & uvCoord) {
        glm::vec4 albedo       = GetTexture(material.Albedo, uvCoord);
        glm::vec3 diffuseColor = albedo;
        return glm::vec4(glm::pow(diffuseColor, glm::vec3(2.2)), albedo.w);
    }

    /******************* 1. Ray-triangle intersection *****************/
    bool IntersectTriangle(Intersection & output, Ray const & ray, glm::vec3 const & p1, glm::vec3 const & p2, glm::vec3 const & p3) {
        // your code here
        glm::vec3   v1 = p2 - p1, v2 = p3 - p1;
        glm::vec3   d = ray.Direction, p = ray.Origin;
        glm::mat3x3 M(-d[0], -d[1], -d[2], v1[0], v1[1], v1[2], v2[0], v2[1], v2[2]);
        if (glm::determinant(M) == 0) return false;
        glm::vec3 out = glm::inverse(M) * (p - p1);
        output.t      = out[0];
        output.u      = out[1];
        output.v      = out[2];
        if (output.u + output.v < 1 && output.u > 0 && output.v > 0 && output.t > 0) {
            return true;
        }
        return false;
    }

    glm::vec3 RayTrace(const RayIntersector & intersector, Ray ray, int maxDepth, bool enableShadow) {
        glm::vec3 color(0.0f);
        glm::vec3 weight(1.0f);

        for (int depth = 0; depth < maxDepth; depth++) {
            auto rayHit = intersector.IntersectRay(ray);
            if (! rayHit.IntersectState) return color;
            const glm::vec3 pos       = rayHit.IntersectPosition;
            const glm::vec3 n         = glm::normalize(rayHit.IntersectNormal);
            const glm::vec3 kd        = rayHit.IntersectAlbedo;
            const glm::vec3 ks        = rayHit.IntersectMetaSpec;
            const float     alpha     = rayHit.IntersectAlbedo.w;
            const float     shininess = rayHit.IntersectMetaSpec.w * 256;

            glm::vec3 result(0.0f);
            /******************* 2. Whitted-style ray tracing *****************/
            // your code here
            glm::vec3 d = -glm::normalize(ray.Direction);
            for (const Engine::Light & light : intersector.InternalScene->Lights) {
                glm::vec3 l;
                float     attenuation;
                /******************* 3. Shadow ray *****************/
                if (light.Type == Engine::LightType::Point) {
                    l           = light.Position - pos;
                    attenuation = 1.0f / glm::dot(l, l);
                    if (enableShadow) {
                        // your code here
                        glm::vec3 new_pos = pos;
                        while (1) {
                            auto new_hit = intersector.IntersectRay(Ray(new_pos, l));
                            if (! new_hit.IntersectState) break;
                            else {
                                glm::vec3 hit_pos = new_hit.IntersectPosition;
                                if (dot(l, l) <= dot(hit_pos - pos, l)) break;
                                else if (new_hit.IntersectAlbedo.w < 0.2)
                                    new_pos = new_hit.IntersectPosition;
                                else {
                                    attenuation *= (1 - new_hit.IntersectAlbedo.w);
                                    break;
                                }
                            }
                        }
                    }
                }

                else if (light.Type == Engine::LightType::Directional) {
                    l           = light.Direction;
                    attenuation = 1.0f;
                    if (enableShadow) {
                        // your code here
                        glm::vec3 new_pos = pos;
                        while (1) {
                            auto new_hit = intersector.IntersectRay(Ray(new_pos, l));
                            if (! new_hit.IntersectState) break;
                            else if (new_hit.IntersectAlbedo.w < 0.2)
                                new_pos = new_hit.IntersectPosition;
                            else {
                                attenuation *= (1 - new_hit.IntersectAlbedo.w);
                                break;
                            }
                        }
                    }
                }

                /******************* 2. Whitted-style ray tracing *****************/
                // your code here
                l                         = glm::normalize(l);
                glm::vec3 mirror          = n * l;
                mirror                    = n * (mirror[0] + mirror[1] + mirror[2]);
                glm::vec3 lightDir_mirror = glm::normalize(mirror + mirror - l);
                float     cos_theta       = cos_vec(l, n);
                float     cos_phi         = cos_vec(lightDir_mirror, d);
                result                    = (kd * glm::vec3(cos_theta) + ks * glm::vec3(pow(cos_phi, shininess))) * light.Intensity * attenuation;
            }

            if (alpha < 0.9) {
                // refraction
                // accumulate color
                glm::vec3 R = alpha * glm::vec3(1.0f);
                color += weight * R * result;
                weight *= glm::vec3(1.0f) - R;

                // generate new ray
                ray = Ray(pos, ray.Direction);
            } else {
                // reflection
                // accumulate color
                glm::vec3 R = ks * glm::vec3(0.5f);
                color += weight * (glm::vec3(1.0f) - R) * result;
                weight *= R;

                // generate new ray
                glm::vec3 out_dir = ray.Direction - glm::vec3(2.0f) * n * glm::dot(n, ray.Direction);
                ray               = Ray(pos, out_dir);
            }
        }

        return color;
    }
} // namespace VCX::Labs::Rendering