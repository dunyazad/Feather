#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> // 행렬 변환(translate, rotate, scale 등)
#include <glm/gtc/type_ptr.hpp>         // float* 포인터 변환 등
#include <glm/gtx/transform.hpp>        // 추가 변환 기능 (deprecated 경향 있음)
#include <glm/gtc/quaternion.hpp>       // 쿼터니언
#include <glm/gtx/quaternion.hpp>       // 쿼터니언 추가 기능
#include <glm/gtc/matrix_inverse.hpp>   // 행렬 역행렬 관련 함수
#include <glm/gtc/constants.hpp>        // 상수 (pi 등)
#include <glm/gtc/random.hpp>           // 랜덤 함수
#include <glm/gtc/noise.hpp>            // Perlin 등 노이즈 함수
#include <glm/gtx/norm.hpp>             // norm 관련 함수 (normalize, length 등)
#include <glm/gtx/string_cast.hpp>      // std::string 변환

namespace glm
{
    inline glm::vec3 trianglenormal(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
    {
        return glm::normalize(glm::cross(v1 - v0, v2 - v0));
    }
}
