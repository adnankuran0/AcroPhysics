#pragma once
#include "Math/Vector3.h"
#include "Math/Quaternion.h"
#include <vector>
#include "Public/Rigidbody.h"
#include "Public/World.h"

class Scene
{

public:
    Scene() = default;
    void Init(Acro::World& world);
    void Reset();
    void Update(double glfwTime);
    inline std::vector<Acro::Rigidbody>& GetBodies() noexcept { return bodies; }
    inline Acro::Rigidbody GetGround() noexcept { return groundBody; }
private:
    struct BodyInitialState {
        Acro::Math::Vector3 position;
        Acro::Math::Quaternion orientation;
    };
    std::vector<BodyInitialState> initialStates;
    bool requestReset = false;
    std::vector<Acro::Rigidbody> bodies;
    Acro::Rigidbody groundBody{};
};