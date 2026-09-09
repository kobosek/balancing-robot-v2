#include "control_math/WheelVelocityController.hpp"

int main()
{
    const control_math::PidParameters parameters{
        0.01f, 0.05f, 0.0001f,
        -1.0f, 1.0f,
        -10.0f, 10.0f
    };
    control_math::WheelVelocityController controller(parameters);

    const auto result = controller.update(120.0f, 100.0f, 0.005f);
    return result.valid ? 0 : 1;
}
