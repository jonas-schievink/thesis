#ifndef PI_HPP
#define PI_HPP

const float MAX_EFFORT = 1.0f;
const float MIN_EFFORT = -1.0f;

/**
 * @brief PI controller implementation.
 */
class PIController {
    float m_setpoint;   // target
    float m_effort;     // last control effort returned by `update`
    float m_integral;   // cumulative error
    float m_kp, m_ki;
    float m_integralLimit;  // integral windup limit

public:
    /**
     * @brief Create a new parameterized controller.
     */
    PIController(float kp, float ki, float integralLimit);

    /**
     * @brief Create a useless dummy controller.
     *
     * This is needed to allow default construction before we can get the
     * parameters from ROS, because C++.
     */
    PIController();

    /**
     * @brief Sets the setpoint of the controller (the target value).
     */
    void setSetpoint(float setpoint);

    /**
     * @brief Updates the PID controller state and returns the control effort.
     * @param actual Actual system state (wheel speed).
     * @param delta Time since the last call to `update`.
     * @return Control effort to apply to the actuators (-1...1).
     *
     * This should be called in regular intervals.
     */
    float update(float actual, float delta);
};

#endif
