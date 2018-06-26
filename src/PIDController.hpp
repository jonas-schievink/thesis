#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP

const float MAX_EFFORT = 1.0f;
const float MIN_EFFORT = -1.0f;

/**
 * @brief PID controller implementation.
 */
class PIDController {
    float m_setpoint;   // target
    float m_effort;     // last control effort returned by `update`
    float m_integral;   // cumulative error
    float m_lastError;  // last error, for derivative calculation
    float m_kp, m_ki, m_kd;
    float m_integralLimit;  // integral windup limit

public:
    /**
     * @brief Create a new parameterized controller.
     */
    PIDController(float kp, float ki, float kd, float integralLimit);

    /**
     * @brief Create a useless dummy controller.
     *
     * This is needed to allow default construction before we can get the
     * parameters from ROS, because C++.
     */
    PIDController();

    /**
     * @brief Sets the setpoint of the controller (the target value).
     */
    void setSetpoint(float setpoint);

    /**
     * @brief Updates the PID controller state and returns the control effort.
     * @param actual Actual system state (wheel speed).
     * @param delta Time since the last call to `update`.
     * @return Actuator output (-1...1).
     *
     * This should be called in regular intervals.
     */
    float update(float actual, float delta);

    /**
     * @brief Sets the setpoint and output to 0 and clears the internal state.
     *
     * This can be used to "halt" the robot when it's known that the controller
     * can safely output 0.
     */
    void reset();
};

#endif
