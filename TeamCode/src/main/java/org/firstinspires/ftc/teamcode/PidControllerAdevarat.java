package org.firstinspires.ftc.teamcode;

/**
 * PID controller - originally courtesy of Peter Tischler.
 * Fixed version with proper time handling, output clamping, and anti-windup.
 */
public class PidControllerAdevarat
{
    private double m_P;                     // factor for "proportional" control
    private double m_I;                     // factor for "integral" control
    private double m_D;                     // factor for "derivative" control
    private double m_input;                 // sensor input for pid controller
    private double m_maximumOutput = 1.0;   // maximum output
    private double m_minimumOutput = -1.0;  // minimum output
    private boolean m_outputClampEnabled = false;  // only clamp if explicitly set
    private double m_maximumInput = 0.0;    // maximum input - limit setpoint to this
    private double m_minimumInput = 0.0;    // minimum input - limit setpoint to this
    private boolean m_continuous = false;   // do the endpoints wrap around? eg. Absolute encoder
    private boolean m_enabled = false;      // is the pid controller enabled
    private double m_prevError = 0.0;       // previous error for derivative
    private double m_totalError = 0.0;      // the sum of the errors for use in the integral calc
    private double m_tolerance = 0.01;      // the absolute error that is considered on target
    private double m_setpoint = 0.0;
    private double m_error = 0.0;
    private double m_result = 0.0;
    private long m_lastTime = 0;            // last time in nanoseconds

    public PidControllerAdevarat(double Kp, double Ki, double Kd)
    {
        m_P = Kp;
        m_I = Ki;
        m_D = Kd;
    }

    /**
     * Core PID calculation with time correction.
     */
    private void calculate()
    {
        if (m_enabled)
        {
            // Calculate delta time in seconds using nanosecond precision
            long now = System.nanoTime();
            double dt;
            if (m_lastTime == 0) {
                dt = 0.02; // Assume 50Hz on first call
            } else {
                dt = (now - m_lastTime) / 1e9; // Convert nanos to seconds
            }
            m_lastTime = now;

            // Prevent division by zero or negative dt
            if (dt <= 0) {
                dt = 0.001;
            }

            // Calculate the error signal
            m_error = m_setpoint - m_input;

            // If continuous is set to true allow wrap around
            if (m_continuous)
            {
                double range = m_maximumInput - m_minimumInput;
                if (range > 0 && Math.abs(m_error) > range / 2)
                {
                    if (m_error > 0)
                        m_error = m_error - range;
                    else
                        m_error = m_error + range;
                }
            }

            // Integrate the errors with anti-windup
            // Only accumulate if:
            // 1. Error is above tolerance (prevents small oscillation windup)
            // 2. Integral term won't exceed output bounds (prevents saturation windup)
            if (Math.abs(m_error) > m_tolerance)
            {
                double potentialTotalError = m_totalError + m_error * dt;
                double potentialI = potentialTotalError * m_I;

                // Signed check - allows negative integral to be bounded correctly
                if (potentialI >= m_minimumOutput && potentialI <= m_maximumOutput)
                {
                    m_totalError = potentialTotalError;
                }
            }

            // Perform the primary PID calculation with time correction
            // P: proportional to current error
            // I: proportional to accumulated error over time
            // D: proportional to rate of change of error
            double derivative = (m_error - m_prevError) / dt;
            m_result = m_P * m_error + m_I * m_totalError + m_D * derivative;

            // Clamp output only if explicitly enabled via setOutputRange()
            if (m_outputClampEnabled)
            {
                m_result = Math.max(m_minimumOutput, Math.min(m_maximumOutput, m_result));
            }

            // Store error for next derivative calculation
            m_prevError = m_error;
        }
    }

    /**
     * Set the PID Controller gain parameters.
     * Resets integral if Ki changes.
     * @param p Proportional coefficient
     * @param i Integral coefficient
     * @param d Differential coefficient
     */
    public void setPID(double p, double i, double d)
    {
        if (i != m_I) {
            m_totalError = 0;
        }
        m_P = p;
        m_I = i;
        m_D = d;
    }

    /**
     * Get the Proportional coefficient
     * @return proportional coefficient
     */
    public double getP()
    {
        return m_P;
    }

    /**
     * Get the Integral coefficient
     * @return integral coefficient
     */
    public double getI()
    {
        return m_I;
    }

    /**
     * Get the Differential coefficient
     * @return differential coefficient
     */
    public double getD()
    {
        return m_D;
    }

    /**
     * Return the current PID result for the last input set with setInput().
     * @return the latest calculated output
     */
    public double performPID()
    {
        calculate();
        return m_result;
    }

    /**
     * Return the current PID result for the specified input.
     * @param input Input value to be used to calculate the PID result.
     * @return the latest calculated output
     */
    public double performPID(double input)
    {
        setInput(input);
        return performPID();
    }

    /**
     * Set the PID controller to consider the input to be continuous,
     * Rather than using the max and min as constraints, it considers them to
     * be the same point and automatically calculates the shortest route to
     * the setpoint. Useful for angles.
     * @param continuous Set to true turns on continuous, false turns off continuous
     */
    public void setContinuous(boolean continuous)
    {
        m_continuous = continuous;
    }

    /**
     * Set the PID controller to consider the input to be continuous.
     */
    public void setContinuous()
    {
        this.setContinuous(true);
    }

    /**
     * Sets the maximum and minimum values expected from the input.
     * Used for continuous mode wrap-around and input clamping.
     *
     * @param minimumInput the minimum value expected from the input
     * @param maximumInput the maximum value expected from the input
     */
    public void setInputRange(double minimumInput, double maximumInput)
    {
        m_minimumInput = minimumInput;
        m_maximumInput = maximumInput;
        setSetpoint(m_setpoint);
    }

    /**
     * Sets the minimum and maximum output values.
     * Enables output clamping when called.
     *
     * @param minimumOutput the minimum value to write to the output
     * @param maximumOutput the maximum value to write to the output
     */
    public void setOutputRange(double minimumOutput, double maximumOutput)
    {
        m_minimumOutput = minimumOutput;
        m_maximumOutput = maximumOutput;
        m_outputClampEnabled = true;
    }

    /**
     * Set the setpoint for the PIDController
     * @param setpoint the desired setpoint
     */
    public void setSetpoint(double setpoint)
    {
        // Clamp setpoint to input range if range is valid
        if (m_maximumInput > m_minimumInput)
        {
            if (setpoint > m_maximumInput)
                m_setpoint = m_maximumInput;
            else if (setpoint < m_minimumInput)
                m_setpoint = m_minimumInput;
            else
                m_setpoint = setpoint;
        }
        else
        {
            m_setpoint = setpoint;
        }
    }

    /**
     * Returns the current setpoint of the PIDController
     * @return the current setpoint
     */
    public double getSetpoint()
    {
        return m_setpoint;
    }

    /**
     * Returns the current difference of the input from the setpoint
     * @return the current error
     */
    public synchronized double getError()
    {
        return m_error;
    }

    /**
     * Set the absolute error which is considered tolerable for use with onTarget().
     * @param tolerance absolute error which is tolerable
     */
    public void setTolerance(double tolerance)
    {
        m_tolerance = Math.abs(tolerance);
    }

    /**
     * Return true if the error is within the tolerance.
     * Correctly handles continuous mode wrap-around.
     * @return true if the error is less than the tolerance
     */
    public boolean onTarget()
    {
        // Calculate error with continuous wrap-around if enabled
        double error = m_setpoint - m_input;

        if (m_continuous)
        {
            double range = m_maximumInput - m_minimumInput;
            if (range > 0 && Math.abs(error) > range / 2)
            {
                if (error > 0)
                    error = error - range;
                else
                    error = error + range;
            }
        }

        return (Math.abs(error) < m_tolerance);
    }

    /**
     * Begin running the PIDController
     */
    public void enable()
    {
        m_enabled = true;
    }

    /**
     * Stop running the PIDController.
     */
    public void disable()
    {
        m_enabled = false;
    }

    /**
     * Check if the PID controller is enabled
     * @return true if enabled
     */
    public boolean enabled()
    {
        return m_enabled;
    }

    /**
     * Reset the previous error, the integral term, timestamp, and disable the controller.
     */
    public void reset()
    {
        disable();
        m_prevError = 0;
        m_totalError = 0;
        m_result = 0;
        m_lastTime = 0;
    }

    /**
     * Set the input value to be used by the next call to performPID().
     * @param input Input value to the PID calculation.
     */
    public void setInput(double input)
    {
        // Clamp input to range if range is valid (normal clamping, no weird minimum-forcing)
        if (m_maximumInput > m_minimumInput)
        {
            if (input > m_maximumInput)
                m_input = m_maximumInput;
            else if (input < m_minimumInput)
                m_input = m_minimumInput;
            else
                m_input = input;
        }
        else
        {
            m_input = input;
        }
    }

    /**
     * Get the integral sum (for debugging/tuning)
     * @return the accumulated integral (time-corrected)
     */
    public double getISum()
    {
        return m_totalError;
    }
}
