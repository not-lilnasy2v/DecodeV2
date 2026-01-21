package org.firstinspires.ftc.teamcode.NouHard;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ServoImplExEx {
    private final ServoImplEx servo;
    private double minPos = 0.0, maxPos = 1.0;

    private double centerPos = 0.5;
    private double degreesPerUnit = 355;

    private ServoImplExEx(ServoImplEx servo) {
        this.servo = servo;
    }

    /**
     * Factory method to get a ServoImplExEx from the hardware map.
     * @param hardwareMap the hardware map
     * @param name the device name configured in the robot configuration
     * @return a new ServoImplExEx wrapping the servo
     */
    public static ServoImplExEx get(HardwareMap hardwareMap, String name) {
        ServoImplEx baseServo = hardwareMap.get(ServoImplEx.class, name);
        return new ServoImplExEx(baseServo);
    }

    public synchronized void setMinPosition(double minPosition) {
        minPos = minPosition;
    }

    public synchronized void setMaxPosition(double maxPosition) {
        maxPos = maxPosition;
    }

    public synchronized void setCenterPosition(double center) {
        centerPos = center;
    }

    public synchronized void setDegreesPerUnit(double degrees) {
        degreesPerUnit = degrees;
    }

    public double getCenterPosition() {
        return centerPos;
    }

    /**
     * Converts an angle in degrees to a servo position.
     * @param angleDegrees the angle in degrees (0 = center, positive = one direction, negative = other)
     * @return the servo position (clamped to min/max)
     */
    public double angleToPosition(double angleDegrees) {
        // Convert angle to servo position: center + (angle / totalDegrees)
        double position = centerPos + (angleDegrees / degreesPerUnit);
        // Clamp to min/max
        if (position < minPos) return minPos;
        if (position > maxPos) return maxPos;
        return position;
    }

    public synchronized void setPosition(double position) {
        if (position < minPos) {
            servo.setPosition(minPos);
        } else if (maxPos < position) {
            servo.setPosition(maxPos);
        } else {
            servo.setPosition(position);
        }
    }

    // Delegate common methods to the underlying servo
    public double getPosition() {
        return servo.getPosition();
    }

    public void setDirection(ServoImplEx.Direction direction) {
        servo.setDirection(direction);
    }

    public ServoImplEx.Direction getDirection() {
        return servo.getDirection();
    }

    public void setPwmRange(ServoImplEx.PwmRange range) {
        servo.setPwmRange(range);
    }

    public void setPwmEnable() {
        servo.setPwmEnable();
    }

    public void setPwmDisable() {
        servo.setPwmDisable();
    }
}
