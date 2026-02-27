package org.firstinspires.ftc.teamcode.NouHard;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ServoImplExEx {
    private final ServoImplEx servo;
    private AnalogInput encoder = null;

    // Pentru servo normal (positional)
    private double minPos = 0.0, maxPos = 1.0;
    private double centerPos = 0.5;
    private double degreesPerUnit = 355;

    // Pentru servo continuu (CR) cu encoder
    private boolean isContinuous = false;
    private double encoderMaxVoltage = 3.3;
    private double encoderReferenceVoltage = 0.0;  // Voltaj la 0 grade
    private double minAngle = -180.0;  // Limita stanga in grade
    private double maxAngle = 180.0;   // Limita dreapta in grade
    private double deadzone = 2.0;     // Grade - sub aceasta eroare, servo se opreste
    private double speed = 0.3;        // Viteza CR servo (0-0.5)
    private boolean invertDirection = false;
    private double lastRawAngle = Double.NaN;
    private double angleOffset = 0;

    private ServoImplExEx(ServoImplEx servo) {
        this.servo = servo;
    }

    /**
     * Factory method pentru servo normal (positional)
     */
    public static ServoImplExEx get(HardwareMap hardwareMap, String name) {
        ServoImplEx baseServo = hardwareMap.get(ServoImplEx.class, name);
        return new ServoImplExEx(baseServo);
    }

    /**
     * Factory method pentru servo continuu cu encoder analogic (Axon MAX)
     */
    public static ServoImplExEx getContinuous(HardwareMap hardwareMap, String servoName, String encoderName) {
        ServoImplEx baseServo = hardwareMap.get(ServoImplEx.class, servoName);
        AnalogInput analogEncoder = hardwareMap.get(AnalogInput.class, encoderName);
        ServoImplExEx instance = new ServoImplExEx(baseServo);
        instance.encoder = analogEncoder;
        instance.isContinuous = true;
        return instance;
    }

    // ==================== CONFIGURARE GENERALA ====================

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

    public boolean isContinuous() {
        return isContinuous;
    }

    // ==================== CONFIGURARE CR SERVO ====================

    /**
     * Seteaza voltajul maxim al encoderului (default 3.3V)
     */
    public synchronized void setEncoderMaxVoltage(double voltage) {
        encoderMaxVoltage = voltage;
    }

    /**
     * Seteaza voltajul de referinta (cand turela e la 0 grade)
     * Apeleaza aceasta functie cand turela e in pozitia "inainte"
     */
    public synchronized void setEncoderReference() {
        if (encoder != null) {
            encoderReferenceVoltage = encoder.getVoltage();
        }
        lastRawAngle = Double.NaN;
        angleOffset = 0;
    }

    public synchronized void setEncoderReferenceVoltage(double voltage) {
        encoderReferenceVoltage = voltage;
        lastRawAngle = Double.NaN;
        angleOffset = 0;
    }

    public double getEncoderReferenceVoltage() {
        return encoderReferenceVoltage;
    }

    /**
     * Seteaza limitele unghiului in grade
     */
    public synchronized void setAngleLimits(double minDegrees, double maxDegrees) {
        minAngle = minDegrees;
        maxAngle = maxDegrees;
    }

    public synchronized void setMinAngle(double degrees) {
        minAngle = degrees;
    }

    public synchronized void setMaxAngle(double degrees) {
        maxAngle = degrees;
    }

    public double getMinAngle() {
        return minAngle;
    }

    public double getMaxAngle() {
        return maxAngle;
    }

    /**
     * Seteaza deadzone-ul in grade (sub aceasta eroare servo-ul se opreste)
     */
    public synchronized void setDeadzone(double degrees) {
        deadzone = degrees;
    }

    /**
     * Seteaza viteza CR servo (0.0 - 0.5)
     */
    public synchronized void setSpeed(double speedValue) {
        speed = Math.max(0.0, Math.min(0.5, speedValue));
    }

    /**
     * Inverseaza directia servo-ului
     */
    public synchronized void setInvertDirection(boolean invert) {
        invertDirection = invert;
    }

    // ==================== CITIRE ENCODER ====================

    /**
     * Citeste voltajul raw de la encoder
     */
    public double getEncoderVoltage() {
        if (encoder == null) return 0;
        return encoder.getVoltage();
    }

    /**
     * Citeste unghiul curent in grade (relativ la referinta)
     * 0 = pozitia de referinta, pozitiv/negativ = stanga/dreapta
     */
    public double getCurrentAngle() {
        if (encoder == null) return 0;

        double voltage = encoder.getVoltage();
        double absoluteDegrees = (voltage / encoderMaxVoltage) * 360.0;
        double referenceDegrees = (encoderReferenceVoltage / encoderMaxVoltage) * 360.0;
        double rawAngle = absoluteDegrees - referenceDegrees;

        if (!Double.isNaN(lastRawAngle)) {
            double delta = rawAngle - lastRawAngle;
            if (delta > 180) angleOffset -= 360;
            if (delta < -180) angleOffset += 360;
        }
        lastRawAngle = rawAngle;

        return rawAngle + angleOffset;
    }

    /**
     * Verifica daca unghiul curent e in limite
     */
    public boolean isWithinLimits() {
        double angle = getCurrentAngle();
        return angle >= minAngle && angle <= maxAngle;
    }

    /**
     * Verifica daca unghiul curent e la limita stanga
     */
    public boolean isAtMinLimit() {
        return getCurrentAngle() <= minAngle;
    }

    /**
     * Verifica daca unghiul curent e la limita dreapta
     */
    public boolean isAtMaxLimit() {
        return getCurrentAngle() >= maxAngle;
    }

    // ==================== CONTROL CR SERVO ====================

    /**
     * Opreste CR servo-ul (setPosition 0.5)
     */
    public void stop() {
        servo.setPosition(0.5);
    }

    /**
     * Seteaza puterea CR servo (-1.0 la 1.0)
     * -1.0 = full speed stanga, 0 = stop, 1.0 = full speed dreapta
     * Respecta limitele din encoder!
     */
    public void setPower(double power) {
        if (!isContinuous) {
            // Pentru servo normal, converteste power la position
            setPosition(0.5 + power * 0.5);
            return;
        }

        // Verifica limitele
        double currentAngle = getCurrentAngle();
        if (power > 0 && currentAngle >= maxAngle) {
            power = 0;
        }
        if (power < 0 && currentAngle <= minAngle) {
            power = 0;
        }

        double position = 0.5 + (power * 0.5);
        if (invertDirection) {
            position = 1.0 - position;
        }
        servo.setPosition(position);
    }

    /**
     * Misca CR servo-ul catre un unghi target
     * Returneaza true daca a ajuns (eroare < deadzone)
     */
    public boolean moveToAngle(double targetDegrees) {
        if (!isContinuous || encoder == null) {
            return true;
        }

        // Clamp target la limite
        targetDegrees = Math.max(minAngle, Math.min(maxAngle, targetDegrees));

        double currentAngle = getCurrentAngle();
        double error = targetDegrees - currentAngle;

        if (Math.abs(error) < deadzone) {
            stop();
            return true;
        }

        double power;
        if (error > 0) {
            power = speed;
        } else {
            power = -speed;
        }

        // Verifica limitele
        if (currentAngle >= maxAngle && error > 0) {
            stop();
            return true;
        }
        if (currentAngle <= minAngle && error < 0) {
            stop();
            return true;
        }

        double position = 0.5 + power;
        if (invertDirection) {
            position = 1.0 - position;
        }
        servo.setPosition(position);

        return false;
    }

    /**
     * Salveaza limita curenta ca limita stanga
     */
    public void saveCurrentAsMinLimit() {
        minAngle = getCurrentAngle();
    }

    /**
     * Salveaza limita curenta ca limita dreapta
     */
    public void saveCurrentAsMaxLimit() {
        maxAngle = getCurrentAngle();
    }

    // ==================== SERVO NORMAL (POSITIONAL) ====================

    /**
     * Converteste un unghi in grade la pozitie servo (pentru servo normal)
     */
    public double angleToPosition(double angleDegrees) {
        double position = centerPos + (angleDegrees / degreesPerUnit);
        if (position < minPos) return minPos;
        if (position > maxPos) return maxPos;
        return position;
    }

    /**
     * Seteaza pozitia servo-ului (0.0 - 1.0)
     * Pentru servo normal: pozitie efectiva
     * Pentru CR servo: 0.5 = stop, 0 = full stanga, 1 = full dreapta
     */
    public synchronized void setPosition(double position) {
        if (position < minPos) {
            servo.setPosition(minPos);
        } else if (maxPos < position) {
            servo.setPosition(maxPos);
        } else {
            servo.setPosition(position);
        }
    }

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
