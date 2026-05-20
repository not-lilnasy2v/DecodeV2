package org.firstinspires.ftc.teamcode.NouHard;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RTPAxon {

    public enum Direction { FORWARD, REVERSE }

    private final AnalogInput servoEncoder;
    private final ServoImplEx master;
    private final ServoImplEx slave;

    private boolean rtp;
    private double power;
    private double maxPower;
    private Direction direction;

    private double previousAngle;
    private double totalRotation;
    private double targetRotation;

    private double kP;
    private double kI;
    private double kD;
    private double integralSum;
    private double lastError;
    private double maxIntegralSum;
    private ElapsedTime pidTimer;

    public double STARTPOS;
    public int ntry = 0;
    public int cliffs = 0;
    public double homeAngle;

    private double encoderReferenceVoltage = 0.0;
    private double minAngle = -Double.MAX_VALUE;
    private double maxAngle =  Double.MAX_VALUE;

    public RTPAxon(ServoImplEx servo, AnalogInput encoder) {
        this(servo, null, encoder, Direction.FORWARD);
    }

    public RTPAxon(ServoImplEx servo, AnalogInput encoder, Direction direction) {
        this(servo, null, encoder, direction);
    }

    public RTPAxon(ServoImplEx master, ServoImplEx slave, AnalogInput encoder) {
        this(master, slave, encoder, Direction.FORWARD);
    }

    public RTPAxon(ServoImplEx master, ServoImplEx slave, AnalogInput encoder, Direction direction) {
        this.master = master;
        this.slave  = slave;
        this.servoEncoder = encoder;
        this.direction = direction;
        rtp = true;
        initialize();
    }

    public static RTPAxon getDual(HardwareMap hw, String masterName, String slaveName, String encoderName) {
        return new RTPAxon(
                hw.get(ServoImplEx.class, masterName),
                hw.get(ServoImplEx.class, slaveName),
                hw.get(AnalogInput.class, encoderName));
    }

    public static RTPAxon getSingle(HardwareMap hw, String servoName, String encoderName) {
        return new RTPAxon(
                hw.get(ServoImplEx.class, servoName),
                null,
                hw.get(AnalogInput.class, encoderName));
    }

    private void initialize() {
        setPowerRaw(0);
        busyWait(50_000_000L);

        do {
            STARTPOS = getCurrentAngle();
            if (Math.abs(STARTPOS) > 1) {
                previousAngle = getCurrentAngle();
            } else {
                busyWait(50_000_000L);
            }
            ntry++;
        } while (Math.abs(previousAngle) < 0.2 && (ntry < 50));

        totalRotation = 0;
        homeAngle = previousAngle;

        kP = 0.015;
        kI = 0.0005;
        kD = 0.0025;
        integralSum = 0.0;
        lastError = 0.0;
        maxIntegralSum = 100.0;
        pidTimer = new ElapsedTime();
        pidTimer.reset();

        maxPower = 0.25;
        cliffs = 0;
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public void setPower(double power) {
        this.power = Math.max(-maxPower, Math.min(maxPower, power));
        setPowerRaw(this.power * (direction == Direction.REVERSE ? -1 : 1));
    }

    private void setPowerRaw(double signedPower) {
        double pos = 0.5 - signedPower;
        if (pos < 0.0) pos = 0.0;
        if (pos > 1.0) pos = 1.0;
        if (master != null) master.setPosition(pos);
        if (slave  != null) slave.setPosition(pos);
    }

    public double getPower()                 { return power; }
    public void   setMaxPower(double m)      { maxPower = m; }
    public double getMaxPower()              { return maxPower; }
    public void   setRtp(boolean r)          { rtp = r; if (r) resetPID(); }
    public boolean getRtp()                  { return rtp; }
    public void   setKP(double v)            { kP = v; }
    public void   setKI(double v)            { kI = v; resetIntegral(); }
    public void   setKD(double v)            { kD = v; }
    public void   setPidCoeffs(double p, double i, double d) { setKP(p); setKI(i); setKD(d); }
    public double getKP()                    { return kP; }
    public double getKI()                    { return kI; }
    public double getKD()                    { return kD; }
    public void   setK(double k)             { setKP(k); }
    public double getK()                     { return getKP(); }
    public void   setMaxIntegralSum(double m){ maxIntegralSum = m; }
    public double getMaxIntegralSum()        { return maxIntegralSum; }
    public double getTotalRotation()         { return totalRotation; }
    public double getTargetRotation()        { return targetRotation; }

    public void changeTargetRotation(double change) {
        targetRotation += change;
        clampTarget();
    }

    public void setTargetRotation(double target) {
        targetRotation = target;
        clampTarget();
        resetPID();
    }

    public double getCurrentAngle() {
        if (servoEncoder == null) return 0;
        double voltage = servoEncoder.getVoltage();
        double absoluteDeg = (voltage / 3.3) * 360.0;
        double refDeg      = (encoderReferenceVoltage / 3.3) * 360.0;
        double angle = absoluteDeg - refDeg;

        while (angle >= 180.0)  angle -= 360.0;
        while (angle <  -180.0) angle += 360.0;

        return direction == Direction.REVERSE ? -angle : angle;
    }

    public boolean isAtTarget() {
        return isAtTarget(5);
    }

    public boolean isAtTarget(double tolerance) {
        return Math.abs(targetRotation - totalRotation) < tolerance;
    }

    public void forceResetTotalRotation() {
        totalRotation = 0;
        previousAngle = getCurrentAngle();
        homeAngle = previousAngle;
        cliffs = 0;
        resetPID();
    }

    public void resetPID() {
        resetIntegral();
        lastError = 0;
        if (pidTimer != null) pidTimer.reset();
    }

    public void resetIntegral() {
        integralSum = 0;
    }

    public void setEncoderReferenceVoltage(double voltage) {
        encoderReferenceVoltage = voltage;
        previousAngle = getCurrentAngle();
        homeAngle = 0;
        totalRotation = previousAngle;
        cliffs = 0;
        resetPID();
    }

    public void setEncoderReferenceFromCurrent() {
        if (servoEncoder != null) setEncoderReferenceVoltage(servoEncoder.getVoltage());
    }

    public double getEncoderReferenceVoltage() { return encoderReferenceVoltage; }

    public void setAngleLimits(double min, double max) {
        minAngle = min;
        maxAngle = max;
    }

    public void updateTargetRotation(double target) {
        targetRotation = target;
        clampTarget();
    }

    public void setTargetFromCameraOffset(double tx, double scaleFactor) {
        updateTargetRotation(totalRotation - tx * scaleFactor);
    }

    public void stop() {
        setPower(0);
    }

    public double getEncoderVoltage() {
        return servoEncoder != null ? servoEncoder.getVoltage() : 0;
    }

    private void clampTarget() {
        if (targetRotation < minAngle) targetRotation = minAngle;
        if (targetRotation > maxAngle) targetRotation = maxAngle;
    }

    private static void busyWait(long nanos) {
        long start = System.nanoTime();
        while (System.nanoTime() - start < nanos) { /* spin */ }
    }

    public synchronized void update() {
        double currentAngle = getCurrentAngle();
        double angleDifference = currentAngle - previousAngle;

        if (angleDifference > 180) {
            angleDifference -= 360;
            cliffs--;
        } else if (angleDifference < -180) {
            angleDifference += 360;
            cliffs++;
        }

        totalRotation = currentAngle - homeAngle + cliffs * 360;
        previousAngle = currentAngle;

        if (!rtp) return;

        double dt = pidTimer.seconds();
        pidTimer.reset();

        if (dt < 0.001 || dt > 1.0) {
            return;
        }

        double error = targetRotation - totalRotation;

        integralSum += error * dt;
        integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));

        final double INTEGRAL_DEADZONE = 2.0;
        if (Math.abs(error) < INTEGRAL_DEADZONE) {
            integralSum *= 0.95;
        }

        double derivative = (error - lastError) / dt;
        lastError = error;

        double pTerm = kP * error;
        double iTerm = kI * integralSum;
        double dTerm = kD * derivative;

        double output = pTerm + iTerm + dTerm;

        final double DEADZONE = 0.5;
        if (Math.abs(error) > DEADZONE) {
            double p = Math.min(maxPower, Math.abs(output)) * Math.signum(output);
            setPower(p);
        } else {
            setPower(0);
        }
    }

    @SuppressLint("DefaultLocale")
    public String log() {
        return String.format(
                "Volts: %.3f | Angle: %.2f | Total: %.2f | Target: %.2f\n" +
                "Power: %.3f | P=%.4f I=%.4f D=%.4f\n" +
                "Error: %.2f | Integral: %.2f",
                getEncoderVoltage(),
                getCurrentAngle(),
                totalRotation,
                targetRotation,
                power,
                kP, kI, kD,
                targetRotation - totalRotation,
                integralSum);
    }
}
