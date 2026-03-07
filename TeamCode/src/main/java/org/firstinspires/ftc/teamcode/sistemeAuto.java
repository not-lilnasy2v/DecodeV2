package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.NouHard.ServoImplExEx;

import java.util.List;

public class sistemeAuto {
    public DcMotorEx shooter, shooter2, intake, scula;
    public ServoImplEx sortare, bascula;
    public ServoImplExEx turelaD, turelaS, unghiD;
    public DistanceSensor distanta;
    public volatile double cachedDistanta = 819.0;
    public VoltageSensor voltageSensor;
    public Limelight3A limelight;
    public NormalizedColorSensor colors;
    public NormalizedColorSensor colorv2;

    public static final int CULOARE_VERDE = 0;
    public static final int CULOARE_MOV = 1;
    public static final int CULOARE_NIMIC = -1;

    private final float[] hsvMain = new float[3];
    private final float[] hsvBackup = new float[3];

    private static final int SAMPLES = 3;
    private static final int MIN_VOTES = 2;
    private int lastDetectedColor = CULOARE_NIMIC;
    private long lastDetectionTime = 0;
    private static final long DETECTION_CACHE_MS = 50;

    public final double SkP = 80, SkI = 0.050, SkF = 15.50, SkD = 5;
//    public final double SkPS = 70, SkIS = 0.050, SkFS= 13.50, SkDS = 5;


    public void initsisteme(HardwareMap hard) {
        shooter = hard.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter2 = hard.get(DcMotorEx.class, "shooter2");
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        turelaD = ServoImplExEx.getContinuous(hard, "turelaD", "turretD");
        turelaD.setEncoderReferenceVoltage(REFERINTA_VOLTAJ_D);
        turelaD.setAngleLimits(LIMITA_STANGA_GRADE, LIMITA_DREAPTA_GRADE);
        turelaD.setDeadzone(TURELA_DEADZONE);
        turelaD.setPosition(0.5);

        turelaS = ServoImplExEx.getContinuous(hard, "turelaS", "turretS");
        turelaS.setPosition(0.5);

        bascula = hard.get(ServoImplEx.class, "bascula");
        bascula.setPosition(Pozitii.sede);

        scula = hard.get(DcMotorEx.class, "scula");
        scula.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sortare = hard.get(ServoImplEx.class, "sortare");
        sortare.setPosition(Pozitii.luarea1);
        unghiD = ServoImplExEx.get(hard, "unghiD");
        unghiD.setPosition(0.1961);
        unghiD.setMinPosition(0.1485);
        unghiD.setMaxPosition(0.4829);

        distanta = hard.get(DistanceSensor.class, "distanta");

        colors = hard.get(NormalizedColorSensor.class, "color");
        colorv2 = hard.get(NormalizedColorSensor.class, "colorv2");

        colors.setGain(Pozitii.COLOR_SENSOR_GAIN);
        colorv2.setGain(Pozitii.COLOR_SENSOR_GAIN);

        if (colors instanceof SwitchableLight) {
            ((SwitchableLight) colors).enableLight(true);
        }
        if (colorv2 instanceof SwitchableLight) {
            ((SwitchableLight) colorv2).enableLight(true);
        }

        intake = hard.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);


        voltageSensor = hard.get(VoltageSensor.class, "Control Hub");

        limelight = hard.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void kdf(long t) {
        try { Thread.sleep(t); } catch (InterruptedException ignored) {}
    }



    public void tracks(com.pedropathing.follower.Follower follower,
                       double targetX, double targetY) {
        com.pedropathing.geometry.Pose pose = follower.getPose();
        long now = System.nanoTime();

        // Prima iteratie dupa reset - doar inregistreaza si iese
        if (hybridLastTime == 0) {
            hybridLastTime = now;
            hybridLastHeading = pose.getHeading();
            lastPoseX = pose.getX();
            lastPoseY = pose.getY();
            return;
        }

        double dt = (now - hybridLastTime) / 1_000_000_000.0;
        dt = Math.max(0.005, Math.min(0.1, dt));
        hybridLastTime = now;

        // Heading rate filtrat + normalizat
        double headingDiff = pose.getHeading() - hybridLastHeading;
        headingDiff = normalizeAngle(headingDiff);
        hybridLastHeading = pose.getHeading();
        double rawHR = headingDiff / dt;
        filteredHeadingRate += HR_FILTER * (rawHR - filteredHeadingRate);

        // Velocity filtrat
        xVelocity += VEL_FILTER * ((pose.getX() - lastPoseX) / dt - xVelocity);
        yVelocity += VEL_FILTER * ((pose.getY() - lastPoseY) / dt - yVelocity);
        lastPoseX = pose.getX();
        lastPoseY = pose.getY();

        // Odometry angle cu velocity prediction
        double predictedX = pose.getX() + xVelocity * VEL_LEAD_TIME;
        double predictedY = pose.getY() + yVelocity * VEL_LEAD_TIME;
        double dx = targetX - predictedX;
        double dy = targetY - predictedY;
        double distToTarget = Math.hypot(dx, dy);
        double angleToTarget = Math.atan2(dy, dx);
        double turretRad = normalizeAngle(angleToTarget - pose.getHeading());
        double odomAngle = Math.toDegrees(turretRad) * SCALE_FACTOR;
        odomAngle = clamp(odomAngle, LIMITA_STANGA_GRADE, LIMITA_DREAPTA_GRADE);

        // Translational feedforward
        double tangentialVel = xVelocity * Math.sin(angleToTarget) - yVelocity * Math.cos(angleToTarget);
        double bearingRate = tangentialVel / Math.max(distToTarget, MIN_DIST);
        double translationalFF = Math.toDegrees(bearingRate) * SCALE_FACTOR * TRANS_FF_GAIN;

        // Limelight - hybrid offset (ca in TeleOp)
        LLResult result = limelight.getLatestResult();
        hybridLimelightVede = false;
        if (result != null && result.isValid()) {
            List<FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                double tx = result.getTx();
                hybridLimelightVede = true;
                double currentAngle = turelaD.getCurrentAngle();
                double angleAtCapture = currentAngle - Math.toDegrees(filteredHeadingRate) * LL_LATENCY * SCALE_FACTOR;
                double trueTarget = angleAtCapture - tx * SCALE_FACTOR;
                double offsetError = trueTarget - odomAngle;
                llOffset += ALPHA * (offsetError - llOffset);
                double rawOutput = POS_KP * (odomAngle + llOffset - currentAngle);
                if (Math.abs(rawOutput) < POS_MAX_POWER) {
                    llIntegral += offsetError * dt;
                    llIntegral = clamp(llIntegral, -H_MAX_INTEGRAL, H_MAX_INTEGRAL);
                }
                llOffset = clamp(llOffset, -MAX_OFFSET, MAX_OFFSET);
            }
        }
        if (!hybridLimelightVede) {
            llOffset *= DECAY;
            llIntegral *= 0.95;
        }

        // Target angle
        double targetAngle = odomAngle + llOffset + llIntegral * LL_KI + TURELA_OFFSET_DEG;
        targetAngle += filteredHeadingRate * H_FF_GAIN_DEG + translationalFF;
        targetAngle = clamp(targetAngle, LIMITA_STANGA_GRADE, LIMITA_DREAPTA_GRADE);

        // Smoothed target cu adaptive alpha
        double currentAngle = turelaD.getCurrentAngle();
        double preError = Math.abs(targetAngle - currentAngle);
        double adaptiveAlpha;
        if (preError > DISTURBANCE_THRESHOLD) adaptiveAlpha = 0.95;
        else if (preError > 5.0) adaptiveAlpha = 0.85;
        else adaptiveAlpha = T_ALPHA;

        if (!trackingInitialized) {
            smoothedTarget = targetAngle;
            trackingInitialized = true;
        } else {
            smoothedTarget += adaptiveAlpha * (targetAngle - smoothedTarget);
        }

        // Position PID cu measurement-based derivative
        turelaTargetGrade = smoothedTarget;
        double posError = smoothedTarget - currentAngle;
        double posDerivative = -(currentAngle - prevMeasurement) / dt;
        prevMeasurement = currentAngle;

        double absError = Math.abs(posError);
        if (absError < TURELA_DEADZONE) {
            turelaD.setPosition(0.5);
            turelaS.setPosition(0.5);
            return;
        }

        double effectiveMaxPower;
        if (absError > 20.0) effectiveMaxPower = BOOST_MAX_POWER;
        else if (absError > 10.0) effectiveMaxPower = POS_MAX_POWER + 0.05;
        else effectiveMaxPower = POS_MAX_POWER;

        double power = POS_KP * posError + POS_KD * posDerivative;
        if (absError < TURELA_DEADZONE * 2) {
            double ramp = (absError - TURELA_DEADZONE) / TURELA_DEADZONE;
            power *= Math.max(0, ramp);
        }
        if (Math.abs(power) < POS_MIN_POWER && absError > TURELA_DEADZONE) {
            power = Math.signum(power) * POS_MIN_POWER;
        }
        power = clamp(power, -effectiveMaxPower, effectiveMaxPower);
        turelaD.setPosition(0.5 - power);
        turelaS.setPosition(0.5 - power);
    }

    public int detecteazaBiloaca() {
        long now = System.currentTimeMillis();
        if (lastDetectedColor != CULOARE_NIMIC && (now - lastDetectionTime) < DETECTION_CACHE_MS) {
            return lastDetectedColor;
        }

        int verdeVotes = 0;
        int movVotes = 0;

        for (int i = 0; i < SAMPLES; i++) {
            int mainResult = detectSingleMain();
            int backupResult = detectBackup();

            if (mainResult == CULOARE_VERDE || backupResult == CULOARE_VERDE) verdeVotes++;
            if (mainResult == CULOARE_MOV || backupResult == CULOARE_MOV) movVotes++;
        }

        int result = CULOARE_NIMIC;
        if (verdeVotes >= MIN_VOTES && verdeVotes > movVotes) {
            result = CULOARE_VERDE;
        } else if (movVotes >= MIN_VOTES && movVotes > verdeVotes) {
            result = CULOARE_MOV;
        }

        if (result != CULOARE_NIMIC) {
            lastDetectedColor = result;
            lastDetectionTime = now;
        }

        return result;
    }


    private int detectSingleMain() {
        NormalizedRGBA rgba = colors.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsvMain);

        float hue = hsvMain[0];
        float sat = hsvMain[1];
        float val = hsvMain[2];
        if (sat < Pozitii.MIN_SATURATION || val < Pozitii.MIN_VALUE) return CULOARE_NIMIC;
        if (hue >= Pozitii.MAIN_VERDE_HUE_MIN && hue <= Pozitii.MAIN_VERDE_HUE_MAX) return CULOARE_VERDE;
        if (hue >= Pozitii.MAIN_MOV_HUE_MIN && hue <= Pozitii.MAIN_MOV_HUE_MAX) return CULOARE_MOV;
        return CULOARE_NIMIC;
    }

    private int detectBackup() {
        NormalizedRGBA rgba = colorv2.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsvBackup);
        float hue = hsvBackup[0];
        float sat = hsvBackup[1];
        float val = hsvBackup[2];
        if (sat < Pozitii.MIN_SATURATION || val < Pozitii.MIN_VALUE) return CULOARE_NIMIC;
        if (hue >= Pozitii.BACKUP_VERDE_HUE_MIN && hue <= Pozitii.BACKUP_VERDE_HUE_MAX) return CULOARE_VERDE;
        if (hue >= Pozitii.BACKUP_MOV_HUE_MIN && hue <= Pozitii.BACKUP_MOV_HUE_MAX) return CULOARE_MOV;
        return CULOARE_NIMIC;
    }

    public void resetareDetection() {
        lastDetectedColor = CULOARE_NIMIC;
        lastDetectionTime = 0;
    }

    public int detectIdTag() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                return fiducials.get(0).getFiducialId();
            }
        }
        return 0;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double llIntegral = 0;
    private double llOffset = 0;
    private double turelaTargetGrade = 0;
    private double filteredHeadingRate = 0;
    private double smoothedTarget = 0;
    private boolean trackingInitialized = false;
    private double prevMeasurement = 0;

    private static final double voltajeNominale = 12.85;

    public void applyVoltageCompensatedPIDF() {
        double currentVoltage = voltageSensor.getVoltage();
        currentVoltage = Math.max(9.0, Math.min(14.0, currentVoltage));
        double voltageCompensation = voltajeNominale / currentVoltage;
        double compensatedF = SkF * voltageCompensation;
        PIDFCoefficients compensatedPID = new PIDFCoefficients(SkP, SkI, SkD, compensatedF);
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, compensatedPID);
        shooter2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, compensatedPID);
    }

    public void resetTurelaPID() {
        llIntegral = 0;
        llOffset = 0;
        turelaTargetGrade = 0;
        filteredHeadingRate = 0;
        smoothedTarget = 0;
        trackingInitialized = false;
        prevMeasurement = 0;
        hybridLastTime = 0;
        hybridLastHeading = 0;
        xVelocity = 0;
        yVelocity = 0;
        lastPoseX = 0;
        lastPoseY = 0;
    }

    public double getLimelightTx() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                return result.getTx();
            }
        }
        return 0;
    }

    public void resetLimelightCorrection() {
        llOffset = 0;
        llIntegral = 0;
    }

    public double getTurelaError() {
        return Math.abs(turelaTargetGrade - turelaD.getCurrentAngle());
    }

    public boolean isTagVisible() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<FiducialResult> fiducials = result.getFiducialResults();
            return fiducials != null && !fiducials.isEmpty();
        }
        return false;
    }

    private static final double LIMITA_STANGA_GRADE = -219.9;
    private static final double LIMITA_DREAPTA_GRADE = 218.3;
    private static final double REFERINTA_VOLTAJ_D = 0.3730;
    private static final double TURELA_DEADZONE = 2.0;
    private static final double SCALE_FACTOR = 2.435;

    private static final double ALPHA = 0.25;
    private static final double MAX_OFFSET = 35.0;
    private static final double DECAY = 0.97;
    private static final double H_FF_GAIN_DEG = 1.2;
    private static final double HR_FILTER = 0.30;
    private static final double VEL_FILTER = 0.22;
    private static final double VEL_LEAD_TIME = 0.2;
    private static final double T_ALPHA = 0.45;
    private static final double LL_LATENCY = 0.02;
    private static final double LL_KI = 0.03;
    private static final double H_MAX_INTEGRAL = 15.0;
    private static final double TRANS_FF_GAIN = 0.2;
    private static final double MIN_DIST = 12.0;
    private static final double DISTURBANCE_THRESHOLD = 17.0;
    private static final double TURELA_OFFSET_DEG = 3.0;

    private static final double POS_KP = 0.0025;
    private static final double POS_KD = 0.00010;
    private static final double POS_MIN_POWER = 0.05;
    private static final double POS_MAX_POWER = 0.45;
    private static final double BOOST_MAX_POWER = 0.55;

    private long hybridLastTime = 0;
    private boolean hybridLimelightVede = false;
    private double hybridLastHeading = 0;
    private double lastPoseX = 0, lastPoseY = 0;
    private double xVelocity = 0, yVelocity = 0;

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}

