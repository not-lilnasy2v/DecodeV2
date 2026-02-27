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
    public ServoImplEx Saruncare, sortare, bascula;
    public ServoImplExEx turelaD, turelaS, unghiD;
    public DistanceSensor distanta;
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
        scula.setDirection(DcMotorSimple.Direction.REVERSE);
//        Saruncare = hard.get(ServoImplEx.class, "aruncare");
//        Saruncare.setPosition(Pozitii.coborare);
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
        long lastTime = System.currentTimeMillis();
        while (lastTime + t > System.currentTimeMillis()) {

        }
    }



    public void tracks(com.pedropathing.follower.Follower follower,
                       double targetX, double targetY) {
        com.pedropathing.geometry.Pose pose = follower.getPose();
        long now = System.nanoTime();

        double headingRate = 0;
        if (hybridLastTime != 0) {
            double hdt = (now - hybridLastTime) / 1_000_000_000.0;
            if (hdt > 0.005) {
                headingRate = (pose.getHeading() - hybridLastHeading) / hdt;
            }
        }
        hybridLastHeading = pose.getHeading();

        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        double angleToTarget = Math.atan2(dy, dx);
        double turretAngleRad = angleToTarget - pose.getHeading();
        turretAngleRad = normalizeAngle(turretAngleRad);
        double odomAngle = Math.toDegrees(turretAngleRad) * SCALE_FACTOR;
        odomAngle = Math.max(LIMITA_STANGA_GRADE, Math.min(LIMITA_DREAPTA_GRADE, odomAngle));

        LLResult result = limelight.getLatestResult();
        hybridLimelightVede = false;
        double tx = 0;
        if (result != null && result.isValid()) {
            List<FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                tx = result.getTx();
                hybridLimelightVede = true;
            }
        }

        double targetAngle;
        if (!hybridLimelightVede) {
            llIntegral = 0;
            llLastError = 0;
            targetAngle = odomAngle;
        } else {
            double dt = (hybridLastTime == 0) ? 0.01 : (now - hybridLastTime) / 1_000_000_000.0;
            dt = Math.max(0.005, dt);
            double error = -tx * SCALE_FACTOR;
            llIntegral += error * dt;
            llIntegral = Math.max(-H_MAX_INTEGRAL, Math.min(H_MAX_INTEGRAL, llIntegral));
            double derivative = (error - llLastError) / dt;
            llLastError = error;
            double correction = H_KP * error + H_KI * llIntegral + H_KD * derivative;
            correction = Math.max(-H_MAX_CORRECTION, Math.min(H_MAX_CORRECTION, correction));
            double currentAngle = turelaD.getCurrentAngle();
            double baseAngle = H_ODOM_WEIGHT * odomAngle + (1 - H_ODOM_WEIGHT) * currentAngle;
            targetAngle = baseAngle + correction;
            targetAngle = Math.max(LIMITA_STANGA_GRADE, Math.min(LIMITA_DREAPTA_GRADE, targetAngle));
        }
        hybridLastTime = now;

        targetAngle += headingRate * H_FF_GAIN_DEG;
        targetAngle = Math.max(LIMITA_STANGA_GRADE, Math.min(LIMITA_DREAPTA_GRADE, targetAngle));

        turelaTargetGrade = targetAngle;
        double currentAngle = turelaD.getCurrentAngle();
        double posError = targetAngle - currentAngle;
        double posDt = (hybridLastTime == 0) ? 0.01 : (now - hybridLastTime) / 1_000_000_000.0;
        posDt = Math.max(0.005, posDt);
        double posDerivative = (posError - posLastError) / posDt;
        posLastError = posError;

        if (Math.abs(posError) < TURELA_DEADZONE) {
            turelaD.setPosition(0.5);
            turelaS.setPosition(0.5);
            return;
        }
        double power = POS_KP * posError + POS_KD * posDerivative;
        if (Math.abs(power) < POS_MIN_POWER) {
            power = Math.signum(power) * POS_MIN_POWER;
        }
        power = Math.max(-POS_MAX_POWER, Math.min(POS_MAX_POWER, power));
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

    public int detecteazaBiloocaInstant() {
        int mainResult = detectSingleMain();
        if (mainResult != CULOARE_NIMIC) return mainResult;
        return detectBackup();
    }

    private int detectSingleMain() {
        NormalizedRGBA rgba = colors.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsvMain);

        float hue = hsvMain[0];
        float sat = hsvMain[1];
        float val = hsvMain[2];

        if (sat < Pozitii.MIN_SATURATION || val < Pozitii.MIN_VALUE) {
            return CULOARE_NIMIC;
        }

        if (hue >= Pozitii.MAIN_VERDE_HUE_MIN && hue <= Pozitii.MAIN_VERDE_HUE_MAX) {
            return CULOARE_VERDE;
        }
        if (hue >= Pozitii.MAIN_MOV_HUE_MIN && hue <= Pozitii.MAIN_MOV_HUE_MAX) {
            return CULOARE_MOV;
        }

        return CULOARE_NIMIC;
    }

    private int detectBackup() {
        NormalizedRGBA rgba = colorv2.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsvBackup);

        float hue = hsvBackup[0];
        float sat = hsvBackup[1];
        float val = hsvBackup[2];

        if (sat < Pozitii.MIN_SATURATION || val < Pozitii.MIN_VALUE) {
            return CULOARE_NIMIC;
        }

        if (hue >= Pozitii.BACKUP_VERDE_HUE_MIN && hue <= Pozitii.BACKUP_VERDE_HUE_MAX) {
            return CULOARE_VERDE;
        }
        if (hue >= Pozitii.BACKUP_MOV_HUE_MIN && hue <= Pozitii.BACKUP_MOV_HUE_MAX) {
            return CULOARE_MOV;
        }

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
    private double llLastError = 0;
    private double posLastError = 0;
    private double turelaTargetGrade = 0;

    private static final double voltajeNominale = 13.45;

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
        llLastError = 0;
        posLastError = 0;
        turelaTargetGrade = 0;
        hybridLastTime = 0;
        hybridLastHeading = 0;
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

    public boolean isTagVisible() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<FiducialResult> fiducials = result.getFiducialResults();
            return fiducials != null && !fiducials.isEmpty();
        }
        return false;
    }

    private static final double LIMITA_STANGA_GRADE = -261.8;
    private static final double LIMITA_DREAPTA_GRADE = 291.5;
    private static final double REFERINTA_VOLTAJ_D = 0.3690;
    private static final double TURELA_DEADZONE = 2.0;
    private static final double SCALE_FACTOR = 3.074;

    private static final double H_KP = 1.0;
    private static final double H_KI = 0;
    private static final double H_KD = 0.05;
    private static final double H_MAX_INTEGRAL = 30;
    private static final double H_MAX_CORRECTION = 20.0;
    private static final double H_ODOM_WEIGHT = 0.3;
    private static final double H_FF_GAIN_DEG = 1.5;

    private static final double POS_KP = 0.001;
    private static final double POS_KD = 0.000054;
    private static final double POS_MIN_POWER = 0.04;
    private static final double POS_MAX_POWER = 0.15;

    private long hybridLastTime = 0;
    private boolean hybridLimelightVede = false;
    private double hybridLastHeading = 0;
}

