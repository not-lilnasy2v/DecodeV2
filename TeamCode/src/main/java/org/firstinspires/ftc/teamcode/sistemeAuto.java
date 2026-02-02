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
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.NouHard.ServoImplExEx;

import java.util.List;

public class sistemeAuto {
    public DcMotorEx shooter, shooter2, intake;
    public ServoImplEx Saruncare, sortare;
    public ServoImplExEx turelaD, turelaS,unghiD,unghiS;
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
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter2 = hard.get(DcMotorEx.class, "shooter2");
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        turelaD = ServoImplExEx.get(hard, "turelaD");
        turelaS = ServoImplExEx.get(hard, "turelaS");
        turelaS.setCenterPosition(0.5);
        turelaD.setCenterPosition(0.5);
        turelaS.setDegreesPerUnit(180);
        turelaD.setDegreesPerUnit(180);
        turelaS.setMinPosition(0);
        turelaD.setMinPosition(0);
        turelaS.setMaxPosition(1);
        turelaD.setMaxPosition(1);
        turelaS.setPosition(0.5);
        turelaD.setPosition(0.5);

        Saruncare = hard.get(ServoImplEx.class, "aruncare");
        Saruncare.setPosition(Pozitii.coborare);
        sortare = hard.get(ServoImplEx.class, "sortare");
        sortare.setPosition(Pozitii.luarea1);
        unghiD = ServoImplExEx.get(hard, "unghiD");
        unghiS = ServoImplExEx.get(hard, "unghiS");

        unghiS.setPosition(0.1961);
        unghiD.setPosition(0.1961);
        unghiD.setMinPosition(0.1485);
        unghiS.setMinPosition(0.1485);
        unghiS.setMaxPosition(0.4829);
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

    public void track(double robotX, double robotY, double robotHeading,
                      double targetX, double targetY) {
        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double angleToTarget = Math.atan2(dy, dx);

        double turretAngleRad = angleToTarget - robotHeading;

        turretAngleRad = normalizeAngle(turretAngleRad);

        double turretAngleDeg = -Math.toDegrees(turretAngleRad);

        double posS = turelaS.angleToPosition(turretAngleDeg);
        double posD = turelaD.angleToPosition(turretAngleDeg);

        turelaS.setPosition(posS);
        turelaD.setPosition(posD);

    }

    public void tracks(com.pedropathing.follower.Follower follower,
                                         double targetX, double targetY) {
        com.pedropathing.geometry.Pose pose = follower.getPose();
        track(pose.getX(), pose.getY(), pose.getHeading(), targetX, targetY);
    }
    public boolean esteBilaPresenta() {
        double distantaCM = distanta.getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM);
        return distantaCM <= Pozitii.DISTANCE_CM;
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

    public int detecteazaBiloacaCuDistanta() {
        if (!esteBilaPresenta()) {
            return CULOARE_NIMIC;
        }
        return detecteazaBiloaca();
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

    public boolean bilaPrezenta(double distantaCM) {
        if (distantaCM < 18.67) return true;
        if (distantaCM < 27) {
            int culoare = detecteazaBiloocaInstant();
            return culoare != CULOARE_NIMIC;
        }
        return false;
    }

    public void resetareDetection() {
        lastDetectedColor = CULOARE_NIMIC;
        lastDetectionTime = 0;
    }

    public float[] MainSensor() {
        NormalizedRGBA rgba = colors.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsvMain);
        return hsvMain;
    }

    public float[] BackUpSensor() {
        NormalizedRGBA rgba = colorv2.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsvBackup);
        return hsvBackup;
    }

    public int UltimaDetectare() {
        return lastDetectedColor;
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

    private double lastTx = 0;
    private double turelaPID_integral = 0;
    private double turelaPID_lastError = 0;
    private static final double TURELA_KP = 0.008;
    private static final double TURELA_KI = 0.0001;
    private static final double TURELA_KD = 0.001;

    public void trackLimelight() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                lastTx = result.getTx();

                double error = lastTx;
                turelaPID_integral += error;
                turelaPID_integral = Math.max(-50, Math.min(50, turelaPID_integral));
                double derivative = error - turelaPID_lastError;
                turelaPID_lastError = error;

                double correction = TURELA_KP * error + TURELA_KI * turelaPID_integral + TURELA_KD * derivative;
                correction = Math.max(-0.15, Math.min(0.15, correction));

                double currentPos = turelaS.getPosition();
                double newPos = currentPos + correction;
                newPos = Math.max(0.0, Math.min(1.0, newPos));

                turelaS.setPosition(newPos);
                turelaD.setPosition(newPos);
            }
        }
    }

    public void stopTurela() {
        turelaS.setPosition(0.5);
        turelaD.setPosition(0.5);
    }

    public void resetTurelaPID() {
        turelaPID_integral = 0;
        turelaPID_lastError = 0;
        lastTx = 0;
    }

    public double getLimelightTx() {
        return lastTx;
    }

    public boolean isTagVisible() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<FiducialResult> fiducials = result.getFiducialResults();
            return fiducials != null && !fiducials.isEmpty();
        }
        return false;
    }
}

