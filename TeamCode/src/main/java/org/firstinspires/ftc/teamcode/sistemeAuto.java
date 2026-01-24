package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.NouHard.ServoImplExEx;

import java.util.List;

public class sistemeAuto {
    public DcMotorEx shooter, intake;
    public ServoImplEx Saruncare, sortare;
    public ServoImplExEx turelaD, turelaS,unghiD,unghiS;
    public HuskyLens huskyLens;
    public DistanceSensor distanta;
    public VoltageSensor voltageSensor;
    public Limelight3A limelight;
    public static final int mov = 2;
    public static final int verde = 1;

    public static final int min_latime = 10;
    public static final int min_Inaltime = 10;

    public final double SkP = 70, SkI = 0.050, SkF = 13.50, SkD = 5;
    public int loculete = 3;

    public void initsisteme(HardwareMap hard) {

        shooter = hard.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

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

        unghiS.setPosition(0);
        unghiD.setPosition(0);
        unghiS.setMaxPosition(0.3538);
        unghiD.setMaxPosition(0.3538);

        distanta = hard.get(DistanceSensor.class, "distanta");



        intake = hard.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hard.get(VoltageSensor.class, "Control Hub");

        limelight = hard.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
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

        double turretAngleDeg = Math.toDegrees(turretAngleRad);

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
    public boolean iipusa() {
        return true;
    }
    public int detecteazaBiloaca() {
        HuskyLens.Block[] bloc = huskyLens.blocks();


        HuskyLens.Block celMaiBunBlock = null;
        int maxArea = 0;

        for (HuskyLens.Block block : bloc) {if (block.width >= min_latime && block.height >= min_Inaltime) {
                int area = block.width * block.height;
                if (area > maxArea) {
                    maxArea = area;
                    celMaiBunBlock = block;
                }
            }
        }

        if (celMaiBunBlock.id == verde) {
            return 0;
        } else if (celMaiBunBlock.id == mov) {
            return 1;
        }

        return -1;
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
}

