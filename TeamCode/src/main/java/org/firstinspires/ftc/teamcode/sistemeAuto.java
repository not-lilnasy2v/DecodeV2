package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class sistemeAuto {
    public DcMotorEx turela, shooter, intake;
    public ServoImplEx Saruncare, sortare, unghiD, unghiS;
    public ColorSensor color;
    public DistanceSensor distanta;
    public VoltageSensor voltageSensor;
    public Limelight3A limelight3A;

    public final double SkP = 70, SkI = 0.050, SkF = 13.50, SkD = 5;

    public static double posP = 12.0;
    public static double posI = 0.0;
    public static double posD = 0.9;

    public static double velP = 9.0;
    public static double velI = 1.7;
    public static double velD = 6.0;
    public static double velF = 12.0;

    public static double maxTurretVelocity = 1200;
    public static double TICKS_PER_DEGREE = 1.63;
    public static double MAX_TURRET_ANGLE = 90;
    public static double MIN_TURRET_ANGLE = -90;
    public static double TolerantaPositionest = 1.0;
    private PidControllerAdevarat positionPID;
    public volatile double telem_posError = 0;
    public volatile double telem_targetVelocity = 0;
    public volatile double telem_actualVelocity = 0;
    public volatile double telem_targetDeg = 0;
    public volatile double telem_currentDeg = 0;
    public int loculete = 3;
    public int idTag = 0;

    public void initsisteme(HardwareMap hard) {

        shooter = hard.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);


        turela = hard.get(DcMotorEx.class, "turela");
        turela.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turela.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turela.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        positionPID = new PidControllerAdevarat(posP, posI, posD);
        positionPID.setOutputRange(-maxTurretVelocity, maxTurretVelocity);
        positionPID.setTolerance(TolerantaPositionest * TICKS_PER_DEGREE);
        positionPID.enable();
        turela.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(velP, velI, velD, velF));

        Saruncare = hard.get(ServoImplEx.class, "aruncare");
        Saruncare.setPosition(Pozitii.coborare);
        sortare = hard.get(ServoImplEx.class, "sortare");
        sortare.setPosition(Pozitii.luarea1);
        unghiD = hard.get(ServoImplEx.class, "unghiD");
        unghiS = hard.get(ServoImplEx.class, "unghiS");
        unghiS.setPosition(Pozitii.max_jos);
        unghiD.setPosition(Pozitii.max_jos);


        distanta = hard.get(DistanceSensor.class, "distanta");

        color = hard.get(ColorSensor.class, "color");

        intake = hard.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        limelight3A = hard.get(Limelight3A.class, "limelight");
        limelight3A.start();

        voltageSensor = hard.get(VoltageSensor.class, "Control Hub");
    }

    public void kdf(long t) {
        long lastTime = System.currentTimeMillis();
        while (lastTime + t > System.currentTimeMillis()) {

        }
    }
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    public void trackTargetWithOdometry(double robotX, double robotY, double robotHeading,
                                         double targetX, double targetY) {
        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double angleToTarget = Math.atan2(dy, dx);

        double turretAngleRad = angleToTarget - robotHeading;
        turretAngleRad = normalizeAngle(turretAngleRad);

        double turretDeg = Math.toDegrees(turretAngleRad);
        turretDeg = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, turretDeg));

        double targetTicks = -turretDeg * TICKS_PER_DEGREE;
        double currentTicks = turela.getCurrentPosition();

        telem_targetDeg = turretDeg;
        telem_currentDeg = -currentTicks / TICKS_PER_DEGREE;

        positionPID.setSetpoint(targetTicks);
        double targetVelocity = positionPID.performPID(currentTicks);

        telem_posError = positionPID.getError();
        telem_targetVelocity = targetVelocity;
        telem_actualVelocity = turela.getVelocity();

        if (!positionPID.onTarget()) {
            turela.setVelocity(targetVelocity);
        } else {
            turela.setVelocity(0);
        }
    }


    public void trackTargetWithOdometry(com.pedropathing.follower.Follower follower,
                                         double targetX, double targetY) {
        com.pedropathing.geometry.Pose pose = follower.getPose();
        trackTargetWithOdometry(pose.getX(), pose.getY(), pose.getHeading(), targetX, targetY);
    }

    public void setTurretAngle(double angleDegrees) {
        angleDegrees = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, angleDegrees));

        double targetTicks = -angleDegrees * TICKS_PER_DEGREE;
        double currentTicks = turela.getCurrentPosition();

        telem_targetDeg = angleDegrees;
        telem_currentDeg = -currentTicks / TICKS_PER_DEGREE;

        positionPID.setSetpoint(targetTicks);
        double targetVelocity = positionPID.performPID(currentTicks);

        telem_posError = positionPID.getError();
        telem_targetVelocity = targetVelocity;
        telem_actualVelocity = turela.getVelocity();

        if (!positionPID.onTarget()) {
            turela.setVelocity(targetVelocity);
        } else {
            turela.setVelocity(0);
        }
    }


    public boolean isTurretOnTarget() {
        return positionPID.onTarget();
    }

    public void stopTurret() {
        turela.setVelocity(0);
        positionPID.reset();
        positionPID.enable();
    }

    public void resetTurretPID() {
        positionPID.reset();
        positionPID.enable();
    }
}

