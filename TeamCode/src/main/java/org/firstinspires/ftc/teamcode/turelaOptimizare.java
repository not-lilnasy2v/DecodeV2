package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
@Disabled

public class turelaOptimizare extends LinearOpMode {

    private DcMotorEx turela, frontRight, frontLeft, backLeft, backRight;
    private Follower follower;

    public static double TargetX = 144;
    public static double TargetY = 144;
    public static double TICKS_PER_DEGREE = 1.5;
    public static double MAX_TURRET_ANGLE = 90;
    public static double MIN_TURRET_ANGLE = -90;
    public static double TURRET_POWER = 1;

    private Pose startingPose = new Pose(59, 9, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        turela = hardwareMap.get(DcMotorEx.class, "turela");
        turela.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turela.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turela.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            follower.update();
            Pose currentPose = follower.getPose();
            double robotX = currentPose.getX();
            double robotY = currentPose.getY();
            double robotHeading = currentPose.getHeading();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            frontLeft.setPower(y + x + rx);
            backLeft.setPower(y - x + rx);
            frontRight.setPower(y - x - rx);
            backRight.setPower(y + x - rx);

            double dx = TargetX - robotX;
            double dy = TargetY - robotY;

            double angleToTarget = Math.atan2(dy, dx);

            double turretAngleRad = angleToTarget - robotHeading;

            turretAngleRad = normalizeAngle(turretAngleRad);

            double turretAngleDeg = Math.toDegrees(turretAngleRad);

            turretAngleDeg = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, turretAngleDeg));

            setTurretPosition(turretAngleDeg);


            telemetry.addData("--- TURRET ---", "");
            telemetry.addData("Target Angle (deg)", turretAngleDeg);
            telemetry.addData("Position (ticks)", turela.getCurrentPosition());
            telemetry.addData("--- ROBOT ---", "");
            telemetry.addData("X", robotX);
            telemetry.addData("Y", robotY);
            telemetry.addData("Heading (deg)", Math.toDegrees(robotHeading));
            telemetry.update();
        }
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private void setTurretPosition(double angleDegrees) {
        int targetTicks = (int)(angleDegrees * TICKS_PER_DEGREE);

        turela.setTargetPosition(-targetTicks);
        turela.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turela.setPower(TURRET_POWER);
    }

}

