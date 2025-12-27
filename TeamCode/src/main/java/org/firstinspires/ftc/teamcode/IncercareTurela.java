package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class IncercareTurela extends LinearOpMode {

    private DcMotorEx shooter, turela, frontRight, frontLeft, backRight, backLeft;
    private Limelight3A limelight3A;

    private boolean aLast = false, apoz = false;
    private boolean searching = false, wasSearching = false;
    private final double kP = 0.03;
    private double power;
    private final int TURRET_MIN_POS = 309; // de masurat cat si cum e (program care face telemetrie la pozitie dar nu ii da putere la motorul de turela si rotit manual la capetele de cursa)
    private final int TURRET_MAX_POS = 647;
    private int pos;


    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turela = hardwareMap.get(DcMotorEx.class, "turela");
        turela.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turela.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turela.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        limelight3A.start();

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested())
                return;
            pos = turela.getCurrentPosition();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            frontLeft.setPower(y + x + rx);
            backLeft.setPower(y - x + rx);
            frontRight.setPower(y - x - rx);
            backRight.setPower(y + x - rx);

            if (gamepad1.a && !aLast) {
                if (gamepad1.a) {
                    apoz = !apoz;
                    if (apoz) {
                        searching = true;
                    } else {
                        searching = false;
                    }
                }
                aLast = gamepad1.a;
            }

            LLResult result = limelight3A.getLatestResult();

            if (searching && !result.isValid()) {
                if (!wasSearching) {
                    power = 0.1;
                }
                if (pos <= TURRET_MIN_POS && power < 0) {
                    power = 0.1;
                } else if (pos >= TURRET_MAX_POS && power > 0) {
                    power = -0.1;
                }
                telemetry.addData("tracking Mode", "searching");
            } else if (result.isValid()) { // this is actually tracking a found AprilTag
                double tx = result.getTx();
                power = -tx * kP;
                telemetry.addData("tracking Mode", "tracking");
                telemetry.addData("tx", tx);
            } else {
                telemetry.addData("tracking Mode", "tracking off - no AprilTag");
                power = 0;
            }

            wasSearching = searching;
            safeSetPower(turela, power, TURRET_MIN_POS, TURRET_MAX_POS);

            telemetry.addData("pos", pos);
            telemetry.addData("power", power);
            telemetry.addData("searching variable value", searching);
            telemetry.update();
        }
    }

    private void safeSetPower(DcMotor motor, double power, int minPos, int maxPos) {
        int motorPosition = motor.getCurrentPosition();
        if ((motorPosition <= minPos && power < 0) ||
                (motorPosition >= maxPos && power > 0)) {
            motor.setPower(0);
        } else {
            motor.setPower(power);
        }
    }
}
