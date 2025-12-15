package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
@Disabled

public class testare extends LinearOpMode {
    private DcMotorEx shooter;
    public final   double P = 10.23;
    public final double I = 0.0;
    public final  double D = 10.58;
    public final double F = 14.95;


    @Override
    public void runOpMode() throws InterruptedException {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
            shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);

            shooter.setVelocity(-2000);

            telemetry.addData("Velocity", shooter.getVelocity());
            telemetry.update();
        }
    }
}
