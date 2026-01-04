package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Shooter PID")
@Configurable
public class testare extends LinearOpMode {
    private DcMotorEx shooter;
    public static double P = 13;
    public static double I = 0.0;
    public static double D = 12;
    public static double F = 14.40;


    @Override
    public void runOpMode() throws InterruptedException {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
            shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
            if(gamepad1.a) {
                shooter.setVelocity(1800);
            }if(gamepad1.x){
                shooter.setVelocity(0);
            }
            telemetry.addData("Velocity", shooter.getVelocity());
            telemetry.update();
        }
    }
}
