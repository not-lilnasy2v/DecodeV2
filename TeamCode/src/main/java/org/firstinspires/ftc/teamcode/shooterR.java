package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Disabled
public class shooterR extends LinearOpMode {
    private DcMotorEx shooter,turela;

    @Override
    public void runOpMode() throws InterruptedException{
        turela = hardwareMap.get(DcMotorEx.class, "turela");

        turela.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turela.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while(opModeIsActive()){
            if(isStopRequested()) return;

            if(gamepad1.x){
                turela.setPower(0.3);
            }
            if(gamepad1.y){
                turela.setPower(0);
            }
            double pozT = turela.getCurrentPosition();
            telemetry.addData("turela", pozT);
            telemetry.update();
        }
    }
}
