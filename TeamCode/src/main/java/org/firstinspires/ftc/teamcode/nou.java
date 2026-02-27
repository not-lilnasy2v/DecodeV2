package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
@Disabled

public class nou extends OpMode {

    DcMotorEx scula,shooter1,shooter2;
    ServoImplEx bascula;
    double poz = 0.8 ;

    @Override
    public void init() {

        scula = hardwareMap.get(DcMotorEx.class, "scula");
        bascula = hardwareMap.get(ServoImplEx.class, "bascula");
        shooter1=hardwareMap.get(DcMotorEx.class,"shooter");
        shooter2=hardwareMap.get(DcMotorEx.class, "shooter2");
        scula.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        if(gamepad1.left_bumper) scula.setPower(-1);
        if(gamepad1.right_bumper) scula.setPower(0);

        if (gamepad1.dpad_up && poz < 1) {
            poz+=0.0005;
        }

        if (gamepad1.dpad_down && poz>0) {
            poz-=0.0005;
        }
        bascula.setPosition(poz);
        if(gamepad1.a) {
            shooter1.setPower(1);
            shooter2.setPower(1);
        }
        if(gamepad1.b){
            shooter1.setPower(0);
            shooter2.setPower(0);
        }
        telemetry.addData("Motor Power", power);
        telemetry.addData("Servo Position", poz);
        telemetry.update();
    }
}
