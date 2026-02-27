package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp

public class servoGasire extends LinearOpMode {
    double pos = 0.5,pos2 = 0.5;
    private ServoImplEx aruncare, sortare, unghiD, unghiS,turelaS,turelaD,bascula;
    private DcMotorEx turela;
//int pos;
    @Override
    public void runOpMode() {

        sortare = hardwareMap.get(ServoImplEx.class, "sortare");
//       aruncare = hardwareMap.get(ServoImplEx.class, "aruncare");
        unghiD = hardwareMap.get(ServoImplEx.class, "unghiD");
        turelaD= hardwareMap.get(ServoImplEx.class, "turelaD");
        turelaS = hardwareMap.get(ServoImplEx.class, "turelaS");
        bascula = hardwareMap.get(ServoImplEx.class, "bascula");
//        turela = hardwareMap.get(DcMotorEx.class, "turela");
//        turela.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
////
//        turela.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        pos= turela.getCurrentPosition();
        waitForStart();
        while (opModeIsActive()) {
//            pos= turela.getCurrentPosition();
            if (gamepad1.a) {
//                turelaS.setPosition(1);
                pos = pos + 0.001;
            }
            if (gamepad1.b) {
                pos =pos -  0.001;
            }
            if (gamepad1.x) {
                pos2 = pos2 + 0.001;
            }
            if (gamepad1.y) {
                pos2 =pos2 -  0.001;
            }

//            aruncare.setPosition(pos);
//            sortare.setPosition(pos);
//            unghiD.setPosition(pos);
//
//            unghiS.setPosition(pos);
            if(pos <= 1.0 || pos >= 0.0) {
                            sortare.setPosition(pos);
//                turelaD.setPosition(pos);
//                turelaS.setPosition(pos);
            }
//
            if(pos2 <= 1.0 || pos2 >= 0.0) {
                bascula.setPosition(pos2);
            }

//            turela.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            turela.setPower(0.1);

            telemetry.addData("pos", pos);
            telemetry.addData("pos2",pos2);
            telemetry.update();
        }

    }
}
