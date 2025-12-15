package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class servoGasire extends LinearOpMode {
//    double pos=0.5;
    private ServoImplEx aruncare, sortare, unghiD, unghiS;
    private DcMotorEx turela;
int pos = 0;
    @Override
    public void runOpMode() {

//        sortare = hardwareMap.get(ServoImplEx.class, "sortare");
//       aruncare = hardwareMap.get(ServoImplEx.class, "aruncare");
//        unghiD = hardwareMap.get(ServoImplEx.class, "unghiD");
//        unghiS = hardwareMap.get(ServoImplEx.class, "unghiS");
        turela = hardwareMap.get(DcMotorEx.class, "turela");
        turela.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        turela.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pos = turela.getCurrentPosition();

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                pos += 1;
            }
            if (gamepad1.b) {
                pos -= 1;
            }

//            aruncare.setPosition(pos);
//            sortare.setPosition(pos);
//            unghiD.setPosition(pos);
//            unghiS.setPosition(pos);

            turela.setTargetPosition(pos);
            turela.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turela.setPower(0.1);

            telemetry.addData("pos", pos);
            telemetry.update();
        }

    }
}
