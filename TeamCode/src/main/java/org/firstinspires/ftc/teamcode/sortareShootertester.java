package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;


@TeleOp
@Disabled
public class sortareShootertester extends LinearOpMode {
    private ServoImplEx aruncare,sortare;
    double pos;

    @Override
    public void runOpMode() throws InterruptedException {
        sortare = hardwareMap.get(ServoImplEx.class, "sortare");
        aruncare = hardwareMap.get(ServoImplEx.class, "aruncare");
        aruncare.setPosition(Pozitii.coborare);
        sortare.setPosition(Pozitii.aruncare1);

        waitForStart();
        while (opModeIsActive()) {
            pos = aruncare.getPosition();
            if (gamepad1.a && pos == Pozitii.coborare || pos == 0.5517) {
                sortare.setPosition(Pozitii.aruncare1);
            }
            if (gamepad1.b && pos == Pozitii.coborare || pos == 0.5517) {
                sortare.setPosition(Pozitii.aruncare2);
            }
            if (gamepad1.x && pos == Pozitii.coborare || pos == 0.5517) {
                sortare.setPosition(Pozitii.aruncare3);
            }

            if (gamepad1.dpad_up) {
                aruncare.setPosition(Pozitii.lansare);
            }
            if (gamepad1.dpad_down) {
                aruncare.setPosition(Pozitii.coborare);
            }
            telemetry.addData("pos", pos);
            telemetry.update();
        }
    }
}
