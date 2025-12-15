package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp

public class turelaOptimizare extends LinearOpMode {
    private DcMotorEx turela;

    private Limelight3A limelight3A;
    private final int turela_min = 309, turela_init = 490, turela_max = 647;
    private double integral = 0, lastError = 0, tx = 0, power,pos;
    private final double p = 0.008, i = 0.0, d = 0.08;
    private boolean pornit = false, aolo = false;

    @Override
    public void runOpMode() throws InterruptedException {
        turela = hardwareMap.get(DcMotorEx.class, "turela");
        turela.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        limelight3A.start();

        pos = turela.getCurrentPosition();
        if(pos != turela_init) {
            turela.setTargetPosition(turela_init);
            turela.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            turela.setPower(0.03);
            turela.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        else turela.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            if (isStopRequested()) return;

            boolean tracking = gamepad1.a;
            if (tracking != pornit) {
                if (gamepad1.a) {
                    aolo = !aolo;
                }
                tracking = pornit;
            }
            if (aolo && limelight3A.isRunning()) {
                do {
                    LLResult result = limelight3A.getLatestResult();
                    if (result.isValid() && pos >= turela_min || pos <= turela_max) {
                        tx = result.getTx();
                        double error = tx;
                        integral += error;
                        double derivative = error - lastError;

                        power = p * error + i * integral + d * derivative;

                        turela.setPower(power);

                        lastError = error;

                    }else if (pos <= turela_min && power < 0 && !result.isValid()) {
                        power = 0.1;
                    } else if (pos >= turela_max && power > 0 && !result.isValid()) {
                        power = -0.1;
                    }
                } while (Math.abs(tx) > 0.05);
            }else turela.setPower(0);
        }
    }
}
