package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Miklosita chelioasa")
@Disabled
public class MiklosFaraPar extends LinearOpMode {
    ColorSensor color;
    DistanceSensor distanta;
    Limelight3A limelight3A;

    @Override
    public void runOpMode() {
        color = hardwareMap.get(ColorSensor.class, "color");
        distanta = hardwareMap.get(DistanceSensor.class, "distanta");
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(1);
        limelight3A.start();
        int rosu = 0, albastru = 0, verde = 0;
        waitForStart();

        while (opModeIsActive()) {
            rosu = color.red();
            albastru = color.blue();
            verde = color.green();
            LLResult result = limelight3A.getLatestResult();
            if(result.isValid()) {
            }
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.addLine("===================================");
            telemetry.addData("distanta", distanta.getDistance(DistanceUnit.CM));
            telemetry.update();

        }
    }
}