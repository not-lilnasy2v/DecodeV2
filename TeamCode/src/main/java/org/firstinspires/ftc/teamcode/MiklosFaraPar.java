package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Miklosita chelioasa")
@Disabled
public class MiklosFaraPar extends LinearOpMode {
    ColorSensor color;
//    NormalizedColorSensor color;
    DistanceSensor distanta;
    Limelight3A limelight3A;

    @Override
    public void runOpMode() {
//        color = hardwareMap.get(NormalizedColorSensor.class, "color");
        color = hardwareMap.get(ColorSensor.class, "color");
        distanta = hardwareMap.get(DistanceSensor.class, "distanta");
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(1);
        limelight3A.start();
//        color.setGain(2.0f);

        int rosu = 0, albastru = 0, verde = 0;
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight3A.getLatestResult();
            if(result.isValid()) {
            }
//NormalizedRGBA colors = color.getNormalizedColors();

            telemetry.addData("red",color.red());
            telemetry.addData("green",color.green());
            telemetry.addData("blue", color.blue());
            telemetry.addLine("===================================");
            telemetry.addData("distanta", distanta.getDistance(DistanceUnit.CM));
            telemetry.update();

        }
    }
}