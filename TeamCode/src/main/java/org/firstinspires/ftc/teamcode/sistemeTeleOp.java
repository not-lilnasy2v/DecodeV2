package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class sistemeTeleOp {
    public DcMotorEx turela, shooter, intake;
    public ServoImplEx Saruncare, sortare,unghiD,unghiS;
    public ColorSensor color;
    public DistanceSensor distanta;
    public VoltageSensor voltageSensor;
    public final double  SkP = 70, SkI = 0.050, SkF = 13.50, SkD = 5;


    public void initsisteme(HardwareMap hard) {
        shooter = hard.get(DcMotorEx.class, "shooter");
        PIDFCoefficients pid = new PIDFCoefficients(SkP, SkI, SkD, SkF);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);

        turela = hard.get(DcMotorEx.class, "turela");
        turela.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turela.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Saruncare = hard.get(ServoImplEx.class, "aruncare");
        Saruncare.setPosition(Pozitii.coborare);
        sortare = hard.get(ServoImplEx.class, "sortare");
        sortare.setPosition(Pozitii.luarea1);
        unghiD = hard.get(ServoImplEx.class, "unghiD");
        unghiS = hard.get(ServoImplEx.class, "unghiS");


        distanta = hard.get(DistanceSensor.class, "distanta");

        color = hard.get(ColorSensor.class, "color");

        intake = hard.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hard.get(VoltageSensor.class, "Control Hub");
    }
    public void kdf(long t) {
        long lastTime = System.currentTimeMillis();
        while (lastTime + t > System.currentTimeMillis()){

        }
    }
}
