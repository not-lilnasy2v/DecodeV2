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

public class sistemeTeleOp {
    public DcMotorEx turela, shooter, intake;
    public ServoImplEx Saruncare, sortare,unghiD,unghiS;
    public ColorSensor color;
    public DistanceSensor distanta;
    public static final double SkP = 10.23, SkI = 0.0, SkF = 14.95, SkD = 10.58;

    public void initsisteme(HardwareMap hard) {
        shooter = hard.get(DcMotorEx.class, "shooter");
        PIDFCoefficients pid = new PIDFCoefficients(SkP, SkI, SkD, SkF);
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);


        turela = hard.get(DcMotorEx.class, "turela");
        turela.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turela.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        turela.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        Saruncare = hard.get(ServoImplEx.class, "aruncare");
        Saruncare.setPosition(Pozitii.coborare);
        sortare = hard.get(ServoImplEx.class, "sortare");
        sortare.setPosition(Pozitii.luarea1);
        unghiD = hard.get(ServoImplEx.class, "unghiD");
        unghiS = hard.get(ServoImplEx.class, "unghiS");
        unghiD.setPosition(Pozitii.max_jos);
        unghiS.setPosition(Pozitii.max_jos);

        distanta = hard.get(DistanceSensor.class, "distanta");

        color = hard.get(ColorSensor.class, "color");

        intake = hard.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);


    }
    public void kdf(long t) {
        long lastTime = System.currentTimeMillis();
        while (lastTime + t > System.currentTimeMillis()){

        }
    }
}
