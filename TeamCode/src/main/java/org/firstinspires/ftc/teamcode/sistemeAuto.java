package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class sistemeAuto {
    public DcMotorEx turela, shooter, intake;
    public ServoImplEx Saruncare, sortare, unghiD, unghiS;
    public ColorSensor color;
    public DistanceSensor distanta;
    public final double TkP = 0.008, TkI = 0.0, TkD = 0.08, SkP = 10.23, SkI = 0.0, SkF = 14.95, SkD = 10.58;
    public int loculete = 3;
    public Limelight3A limelight3A;
    public boolean IntakeFull = false;
    public int idTag = 0;
    public double tx = 0, power = 0, integral = 0, lastError = 0;

    public void initsisteme(HardwareMap hard) {

        shooter = hard.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);


        turela = hard.get(DcMotorEx.class, "turela");
        turela.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turela.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turela.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        limelight3A = hard.get(Limelight3A.class, "limelight");
        limelight3A.start();
//    }
//
//public synchronized  void tracking(){
//
//        double pos = turela.getCurrentPosition();
//    LLResult result = limelight3A.getLatestResult();
//    if (result.isValid() && pos >= Pozitii.TURRET_MIN_POS && pos <= Pozitii.TURRET_MAX_POS) {
//
//        tx = result.getTx();
//
//        integral += tx;
//        double derivative = tx - lastError;
//
//        power = TkP * tx + TkI * integral + TkD * derivative;
//        turela.setPower(power);
//
//        lastError = tx;
//    }
//
//        turela.setPower(0);
//        lastError = 0;
//        integral = 0;
//    }
    }

    public void kdf(long t) {
        long lastTime = System.currentTimeMillis();
        while (lastTime + t > System.currentTimeMillis()) {

        }
    }
}

