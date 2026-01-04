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
    public final double SkP = 13, SkI = 0.0, SkD = 12,SkF = 14.40;

    public static double posP = 12.0;
    public static double posI = 0.0;
    public static double posD = 0.9;

    public static double velP = 9.0;
    public static double velI = 1.7;
    public static double velD = 6;
    public static double velF = 12.0;

    public static double maxTurretVelocity = 1200;
    public static double TICKS_PER_DEGREE = 1.56;
    public static double MAX_TURRET_ANGLE = 90;
    public static double MIN_TURRET_ANGLE = -90;
    public static double TolerantaPositionest = 1.0;

    public volatile double telem_posError = 0;
    public volatile double telem_targetVelocity = 0;
    public volatile double telem_actualVelocity = 0;
    public volatile double telem_posISum = 0;
    public volatile double telem_targetDeg = 0;
    public volatile double telem_currentDeg = 0;

    public void initsisteme(HardwareMap hard) {
        shooter = hard.get(DcMotorEx.class, "shooter");
        PIDFCoefficients pid = new PIDFCoefficients(SkP, SkI, SkD, SkF);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);

        turela = hard.get(DcMotorEx.class, "turela");
        turela.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turela.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turela.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Saruncare = hard.get(ServoImplEx.class, "aruncare");
        Saruncare.setPosition(Pozitii.coborare);
        sortare = hard.get(ServoImplEx.class, "sortare");
        sortare.setPosition(Pozitii.luarea1);
        unghiD = hard.get(ServoImplEx.class, "unghiD");
        unghiS = hard.get(ServoImplEx.class, "unghiS");
        unghiD.setPosition(0.5);
        unghiS.setPosition(0.5);

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
