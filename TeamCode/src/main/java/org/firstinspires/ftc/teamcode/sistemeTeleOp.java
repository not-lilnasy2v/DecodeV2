package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.NouHard.ServoImplExEx;

public class sistemeTeleOp {
    public Limelight3A limelight3A;
    public DcMotorEx shooter, intake;
    public ServoImplExEx Saruncare, sortare, unghiD, unghiS,turelaS,turelaD;

    public DistanceSensor distanta;
    public HuskyLens huskyLens;
    public VoltageSensor voltageSensor;

    public final double SkP = 70, SkI = 0.050, SkF = 13.50, SkD = 5;

    public static final int mov = 2;
    public static final int verde = 1;
    public static final int minim_latime = 10;
    public static final int min_inaltime = 10;


    public void initsisteme(HardwareMap hard) {

        turelaS = ServoImplExEx.get(hard, "turelaS");
        turelaD = ServoImplExEx.get(hard, "turelaD");
        turelaS.setCenterPosition(0.5);
        turelaD.setCenterPosition(0.5);

        turelaS.setDegreesPerUnit(180);
        turelaD.setDegreesPerUnit(180);

        turelaS.setMinPosition(0);
        turelaD.setMinPosition(0);
        turelaS.setMaxPosition(1);
        turelaD.setMaxPosition(1);
        turelaS.setPosition(0.5);
        turelaD.setPosition(0.5);

//        turela = hard.get(DcMotorEx.class, "turela");
//        turela.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter = hard.get(DcMotorEx.class, "shooter");
        PIDFCoefficients pid = new PIDFCoefficients(SkP, SkI, SkD, SkF);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        limelight3A = hard.get(Limelight3A.class, "limelight");

        Saruncare = ServoImplExEx.get(hard, "aruncare");
        Saruncare.setPosition(Pozitii.coborare);
        sortare = ServoImplExEx.get(hard, "sortare");
        sortare.setPosition(Pozitii.luarea1);
        unghiD = ServoImplExEx.get(hard, "unghiD");
        unghiS = ServoImplExEx.get(hard, "unghiS");
        unghiS.setPosition(0.1961);
        unghiD.setPosition(0.1961);
        unghiS.setMaxPosition(0.3538);
        unghiD.setMaxPosition(0.3538);



        distanta = hard.get(DistanceSensor.class, "distanta");

        huskyLens = hard.get(HuskyLens.class, "husky");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        intake = hard.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hard.get(VoltageSensor.class, "Control Hub");
    }

    public void kdf(long t) {
        long lastTime = System.currentTimeMillis();
        while (lastTime + t > System.currentTimeMillis()) {
        }
    }


    public int detecteazaBiloaca() {
        HuskyLens.Block[] blocks = huskyLens.blocks();

        HuskyLens.Block blocaj = null;
        int maxArea = 0;

        for (HuskyLens.Block block : blocks) {
            if (block.width >= minim_latime && block.height >= min_inaltime) {
                int aeara = block.width * block.height;
                if (aeara > maxArea) {
                    maxArea = aeara;
                    blocaj = block;
                }
            }
        }


        if (blocaj.id == verde) {
            return 0;
        } else if (blocaj.id == mov) {
            return 1;
        }

        return -1;
    }
}
