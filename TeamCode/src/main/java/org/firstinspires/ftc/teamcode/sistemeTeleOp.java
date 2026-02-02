package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.NouHard.ServoImplExEx;

public class sistemeTeleOp {
    public DcMotorEx shooter, intake, shooter2;
    public ServoImplExEx Saruncare, sortare, unghiD, unghiS, turelaS, turelaD;

    public DistanceSensor distanta;
    public VoltageSensor voltageSensor;
    public NormalizedColorSensor colors;
    public NormalizedColorSensor colorv2;

    public final double SkP = 80, SkI = 0.050, SkF = 15.50, SkD = 5;

    public static final int CULOARE_VERDE = 0;
    public static final int CULOARE_MOV = 1;
    public static final int CULOARE_NIMIC = -1;

    private final float[] hsvMain = new float[3];
    private final float[] hsvBackup = new float[3];

    private static final int SAMPLES = 3;
    private static final int MIN_VOTES = 2;
    private int lastDetectedColor = CULOARE_NIMIC;
    private long lastDetectionTime = 0;

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

        shooter = hard.get(DcMotorEx.class, "shooter");
        shooter2 = hard.get(DcMotorEx.class, "shooter2");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorEx.Direction.FORWARD);

        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setDirection(DcMotorEx.Direction.REVERSE);

        Saruncare = ServoImplExEx.get(hard, "aruncare");
        Saruncare.setPosition(Pozitii.coborare);
        sortare = ServoImplExEx.get(hard, "sortare");
        sortare.setPosition(Pozitii.luarea1);
        unghiD = ServoImplExEx.get(hard, "unghiD");
        unghiS = ServoImplExEx.get(hard, "unghiS");
        unghiS.setPosition(0.27);
        unghiD.setPosition(0.27);
        unghiD.setMinPosition(0.12);
        unghiS.setMinPosition(0.12);
        unghiS.setMaxPosition(0.41);
        unghiD.setMaxPosition(0.41);


        distanta = hard.get(DistanceSensor.class, "distanta");

        colors = hard.get(NormalizedColorSensor.class, "color");
        colorv2 = hard.get(NormalizedColorSensor.class, "colorv2");

        colors.setGain(Pozitii.COLOR_SENSOR_GAIN);
        colorv2.setGain(Pozitii.COLOR_SENSOR_GAIN);

        if (colors instanceof SwitchableLight) {
            ((SwitchableLight) colors).enableLight(true);
        }
        if (colorv2 instanceof SwitchableLight) {
            ((SwitchableLight) colorv2).enableLight(true);
        }

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
        long now = System.currentTimeMillis();
        if (lastDetectedColor != CULOARE_NIMIC && (now - lastDetectionTime) < 50) {
            return lastDetectedColor;
        }

        int verdeVotes = 0;
        int movVotes = 0;

        for (int i = 0; i < SAMPLES; i++) {
            int mainResult = detectSingleMain();
            int backupResult = detectBackup();

            if (mainResult == CULOARE_VERDE || backupResult == CULOARE_VERDE) verdeVotes++;
            if (mainResult == CULOARE_MOV || backupResult == CULOARE_MOV) movVotes++;
        }

        int result = CULOARE_NIMIC;
        if (verdeVotes >= MIN_VOTES && verdeVotes > movVotes) {
            result = CULOARE_VERDE;
        } else if (movVotes >= MIN_VOTES && movVotes > verdeVotes) {
            result = CULOARE_MOV;
        }

        if (result != CULOARE_NIMIC) {
            lastDetectedColor = result;
            lastDetectionTime = now;
        }

        return result;
    }
    public int detecteazaBiloocaInstant() {
        int mainResult = detectSingleMain();
        if (mainResult != CULOARE_NIMIC) return mainResult;
        return detectBackup();
    }


    private int detectSingleMain() {
        NormalizedRGBA rgba = colors.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsvMain);

        float hue = hsvMain[0];
        float sat = hsvMain[1];
        float val = hsvMain[2];

        if (sat < Pozitii.MIN_SATURATION || val < Pozitii.MIN_VALUE) {
            return CULOARE_NIMIC;
        }

        if (hue >= Pozitii.MAIN_VERDE_HUE_MIN && hue <= Pozitii.MAIN_VERDE_HUE_MAX) {
            return CULOARE_VERDE;
        }
        if (hue >= Pozitii.MAIN_MOV_HUE_MIN && hue <= Pozitii.MAIN_MOV_HUE_MAX) {
            return CULOARE_MOV;
        }

        return CULOARE_NIMIC;
    }

    private int detectBackup() {
        NormalizedRGBA rgba = colorv2.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsvBackup);

        float hue = hsvBackup[0];
        float sat = hsvBackup[1];
        float val = hsvBackup[2];

        if (sat < Pozitii.MIN_SATURATION || val < Pozitii.MIN_VALUE) {
            return CULOARE_NIMIC;
        }

        if (hue >= Pozitii.BACKUP_VERDE_HUE_MIN && hue <= Pozitii.BACKUP_VERDE_HUE_MAX) {
            return CULOARE_VERDE;
        }
        if (hue >= Pozitii.BACKUP_MOV_HUE_MIN && hue <= Pozitii.BACKUP_MOV_HUE_MAX) {
            return CULOARE_MOV;
        }

        return CULOARE_NIMIC;
    }

    public boolean bilaPrezenta(double distantaCM) {
        if (distantaCM < 18.67) return true;

        if (distantaCM < 27) {
            int culoare = detecteazaBiloocaInstant();
            return culoare != CULOARE_NIMIC;
        }

        return false;
    }

    public void resetareDetection() {
        lastDetectedColor = CULOARE_NIMIC;
        lastDetectionTime = 0;
    }
    public float[] MainSensor() {
        NormalizedRGBA rgba = colors.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsvMain);
        return hsvMain;
    }

    public float[] BackUpSensor() {
        NormalizedRGBA rgba = colorv2.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsvBackup);
        return hsvBackup;
    }

    public int UltimaDetectare() {
        return lastDetectedColor;
    }
}
