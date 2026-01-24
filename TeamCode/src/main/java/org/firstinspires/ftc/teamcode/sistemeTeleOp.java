package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.NouHard.ServoImplExEx;

public class sistemeTeleOp {
    public DcMotorEx shooter, intake;
    public ServoImplExEx Saruncare, sortare, unghiD, unghiS, turelaS, turelaD;

    public DistanceSensor distanta;
    public VoltageSensor voltageSensor;
    public NormalizedColorSensor colors;
    public NormalizedColorSensor colorv2;

    public final double SkP = 70, SkI = 0.050, SkF = 13.50, SkD = 5;

    public static final int CULOARE_VERDE = 0;
    public static final int CULOARE_MOV = 1;
    public static final int CULOARE_NIMIC = -1;

    private final float[] hsvMain = new float[3];
    private final float[] hsvBackup = new float[3];

    private static final int SAMPLES = 3;
    private static final int MIN_VOTES = 2;
    private int lastDetectedColor = CULOARE_NIMIC;
    private long lastDetectionTime = 0;
    private static final long DETECTION_CACHE_MS = 50;

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
        PIDFCoefficients pid = new PIDFCoefficients(SkP, SkI, SkD, SkF);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);

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
    public boolean esteBilaPresenta() {
        double distantaCM = distanta.getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM);
        return distantaCM <= Pozitii.DISTANCE_CM;
    }


    /**
     * Detectie RAPIDA cu multi-sampling si voting.
     * Citeste ambii senzori de mai multe ori si foloseste consensul.
     */
    public int detecteazaBiloaca() {
        // Check cache pentru stabilitate
        long now = System.currentTimeMillis();
        if (lastDetectedColor != CULOARE_NIMIC && (now - lastDetectionTime) < DETECTION_CACHE_MS) {
            return lastDetectedColor;
        }

        int verdeVotes = 0;
        int movVotes = 0;

        // Multi-sample rapid - citeste de SAMPLES ori
        for (int i = 0; i < SAMPLES; i++) {
            // Citire paralela ambii senzori
            int mainResult = detectSingleMain();
            int backupResult = detectSingleBackup();

            // Votare
            if (mainResult == CULOARE_VERDE || backupResult == CULOARE_VERDE) verdeVotes++;
            if (mainResult == CULOARE_MOV || backupResult == CULOARE_MOV) movVotes++;
        }

        // Determina culoarea prin consens
        int result = CULOARE_NIMIC;
        if (verdeVotes >= MIN_VOTES && verdeVotes > movVotes) {
            result = CULOARE_VERDE;
        } else if (movVotes >= MIN_VOTES && movVotes > verdeVotes) {
            result = CULOARE_MOV;
        }

        // Update cache
        if (result != CULOARE_NIMIC) {
            lastDetectedColor = result;
            lastDetectionTime = now;
        }

        return result;
    }

    /**
     * Detectie INSTANT - o singura citire, maxim viteza.
     * Foloseste pentru detectie continua in loop rapid.
     */
    public int detecteazaBiloocaInstant() {
        int mainResult = detectSingleMain();
        if (mainResult != CULOARE_NIMIC) return mainResult;
        return detectSingleBackup();
    }

    public int detecteazaBiloacaCuDistanta() {
        if (!esteBilaPresenta()) {
            return CULOARE_NIMIC;
        }
        return detecteazaBiloaca();
    }

    /**
     * Detectie singulara MAIN - optimizata, fara alocare memorie
     */
    private int detectSingleMain() {
        NormalizedRGBA rgba = colors.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsvMain);

        float hue = hsvMain[0];
        float sat = hsvMain[1];
        float val = hsvMain[2];

        // Early exit pentru performanta
        if (sat < Pozitii.MIN_SATURATION || val < Pozitii.MIN_VALUE) {
            return CULOARE_NIMIC;
        }

        // VERDE: 120-170
        if (hue >= Pozitii.MAIN_VERDE_HUE_MIN && hue <= Pozitii.MAIN_VERDE_HUE_MAX) {
            return CULOARE_VERDE;
        }
        // MOV: 220-280
        if (hue >= Pozitii.MAIN_MOV_HUE_MIN && hue <= Pozitii.MAIN_MOV_HUE_MAX) {
            return CULOARE_MOV;
        }

        return CULOARE_NIMIC;
    }

    /**
     * Detectie singulara BACKUP - optimizata, fara alocare memorie
     */
    private int detectSingleBackup() {
        NormalizedRGBA rgba = colorv2.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsvBackup);

        float hue = hsvBackup[0];
        float sat = hsvBackup[1];
        float val = hsvBackup[2];

        // Early exit pentru performanta
        if (sat < Pozitii.MIN_SATURATION || val < Pozitii.MIN_VALUE) {
            return CULOARE_NIMIC;
        }

        // VERDE: 120-170
        if (hue >= Pozitii.BACKUP_VERDE_HUE_MIN && hue <= Pozitii.BACKUP_VERDE_HUE_MAX) {
            return CULOARE_VERDE;
        }
        // MOV: 235-285
        if (hue >= Pozitii.BACKUP_MOV_HUE_MIN && hue <= Pozitii.BACKUP_MOV_HUE_MAX) {
            return CULOARE_MOV;
        }

        return CULOARE_NIMIC;
    }

    /**
     * Reseteaza cache-ul de detectie.
     * Cheama dupa ce bila a fost procesata.
     */
    public void resetDetectionCache() {
        lastDetectedColor = CULOARE_NIMIC;
        lastDetectionTime = 0;
    }
    public float[] getMainSensorHSV() {
        NormalizedRGBA rgba = colors.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsvMain);
        return hsvMain;
    }

    public float[] getBackupSensorHSV() {
        NormalizedRGBA rgba = colorv2.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsvBackup);
        return hsvBackup;
    }

    /**
     * Returneaza ultima culoare detectata din cache.
     */
    public int getLastDetectedColor() {
        return lastDetectedColor;
    }
}
