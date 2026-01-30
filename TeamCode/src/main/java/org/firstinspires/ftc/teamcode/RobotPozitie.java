package org.firstinspires.ftc.teamcode;

public class RobotPozitie {
    public static double X = 0;
    public static double Y = 0;
    public static double heading = 0;
    public static int idTag = 0;
    public static int turelaPosition = 0;

    // Drift acumulat de la Limelight (grade) - persistent intre Auto si TeleOp
    public static double accumulatedDrift = 0;

    // Pozitia manuala de centru pentru turela (setata in Auto)
    public static double turelaCenterPos = 0.5;
}
