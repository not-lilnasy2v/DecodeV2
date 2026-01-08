package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.addExact;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


@TeleOp(name = "Main")
@Configurable
public class Tel extends OpMode {
    private PidControllerAdevarat positionPID;
    private DcMotorEx frontRight, frontLeft, backRight, backLeft;
    private Follower  follower;
    boolean stop;
    double sm = 1;
    double max = 0;
    double FL, BL, BR, FR;
    sistemeTeleOp m = new sistemeTeleOp();
    private final double TargetX = 0;
    private final double TargetY = 144;

    private static double voltajeNominale = 12.68;
    private volatile double lastCompensatedF = 0;
    private volatile double lastVoltage = 0;

    private static double maiTare = 0.5;
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
    private double lastRobotX = RobotPozitie.X, lastRobotY = RobotPozitie.Y, lastRobotH = RobotPozitie.heading;
    private double velocityX = 0, velocityY = 0, velocityH = 0;
    private ElapsedTime Timer = new ElapsedTime();
    public boolean turelaTracking = false, tracking = false, Ipornit = false, IntakePornit = false, SortingPornit = false, SortingToggle = false,TrackingLimelight=false,trackingAjutor=false;
    private double distantare, posU;
    int idTag = RobotPozitie.idTag, positiiTurela = RobotPozitie.turelaPosition;
    private volatile boolean[] slotOcupat = new boolean[3];
    private int getLoculete() {
        int count = 0;
        for (boolean occupied : slotOcupat) {
            if (occupied) count++;
        }
        return count;
    }

    private volatile boolean sugere = false;
    private volatile boolean trageShooting = false;
    private final Object blocat = new Object();

    private void applyVoltageCompensatedPIDF() {
        double currentVoltage = m.voltageSensor.getVoltage();
        currentVoltage = Math.max(10.0, Math.min(14.0, currentVoltage));
        double voltageCompensation = voltajeNominale / currentVoltage;
        double compensatedF = m.SkF * voltageCompensation;
        lastCompensatedF = compensatedF;
        lastVoltage = currentVoltage;
        PIDFCoefficients compensatedPID = new PIDFCoefficients(m.SkP, m.SkI, m.SkD, compensatedF);
        m.shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, compensatedPID);
    }

    @Override
    public void init() {
        m.initsisteme(hardwareMap);
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        positionPID = new PidControllerAdevarat(posP, posI, posD);
        positionPID.setOutputRange(-maxTurretVelocity, maxTurretVelocity);
        positionPID.setTolerance(TolerantaPositionest * TICKS_PER_DEGREE);
        positionPID.enable();

        m.turela.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m.turela.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(velP, velI, velD, velF));

        follower = Constants.createFollower(hardwareMap);
        Pose startingPose = new Pose(RobotPozitie.X,RobotPozitie.Y,RobotPozitie.heading);
        follower.setStartingPose(startingPose);
        follower.update();

        applyVoltageCompensatedPIDF();
    }

    public void start() {
        Chassis.start();
        Butoane.start();
        Turela.start();
        Sortare.start();
        Shooter.start();
    }

    private final Thread Butoane = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                posU = m.unghiS.getPosition();

                // Turela toggle
                boolean dpad_right1 = gamepad1.dpad_right;
                if (turelaTracking != dpad_right1) {
                    if (gamepad1.dpad_right) {
                        tracking = !tracking;
                    }
                    turelaTracking = dpad_right1;
                }

                if (gamepad1.dpad_up) {
                    posU += 0.003;
                }
                if (gamepad1.dpad_down) {
                    posU -= 0.003;
                }
                m.unghiD.setPosition(posU);
                m.unghiS.setPosition(posU);

                // Intake toggle
                boolean gamepad1_a = gamepad1.a;
                if (IntakePornit != gamepad1_a) {
                    if (gamepad1.a) {
                        Ipornit = !Ipornit;
                    }
                    IntakePornit = gamepad1_a;
                }

                boolean gamepad2_a = gamepad2.a;
                if (SortingToggle != gamepad2_a) {
                    if (gamepad2.a) {
                        SortingPornit = !SortingPornit;
                        gamepad2.rumble(500);
                    }
                    SortingToggle = gamepad2_a;
                }
            }
        }
    });

    private final Thread Turela = new Thread(new Runnable() {
        @Override
        public void run() {
            Timer.reset();

            while (!stop) {
                positiiTurela = m.turela.getCurrentPosition();

                positionPID.setPID(posP, posI, posD);

                try {
                    m.turela.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                            new PIDFCoefficients(velP, velI, velD, velF));
                } catch (Exception e) {
                }

                if (tracking) {
                    follower.update();
                    Pose currentPose = follower.getPose();

                    double currentX = currentPose.getX();
                    double currentY = currentPose.getY();
                    double currentH = currentPose.getHeading();

                    double dt = Timer.seconds();
                    Timer.reset();
                    if (dt > 0 && dt < 0.1) {
                        velocityX = (currentX - lastRobotX) / dt;
                        velocityY = (currentY - lastRobotY) / dt;
                        velocityH = normalizeAngle(currentH - lastRobotH) / dt;
                    }
                    lastRobotX = currentX;
                    lastRobotY = currentY;
                    lastRobotH = currentH;

                    double predictedX = currentX + velocityX * maiTare;
                    double predictedY = currentY + velocityY * maiTare;
                    double predictedH = currentH + velocityH * maiTare;

                    double dx = TargetX - predictedX;
                    double dy = TargetY - predictedY;

                    double unghiLaTarget = Math.atan2(dy, dx);
                    double turretAngle = unghiLaTarget - predictedH;
                    turretAngle = normalizeAngle(turretAngle);
                    double turretDeg = Math.toDegrees(turretAngle);

                    turretDeg = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, turretDeg));

                    double targetTicks = -turretDeg * TICKS_PER_DEGREE;
                    double currentTicks = m.turela.getCurrentPosition();

                    telem_targetDeg = turretDeg;
                    telem_currentDeg = -currentTicks / TICKS_PER_DEGREE;

                    positionPID.setSetpoint(targetTicks);
                    double targetVelocity = positionPID.performPID(currentTicks);

                    positionPID.setOutputRange(-maxTurretVelocity, maxTurretVelocity);

                    telem_posError = positionPID.getError();
                    telem_targetVelocity = targetVelocity;
                    telem_actualVelocity = m.turela.getVelocity();
                    telem_posISum = positionPID.getISum();

                    if (!positionPID.onTarget()) {
                        ((DcMotorEx) m.turela).setVelocity(targetVelocity);
                    } else {
                        ((DcMotorEx) m.turela).setVelocity(0);
                    }

                } else {
                    m.turela.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    if (gamepad1.left_bumper) {
                        ((DcMotorEx) m.turela).setVelocity(-100);
                    } else if (gamepad1.right_bumper) {
                        ((DcMotorEx) m.turela).setVelocity(100);
                    } else {
                        ((DcMotorEx) m.turela).setVelocity(0);
                    }

                    positionPID.reset();
                    positionPID.enable();
                }
            }
        }
    });

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    private final Thread Sortare = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                synchronized (blocat) {
                    int loculete = getLoculete();

                    if (Ipornit && !trageShooting && loculete < 3 && !gamepad1.b) {
                        sugere = true;
                        m.intake.setPower(1);

                        distantare = m.distanta.getDistance(DistanceUnit.CM);

                        if (distantare < 20) {
                            double servoPos = m.sortare.getPosition();

                            if (Math.abs(servoPos - Pozitii.luarea1) < 0.1 && !slotOcupat[0]) {
                                slotOcupat[0] = true;
                                if (!slotOcupat[1]) {
                                    m.sortare.setPosition(Pozitii.luarea2);
                                } else if (!slotOcupat[2]) {
                                    m.sortare.setPosition(Pozitii.luarea3);
                                }
                                m.kdf(800);
                            } else if (Math.abs(servoPos - Pozitii.luarea2) < 0.1 && !slotOcupat[1]) {
                                slotOcupat[1] = true;
                                if (!slotOcupat[2]) {
                                    m.sortare.setPosition(Pozitii.luarea3);
                                } else if (!slotOcupat[0]) {
                                    m.sortare.setPosition(Pozitii.luarea1);
                                }
                                m.kdf(800);
                            } else if (Math.abs(servoPos - Pozitii.luarea3) < 0.1 && !slotOcupat[2]) {
                                slotOcupat[2] = true;
                                m.kdf(800);
                                gamepad1.rumble(2000);
                            }

                        }
                    } else if(gamepad1.b){
                        sugere = false;
                        m.intake.setPower(-1);
                    }
                    else{
                        sugere = false;
                        m.intake.setPower(0);
                    }
                }
            }
        }
    });

    private enum SortareShooter {
        SIDLE,
        Incarca,
        CautaSiImpuscai,
        GATA,
        RESETTING
    }

    private SortareShooter Sstare = SortareShooter.SIDLE;
    private int[] cPattern = new int[3];

    private final Thread Shooter = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {

                synchronized (blocat) {
                    int loculete = getLoculete();

                    if (gamepad1.y && loculete > 0 && !sugere) {
                        trageShooting = true;
                        applyVoltageCompensatedPIDF();
                        m.shooter.setVelocity(1750);

                        if (SortingPornit) {
                            switch (Sstare) {
                                case SIDLE:
                                    if (slotOcupat[0]) {
                                        m.sortare.setPosition(Pozitii.aruncare1);
                                    } else if (slotOcupat[1]) {
                                        m.sortare.setPosition(Pozitii.aruncare2);
                                    } else if (slotOcupat[2]) {
                                        m.sortare.setPosition(Pozitii.aruncare3);
                                    }
                                    m.kdf(250);
                                    Sstare = SortareShooter.Incarca;
                                    break;

                                case Incarca:
                                    if (idTag == 3) {
                                        //  purple, purple, green
                                        cPattern[0] = 1;
                                        cPattern[1] = 1;
                                        cPattern[2] = 0;
                                    } else if (idTag == 2) {
                                        //  purple, green, purple
                                        cPattern[0] = 1;
                                        cPattern[1] = 0;
                                        cPattern[2] = 1;
                                    } else if (idTag == 1) {
                                        //  green, purple, purple
                                        cPattern[0] = 0;
                                        cPattern[1] = 1;
                                        cPattern[2] = 1;
                                    } else {
                                        Sstare = SortareShooter.GATA;
                                        break;
                                    }

                                    Sstare = SortareShooter.CautaSiImpuscai;
                                    break;

                                case CautaSiImpuscai:
                                    BilaInPattern();
                                    Sstare = SortareShooter.RESETTING;
                                    break;

                                case RESETTING:
                                    m.sortare.setPosition(Pozitii.luarea1);
                                    m.kdf(300);
                                    Sstare = SortareShooter.GATA;
                                    break;

                                case GATA:
                                    m.shooter.setVelocity(950);
                                    trageShooting = false;
                                    Sstare = SortareShooter.SIDLE;
                                    break;
                            }
                        } else {
                            rapidFireShoot();
                        }
                    } else {
                        if (Sstare != SortareShooter.SIDLE) {
                            m.shooter.setVelocity(950);
                            m.sortare.setPosition(Pozitii.luarea1);
                            trageShooting = false;
                            Sstare = SortareShooter.SIDLE;
                        }
                    }
                }
            }
        }

        private void rapidFireShoot() {
            applyVoltageCompensatedPIDF();  
            m.kdf(250);
            if (getLoculete() > 0 && Sstare == SortareShooter.SIDLE) {
                Sstare = SortareShooter.Incarca;
            }

            double lastPos = m.sortare.getPosition();

            for (int i = 2; i >= 0; i--) {
                if (slotOcupat[i]) {
                    double target = getTarget(i);

                    m.sortare.setPosition(target);

                    double dist = Math.abs(target - lastPos);
                    int moveWait = (int)(dist * 550) + 80;
                    m.kdf(moveWait);

                    m.Saruncare.setPosition(Pozitii.lansare);
                    m.kdf(100);

                    m.Saruncare.setPosition(Pozitii.coborare);
                    m.kdf(80);

                    slotOcupat[i] = false;
                    lastPos = target;
                }
            }

            m.sortare.setPosition(Pozitii.luarea1);
            m.kdf(150);
            m.shooter.setVelocity(950);
            trageShooting = false;
            Sstare = SortareShooter.SIDLE;
        }

        private double getTarget(int slot) {
            if (slot == 0) return Pozitii.aruncare1;
            if (slot == 1) return Pozitii.aruncare2;
            return Pozitii.aruncare3;
        }

        private void BilaInPattern() {
            int loculete = getLoculete();
            for (int step = 0; step < 3 && loculete > 0; step++) {
                int need = cPattern[step];
                boolean ballShot = false;

                if (!ballShot && slotOcupat[0]) {
                    m.sortare.setPosition(Pozitii.aruncare1);
                    m.kdf(950);

                    boolean mov = m.color.green() <= Pozitii.mov_verde;

                    if ((need == 1 && mov) || (need == 0 && !mov)) {
                        TrageBila();
                        slotOcupat[0] = false;
                        loculete--;
                        ballShot = true;
                    }
                }

                if (!ballShot && slotOcupat[1]) {
                    m.sortare.setPosition(Pozitii.aruncare2);
                    m.kdf(950);

                    boolean mov = m.color.green() <= Pozitii.mov_verde;

                    if ((need == 1 && mov) || (need == 0 && !mov)) {
                        TrageBila();
                        slotOcupat[1] = false;
                        loculete--;
                        ballShot = true;
                    }
                }

                if (!ballShot && slotOcupat[2]) {
                    m.sortare.setPosition(Pozitii.aruncare3);
                    m.kdf(950);

                    boolean mov = m.color.green() <= Pozitii.mov_verde;

                    if ((need == 1 && mov) || (need == 0 && !mov)) {
                        TrageBila();
                        slotOcupat[2] = false;
                        loculete--;
                        ballShot = true;
                    }
                }

                if (!ballShot) {
                    for (int i = 0; i < 3; i++) {
                        if (slotOcupat[i]) {
                            if (i == 0) m.sortare.setPosition(Pozitii.aruncare1);
                            else if (i == 1) m.sortare.setPosition(Pozitii.aruncare2);
                            else m.sortare.setPosition(Pozitii.aruncare3);

                            m.kdf(1300);
                            TrageBila();
                            slotOcupat[i] = false;
                            loculete--;
                            break;
                        }
                    }
                }
            }
        }

        private void TrageBila() {
            m.Saruncare.setPosition(Pozitii.lansare);
            m.kdf(150);
            m.Saruncare.setPosition(Pozitii.coborare);
            m.kdf(150);
        }
    });

    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;

                FL = y + x + rx;
                BL = y - x + rx;
                FR = y - x - rx;
                BR = y + x - rx;

                max = abs(FL);
                if (abs(FR) > max) max = abs(FR);
                if (abs(BL) > max) max = abs(BL);
                if (abs(BR) > max) max = abs(BR);

                if (max > 1) {
                    FL /= max;
                    FR /= max;
                    BL /= max;
                    BR /= max;
                }

                if (gamepad1.right_trigger > 0) {
                    sm = 2;
                } else if (gamepad1.left_trigger > 0) {
                    sm = 5;
                } else {
                    sm = 1;
                }

                POWER(FR / sm, BL / sm, BR / sm, FL / sm);

            }
        }
    });

    public void stop() {
        stop = true;
    }

    @Override
    public void loop(){
    }
    public void POWER(double fr1, double bl1, double br1, double fl1) {
        frontRight.setPower(fr1);
        backLeft.setPower(bl1);
        frontLeft.setPower(fl1);
        backRight.setPower(br1);
    }
}