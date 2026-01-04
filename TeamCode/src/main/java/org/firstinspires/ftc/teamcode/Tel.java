package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

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
    private Follower follower;
    boolean stop;
    double sm = 1;
    double max = 0;
    double FL, BL, BR, FR;
    sistemeTeleOp m = new sistemeTeleOp();
    private final double TargetX = 0;
    private final double TargetY = 144;

    private static double maiTare = 0.5;  // Prediction lookahead for turret
    private double lastRobotX = RobotPozitie.X, lastRobotY = RobotPozitie.Y, lastRobotH = RobotPozitie.heading;
    private double velocityX = 0, velocityY = 0, velocityH = 0;
    private ElapsedTime Timer = new ElapsedTime();
    public boolean turelaTracking = false, tracking = false, Ipornit = false, IntakePornit = false, SortingPornit = false, SortingToggle = false,TrackingLimelight=false,trackingAjutor=false;
    private double distantare, posU;
    int idTag = RobotPozitie.idTag, positiiTurela;
    private volatile boolean[] slotOcupat = new boolean[3];
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

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

        positionPID = new PidControllerAdevarat(m.posP, m.posI, m.posD);
        positionPID.setOutputRange(-m.maxTurretVelocity, m.maxTurretVelocity);
        positionPID.setTolerance(m.TolerantaPositionest * m.TICKS_PER_DEGREE);
        positionPID.enable();

        m.turela.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m.turela.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(m.velP, m.velI, m.velD, m.velF));

        follower = Constants.createFollower(hardwareMap);
        Pose startingPose = new Pose(RobotPozitie.X,RobotPozitie.Y,RobotPozitie.heading);
        follower.setStartingPose(startingPose);
        follower.update();
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

                positionPID.setPID(m.posP, m.posI, m.posD);

                try {
                    m.turela.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                            new PIDFCoefficients(m.velP, m.velI, m.velD, m.velF));
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

                    turretDeg = Math.max(m.MIN_TURRET_ANGLE, Math.min(m.MAX_TURRET_ANGLE, turretDeg));

                    double targetTicks = -turretDeg * m.TICKS_PER_DEGREE;
                    double currentTicks = m.turela.getCurrentPosition();

                    m.telem_targetDeg = turretDeg;
                    m.telem_currentDeg = -currentTicks / m.TICKS_PER_DEGREE;

                    positionPID.setSetpoint(targetTicks);
                    double targetVelocity = positionPID.performPID(currentTicks);

                    positionPID.setOutputRange(-m.maxTurretVelocity, m.maxTurretVelocity);

                    m.telem_posError = positionPID.getError();
                    m.telem_targetVelocity = targetVelocity;
                    m.telem_actualVelocity = m.turela.getVelocity();
                    m.telem_posISum = positionPID.getISum();

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
                        if(gamepad2.x){
                            m.shooter.setVelocity(1800);
                        }

                        else if(gamepad2.y){
                            m.shooter.setVelocity(2000);
                        }

                        else{
                            m.shooter.setVelocity(1800);
                        }

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
                                    if (idTag == 23) {
                                        //  purple, purple, green
                                        cPattern[0] = 1;
                                        cPattern[1] = 1;
                                        cPattern[2] = 0;
                                    } else if (idTag == 22) {
                                        //  purple, green, purple
                                        cPattern[0] = 1;
                                        cPattern[1] = 0;
                                        cPattern[2] = 1;
                                    } else if (idTag == 21) {
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
            if (getLoculete() > 0 && Sstare == SortareShooter.SIDLE) {
                Sstare = SortareShooter.Incarca;
            }

            for (int i = 2; i >= 0; i--) {
                if (slotOcupat[i]) {
                    if (i == 0) m.sortare.setPosition(Pozitii.aruncare1);
                    else if (i == 1) m.sortare.setPosition(Pozitii.aruncare2);
                    else m.sortare.setPosition(Pozitii.aruncare3);

                    m.kdf(1100);

                    m.Saruncare.setPosition(Pozitii.lansare);
                    m.kdf(150);
                    m.Saruncare.setPosition(Pozitii.coborare);
                    m.kdf(150);

                    slotOcupat[i] = false;
                }
            }

            m.sortare.setPosition(Pozitii.luarea1);
            m.kdf(300);
            m.shooter.setVelocity(950);
            trageShooting = false;
            Sstare = SortareShooter.SIDLE;
        }

        private void BilaInPattern() {
            for (int step = 0; step < 3 && getLoculete() > 0; step++) {
                int need = cPattern[step];
                boolean ballShot = false;

                for (int slot = 0; slot < 3 && !ballShot; slot++) {
                    if (slotOcupat[slot]) {
                        if (slot == 0) m.sortare.setPosition(Pozitii.aruncare1);
                        else if (slot == 1) m.sortare.setPosition(Pozitii.aruncare2);
                        else m.sortare.setPosition(Pozitii.aruncare3);

                        m.kdf(1100);

                        boolean mov = m.color.green() <= Pozitii.mov_verde;

                        if ((need == 1 && mov) || (need == 0 && !mov)) {
                            TrageBila();
                            slotOcupat[slot] = false;
                            ballShot = true;
                        }
                    }
                }

                if (!ballShot) {
                    for (int slot = 0; slot < 3; slot++) {
                        if (slotOcupat[slot]) {
                            if (slot == 0) m.sortare.setPosition(Pozitii.aruncare1);
                            else if (slot == 1) m.sortare.setPosition(Pozitii.aruncare2);
                            else m.sortare.setPosition(Pozitii.aruncare3);

                            m.kdf(700);
                            TrageBila();
                            slotOcupat[slot] = false;
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

/*
             try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                */
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