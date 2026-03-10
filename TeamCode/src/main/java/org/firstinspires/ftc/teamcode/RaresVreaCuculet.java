package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
@Configurable
public class RaresVreaCuculet extends OpMode {
    private DcMotorEx frontRight, frontLeft, backRight, backLeft;
    private Follower follower;
    double sm = 1;
    double max = 0;
    double FL, BL, BR, FR;
    sistemeTeleOp m = new sistemeTeleOp();
    private volatile boolean stop = false;
    private static double voltajeNominale = 12.68;

    private final double TargetX = 0;
    private static double TargetY = 150;

    private static final double offsetVelocity = 0;
    private static final double multiplicator = 1.0;
    private static final double minVelocity = 800;
    private static final double maxVelocity = 2200;
    private static final double unghiOffset = 0;
    private static final double multiplicatorUnghi = 1.0;

    private static final int distanteSampels = 5;
    private static final double velocitateToleranta = 0.04;
    private static final double MinDistantaTragere = 60;
    private static final double MaxDistantaTragere = 180;

    public volatile boolean Ipornit = false, IntakePornit = false,
            SortingPornit = false, SortingToggle = false,
            Touch = false, trouch = false;
    private volatile double distantare, posU;

    private volatile double DistanceLaTarget = 0;
    private volatile double autoVelocity = 1650;
    private volatile double autoHoodPosition = 0.20;
    public volatile boolean autoShootEnabled = false;
    private volatile boolean SToggle = false;

    private double[] distanceBuff = new double[distanteSampels];
    private int distanceBuffIn = 0;
    private boolean distanceBufferF = false;

    private volatile double rawVelocity = 0, rawUnghidPos = 0;

    int idTag = RobotPozitie.idTag;
    private volatile boolean[] slotOcupat = new boolean[3];
    private volatile int[] slotColor = new int[3];

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

    private double EcuatieVelocity(double x) {
        double result = -0.00000516094 * Math.pow(x, 3)
                + 0.00138528 * Math.pow(x, 2)
                - 0.115998 * x
                + 3.332;
        return result * 1000;
    }

    private double EcuatieHoodAngle(double x) {
        return 0.00155259 * Math.pow(x, 4)
                - 0.417429 * Math.pow(x, 3)
                + 35.2741 * Math.pow(x, 2)
                - 926.56355 * x;
    }

    private double CalculeVelocitate(double distanceInch) {
        rawVelocity = EcuatieVelocity(distanceInch);
        double velocity = (rawVelocity * multiplicator) + offsetVelocity;
        velocity = Math.max(minVelocity, Math.min(maxVelocity, velocity));
        return velocity;
    }

    private double CalculareCucUnghi(double distanceInch) {
        rawUnghidPos = EcuatieHoodAngle(distanceInch);
        double position = (rawUnghidPos * multiplicatorUnghi) + unghiOffset;
        return position;
    }

    private double CalculareDistantaLaCos() {
        Pose currentPose = follower.getPose();
        double dx = TargetX - currentPose.getX();
        double dy = TargetY - currentPose.getY();
        double instantDistance = Math.sqrt(dx * dx + dy * dy);

        distanceBuff[distanceBuffIn] = instantDistance;
        distanceBuffIn = (distanceBuffIn + 1) % distanteSampels;
        if (distanceBuffIn == 0) distanceBufferF = true;

        int count = distanceBufferF ? distanteSampels : distanceBuffIn;
        if (count == 0) return instantDistance;

        double sum = 0;
        for (int i = 0; i < count; i++) {
            sum += distanceBuff[i];
        }
        return sum / count;
    }

    private boolean TragereInRange(double distance) {
        return distance >= MinDistantaTragere && distance <= MaxDistantaTragere;
    }

    private void ShooterAdaptare() {
        DistanceLaTarget = CalculareDistantaLaCos();
        if (TragereInRange(DistanceLaTarget)) {
            autoVelocity = CalculeVelocitate(DistanceLaTarget);
            autoHoodPosition = CalculareCucUnghi(DistanceLaTarget);
        }
    }

    private void applyVoltageCompensatedPIDF() {
        double currentVoltage = m.voltageSensor.getVoltage();
        currentVoltage = Math.max(10.0, Math.min(14.0, currentVoltage));
        double voltageCompensation = voltajeNominale / currentVoltage;
        double compensatedF = m.SkF * voltageCompensation;
        PIDFCoefficients compensatedPID = new PIDFCoefficients(m.SkP, m.SkI, m.SkD, compensatedF);
        m.shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, compensatedPID);
        m.shooter2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, compensatedPID);
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

        follower = Constants.createFollower(hardwareMap);
        Pose startingPose = new Pose(RobotPozitie.X, RobotPozitie.Y, RobotPozitie.heading);
        follower.setStartingPose(startingPose);
        follower.update();

        applyVoltageCompensatedPIDF();

        for (int i = 0; i < distanteSampels; i++) {
            distanceBuff[i] = 0;
        }
    }

    @Override
    public void start() {
        follower.update();
        Chassis.start();
        Butoane.start();
        Sortare.start();
        Shooter.start();
        AutoShootUpdater.start();
    }

    private final Thread AutoShootUpdater = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                try { Thread.sleep(20); } catch (InterruptedException e) { break; }
                if (autoShootEnabled) {
                    follower.update();
                    ShooterAdaptare();

                    m.unghiD.setPosition(autoHoodPosition);

                    if (!trageShooting) {
                        m.shooter.setVelocity(autoVelocity);
                        m.shooter2.setVelocity(autoVelocity);
                    }
                }
            }
        }
    });

    private final Thread Butoane = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                try { Thread.sleep(10); } catch (InterruptedException e) { break; }
                posU = m.unghiD.getPosition();

                boolean gamepad2_lb = gamepad2.left_bumper;
                if (SToggle != gamepad2_lb) {
                    if (gamepad2.left_bumper) {
                        autoShootEnabled = !autoShootEnabled;
                        if (autoShootEnabled) {
                            gamepad2.rumble(200);
                        }
                    }
                    SToggle = gamepad2_lb;
                }

                if (gamepad1.right_bumper) {
                    TargetY += 1;
                }
                if (gamepad1.left_bumper) {
                    TargetY -= 1;
                }

                boolean gamepad1_touch = gamepad1.touchpad;
                if (Touch != gamepad1_touch) {
                    if (gamepad1.touchpad) {
                        trouch = !trouch;
                    }
                    Touch = gamepad1_touch;
                }

                if (!autoShootEnabled) {
                    if (gamepad1.dpad_up) {
                        posU += 0.003;
                    }
                    if (gamepad1.dpad_down) {
                        posU -= 0.003;
                    }
                    m.unghiD.setPosition(posU);
                }

                if (gamepad2.dpad_left) {
                    idTag = 21;
                }
                if (gamepad2.dpad_up) {
                    idTag = 22;
                }
                if (gamepad2.dpad_right) {
                    idTag = 23;
                }

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

                if (gamepad2.touchpad) {
                    m.shooter.setVelocity(0);
                    m.shooter2.setVelocity(0);
                }
            }
        }
    });

    private final Thread Sortare = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                try { Thread.sleep(10); } catch (InterruptedException e) { break; }
                synchronized (blocat) {
                    int loculete = getLoculete();

                    if (Ipornit && !trageShooting && loculete < 3 && !gamepad1.b) {
                        sugere = true;
                        m.intake.setPower(1);

                        distantare = m.distanta.getDistance(DistanceUnit.CM);

                        if (m.bilaPrezenta(distantare)) {
                            int detectedColor = m.detecteazaBiloaca();
                            double servoPos = m.sortare.getPosition();
                            if (Math.abs(servoPos - Pozitii.luarea1) < 0.1 && !slotOcupat[0]) {
                                slotOcupat[0] = true;
                                slotColor[0] = detectedColor;
                                if (!slotOcupat[1]) {
                                    m.sortare.setPosition(Pozitii.luarea2);
                                } else if (!slotOcupat[2]) {
                                    m.sortare.setPosition(Pozitii.luarea3);
                                }
                                m.kdf(350);
                            } else if (Math.abs(servoPos - Pozitii.luarea2) < 0.1 && !slotOcupat[1]) {
                                slotOcupat[1] = true;
                                slotColor[1] = detectedColor;
                                if (!slotOcupat[2]) {
                                    m.sortare.setPosition(Pozitii.luarea3);
                                } else if (!slotOcupat[0]) {
                                    m.sortare.setPosition(Pozitii.luarea1);
                                }
                                m.kdf(350);
                            } else if (Math.abs(servoPos - Pozitii.luarea3) < 0.1 && !slotOcupat[2]) {
                                slotOcupat[2] = true;
                                slotColor[2] = detectedColor;
                                m.kdf(350);
                            }

                            if (getLoculete() == 3) {
                                Ipornit = false;
                                m.intake.setPower(0);
                                gamepad1.rumble(2000);
                                m.sortare.setPosition(Pozitii.aruncare1);
                            }
                        }
                    } else if (gamepad1.b) {
                        sugere = false;
                        m.intake.setPower(-1);
                    } else {
                        sugere = false;
                        m.intake.setPower(0);
                    }
                }
            }
        }
    });

    private int[] cPattern = new int[3];
    private double targetShooterVelocity = 1650;

    private final Thread Shooter = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                try { Thread.sleep(10); } catch (InterruptedException e) { break; }
                synchronized (blocat) {
                    int loculete = getLoculete();

                    if (gamepad1.y && loculete > 0 && !sugere && !trageShooting) {
                        trageShooting = true;
                        applyVoltageCompensatedPIDF();

                        if (autoShootEnabled) {
                            ShooterAdaptare();
                            targetShooterVelocity = autoVelocity;

                            if (!TragereInRange(DistanceLaTarget)) {
                                gamepad1.rumble(100);
                                gamepad2.rumble(100);
                                targetShooterVelocity = 1650;
                            }
                        } else {
                            targetShooterVelocity = 1650;
                        }

                        m.shooter.setVelocity(targetShooterVelocity);
                        m.shooter2.setVelocity(targetShooterVelocity);
                        asteaptaVelocity();

                        if (SortingPornit) {
                            Pattern();
                            shootPattern();
                        } else {
                            rapidFireShoot();
                        }

                        m.sortare.setPosition(Pozitii.luarea1);
                        m.kdf(150);
                        m.shooter.setVelocity(950);
                        m.shooter2.setVelocity(950);
                        trageShooting = false;
                    }
                }
            }
        }

        private void Pattern() {
            if (idTag == 23) {
                cPattern[0] = 1; cPattern[1] = 1; cPattern[2] = 0;
            } else if (idTag == 22) {
                cPattern[0] = 1; cPattern[1] = 0; cPattern[2] = 1;
            } else if (idTag == 21) {
                cPattern[0] = 0; cPattern[1] = 1; cPattern[2] = 1;
            } else {
                cPattern[0] = -1; cPattern[1] = -1; cPattern[2] = -1;
            }
        }

        private void shootPattern() {
            double lastPos = m.sortare.getPosition();

            for (int step = 0; step < 3; step++) {
                int need = cPattern[step];
                int slotToShoot = BallPattern(need);

                if (slotToShoot == -1) break;

                double target = getTarget(slotToShoot);
                m.sortare.setPosition(target);

                double dist = Math.abs(target - lastPos);
                int moveWait = (int)(dist * 550) + 100;
                m.kdf(moveWait);

                m.shooter.setVelocity(targetShooterVelocity);
                m.shooter2.setVelocity(targetShooterVelocity);
                asteaptaVelocity();
                shootBall();

                slotOcupat[slotToShoot] = false;
                slotColor[slotToShoot] = -1;
                lastPos = target;
            }
        }

        private int BallPattern(int needColor) {
            if (needColor == -1) {
                for (int i = 0; i < 3; i++) {
                    if (slotOcupat[i]) return i;
                }
                return -1;
            }

            for (int i = 0; i < 3; i++) {
                if (slotOcupat[i] && slotColor[i] == needColor) {
                    return i;
                }
            }

            for (int i = 0; i < 3; i++) {
                if (slotOcupat[i]) return i;
            }
            return -1;
        }

        private void rapidFireShoot() {
            double lastPos = m.sortare.getPosition();

            for (int i = 2; i >= 0; i--) {
                if (slotOcupat[i]) {
                    double target = getTarget(i);
                    m.sortare.setPosition(target);

                    double dist = Math.abs(target - lastPos);
                    int moveWait = (int)(dist * 550) + 100;
                    m.kdf(moveWait);

                    m.shooter.setVelocity(targetShooterVelocity);
                    m.shooter2.setVelocity(targetShooterVelocity);
                    asteaptaVelocity();
                    shootBall();

                    slotOcupat[i] = false;
                    slotColor[i] = -1;
                    lastPos = target;
                }
            }
        }

        private double getTarget(int slot) {
            if (slot == 0) return Pozitii.aruncare1;
            if (slot == 1) return Pozitii.aruncare2;
            return Pozitii.aruncare3;
        }

        private void shootBall() {
//            m.Saruncare.setPosition(Pozitii.lansare);
//            m.kdf(120);
//            m.Saruncare.setPosition(Pozitii.coborare);
//            m.kdf(100);
        }

        private void asteaptaVelocity() {
            double tolerance = targetShooterVelocity * velocitateToleranta;
            int stableCount = 0;
            int maxWait = 40;
            int waited = 0;

            while (stableCount < 2 && waited < maxWait) {
                double current = m.shooter.getVelocity();
                double error = Math.abs(current - targetShooterVelocity);

                if (error <= tolerance) {
                    stableCount++;
                } else {
                    stableCount = 0;
                }

                m.kdf(8);
                waited++;
            }
        }
    });

    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                try { Thread.sleep(5); } catch (InterruptedException e) { break; }
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

    @Override
    public void loop() {
        int detected = m.detecteazaBiloaca();
        String colorN;
        if (detected == 0) {
            colorN = "verde";
        } else if (detected == 1) {
            colorN = "mov";
        } else {
            colorN = "pulicioi nu ii nimic";
        }
        telemetry.addData("detectat", colorN);
        telemetry.addLine("");

        for (int i = 0; i < 3; i++) {
            String status;
            if (!slotOcupat[i]) {
                status = "cheala ca Miklos";
            } else if (slotColor[i] == 0) {
                status = "verde";
            } else if (slotColor[i] == 1) {
                status = "mov";
            } else {
                status = "?";
            }
            telemetry.addData("Slot " + (i + 1), status);
        }
        telemetry.addData("Total biloace", getLoculete());
        telemetry.addLine("");
        telemetry.addData("AutoShoot", autoShootEnabled ? "pornit" : "oprit");
        telemetry.addData("Distance", String.format("%.1f", DistanceLaTarget));

        String rangeStatus;
        if (DistanceLaTarget < MinDistantaTragere) {
            rangeStatus = "prea aproape";
        } else if (DistanceLaTarget > MaxDistantaTragere) {
            rangeStatus = "prea departe";
        } else {
            rangeStatus = "IN RANGE";
        }
        telemetry.addData("Range", rangeStatus);

        telemetry.addLine("");
        telemetry.addData("Auto Velocity", String.format("%.0f ticks/s", autoVelocity));
        telemetry.addData("Actual Velocity", String.format("%.0f ticks/s", m.shooter.getVelocity()));
        telemetry.addData("Auto Hood", String.format("%.4f", autoHoodPosition));

        telemetry.addLine("");
        telemetry.addData("Raw Velocity (x1000)", String.format("%.1f", rawVelocity));
        telemetry.addData("Raw Hood Angle", String.format("%.4f", rawUnghidPos));

        telemetry.addLine("");
        telemetry.addData("unghi manual", posU);
        telemetry.addData("distanta intake", distantare);
        telemetry.addData("TargetY", TargetY);
        telemetry.update();
    }

    @Override
    public void stop() {
        stop = true;
        m.shooter.setVelocity(0);
        m.shooter2.setVelocity(0);
        m.intake.setPower(0);
    }

    public void POWER(double fr1, double bl1, double br1, double fl1) {
        frontRight.setPower(fr1);
        backLeft.setPower(bl1);
        frontLeft.setPower(fl1);
        backRight.setPower(br1);
    }
}