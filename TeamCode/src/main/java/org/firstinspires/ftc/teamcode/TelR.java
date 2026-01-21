package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp(name = "Main Rosu")
@Configurable
public class TelR extends OpMode {
    private DcMotorEx frontRight, frontLeft, backRight, backLeft;
    private Follower follower;
    volatile boolean stop;
    double sm = 1;
    double max = 0;
    double FL, BL, BR, FR;

    sistemeTeleOp m = new sistemeTeleOp();

    private final double TargetX = 144;
    private final double TargetY = 144;
    private static double voltajeNominale = 12.68;


    public boolean turelaTracking = false, tracking = false, Ipornit = false, IntakePornit = false, SortingPornit = false, SortingToggle = false,Touch = false,trouch=false;
    private double ledistante, posU;
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

    private void applyVoltageCompensatedPIDF() {
        double currentVoltage = m.voltageSensor.getVoltage();
        currentVoltage = Math.max(10.0, Math.min(14.0, currentVoltage));
        double voltageCompensation = voltajeNominale / currentVoltage;
        double compensatedF = m.SkF * voltageCompensation;
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

//        m.turela.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        follower = Constants.createFollower(hardwareMap);

        Pose startingPose = new Pose(RobotPozitie.X, RobotPozitie.Y, RobotPozitie.heading);
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
                boolean dpad_right1 = gamepad2.right_bumper;
                if (turelaTracking != dpad_right1) {
                    if (gamepad2.right_bumper) {
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
                if(gamepad2.dpad_left){
                    idTag=21;
                }
                if(gamepad2.dpad_up){
                    idTag=22;
                }
                if(gamepad2.b){
                    idTag=23;
                }
                boolean gamepad1_touch = gamepad1.touchpad;
                if(Touch != gamepad1_touch){
                    if(gamepad1.touchpad){
                        trouch = !trouch;
                    }
                    Touch = gamepad1_touch;
                }
                if(gamepad1.touchpad){
                    m.turelaS.setPosition(1);
                    m.turelaD.setPosition(1);
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

                if(gamepad2.touchpad){
                    m.shooter.setVelocity(0);
                }
            }
        }
    });

    private final Thread Turela = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                follower.update();
                Pose currentPose = follower.getPose();

                double currentX = currentPose.getX();
                double currentY = currentPose.getY();
                double currentH = currentPose.getHeading();


                if (tracking) {

                    double dx = TargetX - currentX;
                    double dy = TargetY - currentY;

                    double unghiLaTarget = Math.atan2(dy, dx);
                    double turretAngleRad = unghiLaTarget - currentH;

                    turretAngleRad = normalizeAngle(turretAngleRad);

                    double turretAngleDeg = Math.toDegrees(turretAngleRad);

                    double posS = m.turelaS.angleToPosition(turretAngleDeg);
                    double posD = m.turelaD.angleToPosition(turretAngleDeg);

                    m.turelaS.setPosition(posS);
                    m.turelaD.setPosition(posD);

                }else if (!trouch){
                    m.turelaS.setPosition(0.5);
                    m.turelaD.setPosition(0.5);
                }
                else {
                    if (gamepad1.left_bumper) {
                        double pos = m.turelaS.getPosition() + 0.035;
                        m.turelaS.setPosition(pos);
                        m.turelaD.setPosition(pos);
                    } else if (gamepad1.right_bumper) {
                        double pos = m.turelaS.getPosition() - 0.035;
                        m.turelaS.setPosition(pos);
                        m.turelaD.setPosition(pos);
                    }
                }
            }
        }
    });

//    }
//                    if (tracking) {
//                        LLResult result = limelight3A.getLatestResult();
//
//                        if (result != null && result.isValid()) {
//                            tx = result.getTx();
//
//                            double error = tx;
//                            integral += error;
//                            double derivative = error - lastError;
//
//                            power = TkP * error + TkI * integral + TkD * derivative;
//
//                            int targetPos = positiiTurela + (int) (power * 50);
//
//                            if (targetPos > Pozitii.TURRET_MAX_POS) {
//                                targetPos = Pozitii.TURRET_MAX_POS;
//                                integral = 0;
//                            }
//                            if (targetPos < Pozitii.TURRET_MIN_POS) {
//                                targetPos = Pozitii.TURRET_MIN_POS;
//                                integral = 0;
//                            }
//
//                            if (positiiTurela >= Pozitii.TURRET_MIN_POS && positiiTurela <= Pozitii.TURRET_MAX_POS) {
//                                m.turela.setPower(power);
//                            } else {
//                                if ((positiiTurela <= Pozitii.TURRET_MIN_POS && power > 0) ||
//                                        (positiiTurela >= Pozitii.TURRET_MAX_POS && power < 0)) {
//                                    m.turela.setPower(power);
//                                } else {
//                                    m.turela.setPower(0);
//                                }
//                            }
//                            lastError = error;
//
//
//                        } else {
//                            m.turela.setPower(0);
//                        }
//
//                        //securitate
//                        if (positiiTurela > Pozitii.TURRET_MAX_POS) {
//                            positiiTurela = Pozitii.TURRET_MAX_POS;
//                        }
//                        if (positiiTurela < Pozitii.TURRET_MIN_POS) {
//                            positiiTurela = Pozitii.TURRET_MIN_POS;
//                        }
//
//
//                        lastError = 0;
//                        integral = 0;
//                    } else {
//                        m.turela.setPower(0);
//                    }
//                }
//
//                try {
//                    Thread.sleep(20);
//                } catch (InterruptedException e) {
//                    Thread.currentThread().interrupt();
//                }
//            }
//        }
//    });

    private final Thread Sortare = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                synchronized (blocat) {
                    int loculete = getLoculete();

                    if (Ipornit && !trageShooting && loculete < 3 && !gamepad1.b) {
                        sugere = true;
                        m.intake.setPower(1);

                        ledistante = m.distanta.getDistance(DistanceUnit.CM);

                        if (ledistante < 20 ) {
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
                                m.kdf(800);
                            } else if (Math.abs(servoPos - Pozitii.luarea2) < 0.1 && !slotOcupat[1]) {
                                slotOcupat[1] = true;
                                slotColor[1] = detectedColor;
                                if (!slotOcupat[2]) {
                                    m.sortare.setPosition(Pozitii.luarea3);
                                } else if (!slotOcupat[0]) {
                                    m.sortare.setPosition(Pozitii.luarea1);
                                }
                                m.kdf(800);
                            } else if (Math.abs(servoPos - Pozitii.luarea3) < 0.1 && !slotOcupat[2]) {
                                slotOcupat[2] = true;
                                slotColor[2] = detectedColor;
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

    private int[] cPattern = new int[3];
    private double targetShooterVelocity = 1650;

    private final Thread Shooter = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                synchronized (blocat) {
                    int loculete = getLoculete();

                    if (gamepad1.y && loculete > 0 && !sugere && !trageShooting) {
                        trageShooting = true;
                        applyVoltageCompensatedPIDF();

                        targetShooterVelocity = 1650;
                        if (gamepad2.x) targetShooterVelocity = 1650;
                        else if (gamepad2.y) targetShooterVelocity = 2000;
                        m.shooter.setVelocity(targetShooterVelocity);

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
                int slotToShoot = GasestePattern(need);

                if (slotToShoot == -1) break;

                double target = getTarget(slotToShoot);
                m.sortare.setPosition(target);

                double dist = Math.abs(target - lastPos);
                int moveWait = (int)(dist * 550) + 100;
                m.kdf(moveWait);

                shootBall();

                slotOcupat[slotToShoot] = false;
                slotColor[slotToShoot] = -1;
                lastPos = target;
            }
        }

        private int GasestePattern(int needColor) {
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
            asteaptaVelocity();
            m.Saruncare.setPosition(Pozitii.lansare);
            m.kdf(120);
            m.Saruncare.setPosition(Pozitii.coborare);
            m.kdf(100);
        }

        private void asteaptaVelocity() {
            double tolerance = targetShooterVelocity * 0.04;
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
    public void loop() {
        telemetry.update();
    }

    public void POWER(double fr1, double bl1, double br1, double fl1) {
        frontRight.setPower(fr1);
        backLeft.setPower(bl1);
        frontLeft.setPower(fl1);
        backRight.setPower(br1);
    }
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}