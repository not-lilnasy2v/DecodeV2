package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.NouHard.ServoImplExEx;
import java.util.List;


@TeleOp(name = "Main")
@Configurable
public class Tel extends OpMode {

    private DcMotorEx frontRight, frontLeft, backRight, backLeft, scula;
    private ServoImplEx bascula;
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;
    public Follower follower;
    volatile boolean stop;
    double sm = 0.7;
    double FL, BL, BR, FR;
    sistemeTeleOp m = new sistemeTeleOp();
    private static volatile double TargetX = 0;
    private static volatile double TargetY = 144;
    private static double IDLE_RATIO = 0.67;
    private volatile double currentY, currentX, currentH;


    private ServoImplExEx turelaD;
    private ServoImplExEx turelaS;

    private static final double LIMITA_STANGA_GRADE = -219.9;
    private static final double LIMITA_DREAPTA_GRADE = 218.3;
    private static final double REFERINTA_VOLTAJ_D = 0.3730;
    private static final double SCALE_FACTOR = 2.435;

    public static double ALPHA = 0.25;
    public static double MAX_OFFSET = 35.0;
    public static double DECAY = 1.0;
    public static double GAIN_DEG = 1.5;
    public static double HR_FILTER = 0.30;
    public static double T_ALPHA = 0.45;
    public static double SERVO_KP = 0.0027;
    public static double SERVO_KD = 0.00008;
    public static double SERVO_MIN_POWER = 0.05;
    public static double SERVO_MAX_POWER = 0.50;
    public static double BOOST_MAX_POWER = 0.60;
    public static double TURELA_DEADZONE = 2.0;
    public static double VEL_FILTER = 0.22;
    public static double VEL_LEAD_TIME = 0.2;
    public static double LL_LATENCY = 0.02;
    public static double LL_KI = 0.03;
    public static double LL_MAX_INTEGRAL = 15.0;
    public static double TRANS_FF_GAIN = 0.2;
    public static double MIN_DIST = 12.0;
    public static double DISTURBANCE_THRESHOLD = 17.0;

    private volatile double turelaTargetGrade = 0;
    private volatile long trackLastTime = 0;
    private volatile double lastHeading = 0;
    private volatile double filteredHeadingRate = 0;
    private volatile double llOffset = 0;
    private volatile double llIntegral = 0;
    private volatile double smoothedTarget = 0;
    private volatile boolean trackingInitializat = false;
    private volatile boolean hybridLimelightVede = false;
    private volatile double lastTx = 0;
    private volatile double xVelocity = 0;
    private volatile double yVelocity = 0;
    private volatile double lastPoseX = 0;
    private volatile double lastPoseY = 0;
    private volatile double prevMeasurement = 0;

    private static double voltajeNominale = 12.68;
    public volatile boolean turelaTracking = false, tracking = false, Ipornit = false, IntakePornit = false, SortingPornit = false, SortingToggle = false, Touch = false, trouch = false;
    private static final double[] RECOIL_OFFSETS = {0.0, 0.006, 0.014};
    private volatile double distantare, posU;
    private volatile long lastDistanceReadTime = 0;
    volatile int idTag = RobotPozitie.idTag;
    private volatile boolean[] slotOcupat = new boolean[3];
    private volatile int[] slotColor = new int[3];

    private int getLoculete() {
        int count = 0;
        for (boolean occupied : slotOcupat) {
            if (occupied) count++;
        }
        return count;
    }

    private int primaBilaPattern() {
        if (idTag == 23) return 1;
        else if (idTag == 22) return 1;
        else if (idTag == 21) return 0;
        return -1;
    }

    private int BilaCuCuloare(int color) {
        if (color == -1) {
            for (int i = 0; i < 3; i++) {
                if (slotOcupat[i]) return i;
            }
            return -1;
        }
        for (int i = 0; i < 3; i++) {
            if (slotOcupat[i] && slotColor[i] == color) return i;
        }
        for (int i = 0; i < 3; i++) {
            if (slotOcupat[i]) return i;
        }
        return -1;
    }

    private double getAruncarePos(int slot) {
        if (slot == 0) return Pozitii.aruncare1;
        if (slot == 1) return Pozitii.aruncare2;
        return Pozitii.aruncare3;
    }

    private volatile boolean sugere = false;
    private volatile boolean trageShooting = false;
    private final Object blocat = new Object();
    private volatile boolean imuRecalibrating = false;
    private volatile boolean imuReady = false;

    private void recalibrateHeading() {
        if (pinpoint != null && !imuRecalibrating) {
            imuRecalibrating = true;
            pinpoint.recalibrateIMU();
            imuRecalibrating = false;
        }
    }

    private void resetHeadingToZero() {
        if (follower != null) {
            Pose current = follower.getPose();
            follower.setPose(new Pose(current.getX(), current.getY(), 0));
        }
    }

    private void applyVoltageCompensatedPIDF() {
        double currentVoltage = m.voltageSensor.getVoltage();
        currentVoltage = Math.max(9.0, Math.min(14.0, currentVoltage));
        double voltageCompensation = voltajeNominale / currentVoltage;
        double compensatedF = m.SkF * voltageCompensation;
        PIDFCoefficients compensatedPID = new PIDFCoefficients(m.SkP, m.SkI, m.SkD, compensatedF);
        m.shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, compensatedPID);
        m.shooter2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, compensatedPID);
    }

    @Override
    public void init() {
        m.initsisteme(hardwareMap);
        scula = hardwareMap.get(DcMotorEx.class, "scula");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        bascula = hardwareMap.get(ServoImplEx.class, "bascula");

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        m.shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m.shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        scula.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        follower = Constants.createFollower(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.recalibrateIMU();
        ElapsedTime calibrationTimer = new ElapsedTime();
        while (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                && calibrationTimer.milliseconds() < 2000) {
        }
        imuReady = pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;

        Pose startingPose = new Pose(RobotPozitie.X, RobotPozitie.Y, RobotPozitie.heading);
        follower.setStartingPose(startingPose);
        follower.update();
        applyVoltageCompensatedPIDF();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        turelaD = ServoImplExEx.getContinuous(hardwareMap, "turelaD", "turretD");
        turelaD.setEncoderReferenceVoltage(REFERINTA_VOLTAJ_D);
        turelaD.setAngleLimits(LIMITA_STANGA_GRADE, LIMITA_DREAPTA_GRADE);
        turelaD.setDeadzone(TURELA_DEADZONE);
        turelaD.setPosition(0.5);

        turelaS = ServoImplExEx.getContinuous(hardwareMap, "turelaS", "turretS");
        turelaS.setPosition(0.5);

        turelaTargetGrade = 0;
    }

    @Override
    public void init_loop() {
        telemetry.addData("IMU", imuReady ? "Gata" : "Hipa");
        telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
        telemetry.addData("Heading", "%.2f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addLine(imuReady ? "POTI PORNI" : "ASTEAPTA");
        telemetry.update();
    }

    public void start() {
        follower.update();
        Butoane = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!stop) {
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        break;
                    }
                    posU = m.unghiD.getPosition();

                    if (gamepad1.dpad_right) {
                        targetShooterVelocity = 2000;
                        posU = 0.35;
                    }
                    if (gamepad1.dpad_left) {
                        targetShooterVelocity = 1650;
                        posU = 0.27;
                    }
                    if (gamepad2.touchpad) {
                        targetShooterVelocity = 0;
                    }

                    // Turela toggle
                    boolean dpad_right1 = gamepad2.x;
                    if (turelaTracking != dpad_right1) {
                        if (gamepad2.x) {
                            tracking = !tracking;
                            if (tracking) resetTracking();
                        }
                        turelaTracking = dpad_right1;
                    }
                    if (gamepad2.left_bumper && !gamepad2.right_bumper) {
                        TargetY -= 1;
                    }
                    if (gamepad2.right_bumper && !gamepad2.left_bumper) {
                        TargetY += 1;
                    }
                    if (gamepad1.dpad_up) {
                        posU += 0.003;
                    }
                    if (gamepad1.dpad_down) {
                        posU -= 0.003;
                    }
                    posU = Math.max(0, Math.min(1, posU));

                    m.unghiD.setPosition(posU);


                    if (gamepad2.dpad_left) {
                        idTag = 21;
                    }
                    if (gamepad2.dpad_up) {
                        idTag = 22;
                    }
                    if (gamepad2.b) {
                        idTag = 23;
                    }
                    if (gamepad2.dpad_down) {
                        recalibrateHeading();
                        gamepad2.rumble(200);
                    }
                    if (gamepad2.dpad_right) {
                        resetHeadingToZero();
                        gamepad2.rumble(200);
                    }
                    boolean gamepad1_touch = gamepad1.touchpad;
                    if (Touch != gamepad1_touch) {
                        if (gamepad1.touchpad) {
                            trouch = !trouch;
                        }
                        Touch = gamepad1_touch;
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
                            SortingPornit = true;
                            gamepad2.rumble(500);
                        }
                        SortingToggle = gamepad2_a;
                    }

                    if (gamepad2.left_bumper && !sugere && !trageShooting && getLoculete() == 3) {
                        for (int i = 0; i < 3; i++) {
                            if (slotOcupat[i]) {
                                double targetPos;
                                if (i == 0) targetPos = Pozitii.luarea1;
                                else if (i == 1) targetPos = Pozitii.luarea2;
                                else targetPos = Pozitii.luarea3;

                                m.moveSortareTo(targetPos);

                                m.resetareDetection();
                                slotColor[i] = m.detecteazaBiloaca();

                                if (slotColor[i] != -1) {
                                    gamepad2.rumble(150);
                                }
                            }
                        }
                        if (SortingPornit) {
                            int primaColoare = primaBilaPattern();
                            int slotShoot = BilaCuCuloare(primaColoare);
                            if (slotShoot != -1) {
                                m.sortare.setPosition(getAruncarePos(slotShoot));
                            } else {
                                m.sortare.setPosition(getAruncarePos(BilaCuCuloare(-1)));
                            }
                        } else {
                            int[] order = {0, 2, 1};
                            for (int s : order) {
                                if (slotOcupat[s]) {
                                    m.sortare.setPosition(getAruncarePos(s));
                                    break;
                                }
                            }
                        }
                        gamepad2.rumble(300);
                    }
                }
            }
        });
        Turela = new Thread(() -> {
            while (!stop) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    break;
                }

                follower.update();
                Pose currentPose = follower.getPose();
                currentX = currentPose.getX();
                currentY = currentPose.getY();
                currentH = currentPose.getHeading();

                if (tracking) {
                    trackLoop(currentPose);
                } else if (!trouch) {
                    centreazaTurela();
                    resetTracking();
                } else {
                    double manualPutere = 0;
                    if (gamepad1.left_bumper) {
                        manualPutere = -SERVO_MAX_POWER * 2;
                    } else if (gamepad1.right_bumper) {
                        manualPutere = SERVO_MAX_POWER * 2;
                    }

                    if (manualPutere != 0) {
                        setTurelaManual(manualPutere);
                    } else {
                        turelaD.setPosition(0.5);
                        turelaS.setPosition(0.5);
                    }
                }
            }
        });
        Sortare = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!stop) {
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        break;
                    }

                    boolean shouldIntake;
                    boolean bPressed;
                    synchronized (blocat) {
                        int loculete = getLoculete();
                        shouldIntake = Ipornit && !trageShooting && loculete < 3 && !gamepad1.b;
                        bPressed = gamepad1.b;
                    }

                    if (shouldIntake && !trageShooting) {
                        long now = System.currentTimeMillis();
                        if (now - lastDistanceReadTime >= 50) {
                            distantare = m.distanta.getDistance(DistanceUnit.CM);
                            lastDistanceReadTime = now;
                        }
                        if (trageShooting) continue;
                        sugere = true;
                        m.intake.setPower(1);
                        bascula.setPosition(Pozitii.sede);

                        if (m.bilaPrezenta(distantare)) {
                            int detectedColor = m.detecteazaBiloaca();
                            long waitMs = 0;
                            synchronized (blocat) {
                                double oldPos = m.sortare.getPosition();
                                if (Math.abs(oldPos - Pozitii.luarea1) < 0.1 && !slotOcupat[0]) {
                                    slotOcupat[0] = true;
                                    slotColor[0] = detectedColor;
                                    if (!slotOcupat[1]) {
                                        m.sortare.setPosition(Pozitii.luarea2);
                                    } else if (!slotOcupat[2]) {
                                        m.sortare.setPosition(Pozitii.luarea3);
                                    }
                                } else if (Math.abs(oldPos - Pozitii.luarea2) < 0.1 && !slotOcupat[1]) {
                                    slotOcupat[1] = true;
                                    slotColor[1] = detectedColor;
                                    if (!slotOcupat[2]) {
                                        m.sortare.setPosition(Pozitii.luarea3);
                                    } else if (!slotOcupat[0]) {
                                        m.sortare.setPosition(Pozitii.luarea1);
                                    }
                                } else if (Math.abs(oldPos - Pozitii.luarea3) < 0.1 && !slotOcupat[2]) {
                                    slotOcupat[2] = true;
                                    slotColor[2] = detectedColor;
                                }

                                double newPos = m.sortare.getPosition();
                                if (newPos != oldPos) {
                                    waitMs = m.sortareWaitMs(oldPos, newPos);
                                }

                                if (getLoculete() == 3) {
                                    Ipornit = false;
                                    m.intake.setPower(0);
                                    gamepad1.rumble(2000);

                                    double pos3 = m.sortare.getPosition();
                                    if (SortingPornit) {
                                        int primaColoare = primaBilaPattern();
                                        int slotShoot = BilaCuCuloare(primaColoare);
                                        if (slotShoot != -1) {
                                            m.sortare.setPosition(getAruncarePos(slotShoot));
                                        } else {
                                            m.sortare.setPosition(Pozitii.aruncare1);
                                        }
                                    } else {
                                        m.sortare.setPosition(Pozitii.aruncare1);
                                    }
                                    waitMs = m.sortareWaitMs(pos3, m.sortare.getPosition());
                                }
                            }

                            if (waitMs > 0) {
                                m.kdf(waitMs);
                            }
                        }
                    } else if (bPressed) {
                        sugere = false;
                        m.intake.setPower(-1);
                    } else {
                        sugere = false;
                        m.intake.setPower(0);
                    }
                }
            }
        });
        Shooter = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!stop) {
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        break;
                    }
                    synchronized (blocat) {
                        int loculete = getLoculete();

                        if (!trageShooting) {
                            if (loculete == 3) {
                                m.shooter.setVelocity(targetShooterVelocity);
                                m.shooter2.setVelocity(targetShooterVelocity);
                            } else if (loculete > 0) {
                                m.shooter.setVelocity(targetShooterVelocity * IDLE_RATIO);
                                m.shooter2.setVelocity(targetShooterVelocity * IDLE_RATIO);
                            } else {
                                m.shooter.setVelocity(0);
                                m.shooter2.setVelocity(0);
                            }
                        }

                        if (gamepad1.right_trigger_pressed && loculete > 0 && !sugere && !trageShooting) {
                            trageShooting = true;
                            applyVoltageCompensatedPIDF();
                            m.shooter.setVelocity(targetShooterVelocity);
                            m.shooter2.setVelocity(targetShooterVelocity);
                            waitForShooterReady();

                            if (SortingPornit && loculete == 3) {
                                Pattern();
                                shootPattern();
                            } else {
                                rapidFireShoot();
                            }
                            SortingPornit = false;
                            bascula.setPosition(Pozitii.sede);
                            m.sortare.setPosition(Pozitii.luarea1);
                            scula.setPower(0);

                            trageShooting = false;
                            Ipornit = true;
                        }
                    }
                }
            }

            private void Pattern() {
                if (idTag == 23) {
                    cPattern[0] = 1;
                    cPattern[1] = 1;
                    cPattern[2] = 0;
                } else if (idTag == 22) {
                    cPattern[0] = 1;
                    cPattern[1] = 0;
                    cPattern[2] = 1;
                } else if (idTag == 21) {
                    cPattern[0] = 0;
                    cPattern[1] = 1;
                    cPattern[2] = 1;
                } else {
                    cPattern[0] = -1;
                    cPattern[1] = -1;
                    cPattern[2] = -1;
                }
            }

            private void waitForShooterReady() {
                double tolerance = targetShooterVelocity * 0.03;
                double currentVoltage = Math.max(9.0, m.voltageSensor.getVoltage());
                long maxTimeout = targetShooterVelocity >= 2000 ? 500 : 250;
                long timeoutMs = Math.min(maxTimeout, (long) (350 * (voltajeNominale / currentVoltage)));
                long timeout = System.currentTimeMillis() + timeoutMs;
                boolean ready = false;
                while (System.currentTimeMillis() < timeout) {
                    double v1 = Math.abs(m.shooter.getVelocity());
                    double v2 = Math.abs(m.shooter2.getVelocity());
                    if (Math.abs(v1 - targetShooterVelocity) < tolerance
                            && Math.abs(v2 - targetShooterVelocity) < tolerance) {
                        ready = true;
                        break;
                    }
                    try {
                        Thread.sleep(2);
                    } catch (InterruptedException ignored) {
                    }
                }
                if (!ready) {
                    applyVoltageCompensatedPIDF();
                    gamepad1.rumble(100);
                }
            }

            private void shootPattern() {


                double lastPos = m.sortare.getPosition();
                double originalPosU = posU;
                double velScale = targetShooterVelocity / 1550.0;
                int shotsFired = 0;
                scula.setPower(-1);
                bascula.setPosition(Pozitii.sede);

                for (int step = 0; step < 3; step++) {
                    int need = cPattern[step];
                    int slotShoot = GasestePattern(need);

                    if (slotShoot == -1) break;

                    double recoil = RECOIL_OFFSETS[Math.min(shotsFired, 2)] * velScale;
                    m.unghiD.setPosition(originalPosU + recoil);

                    double target = getTarget(slotShoot);
                    if (target != lastPos) {
                        m.sortare.setPosition(target);
                        m.kdf(m.sortareWaitMs(lastPos, target));
                    }

                    applyVoltageCompensatedPIDF();
                    waitForShooterReady();

                    bascula.setPosition(Pozitii.lansare);
                    m.kdf(120);
                    bascula.setPosition(Pozitii.sede);
                    m.kdf(30);

                    slotOcupat[slotShoot] = false;
                    slotColor[slotShoot] = -1;
                    lastPos = target;
                    shotsFired++;
                }
                posU = originalPosU;
                m.unghiD.setPosition(posU);
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
                double originalPosU = posU;
                double velScale = targetShooterVelocity / 1550.0;
                int shotsFired = 0;
                scula.setPower(-1);
                bascula.setPosition(Pozitii.lansareRapid);

                int[] order = {0, 2, 1};
                for (int s : order) {
                    if (slotOcupat[s]) {
                        applyVoltageCompensatedPIDF();

                        double recoil = RECOIL_OFFSETS[Math.min(shotsFired, 2)] * velScale;
                        m.unghiD.setPosition(originalPosU + recoil);

                        double target = getTarget(s);
                        if (target != lastPos) {
                            m.sortare.setPosition(target);
                            m.kdf(m.sortareWaitMs(lastPos, target));
                        }
                        m.kdf(30);

                        slotOcupat[s] = false;
                        slotColor[s] = -1;
                        lastPos = target;
                        shotsFired++;
                    }
                }
                posU = originalPosU;
                m.unghiD.setPosition(posU);
            }

            private double getTarget(int slot) {
                if (slot == 0) return Pozitii.aruncare1;
                if (slot == 1) return Pozitii.aruncare2;
                return Pozitii.aruncare3;
            }
        });
        Chassis = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!stop) {

                    double y = -gamepad1.left_stick_y;
                    double x = gamepad1.left_stick_x * 1.1;
                    double rx = gamepad1.right_stick_x;

                    FL = (y + x + rx);
                    BL = (y - x + rx);
                    BR = (y + x - rx);
                    FR = (y - x - rx);

                    double max = Math.max(Math.max(abs(FL), abs(FR)), Math.max(abs(BL), abs(BR)));
                    if (max > 1) {
                        FL /= max;
                        FR /= max;
                        BL /= max;
                        BR /= max;
                    }
                    if (gamepad1.left_trigger > 0) {
                        sm = 2.7;
                    } else {
                        sm = 0.7;
                    }
                    POWER(FR / sm, BL / sm, BR / sm, FL / sm);
                    try {
                        Thread.sleep(5);
                    } catch (InterruptedException e) {
                        break;
                    }
                }
            }
        });
        Chassis.start();
        Butoane.start();
        Turela.start();
        Sortare.start();
        Shooter.start();
    }

    private Thread Butoane;

    private void trackLoop(Pose pose) {
        long now = System.nanoTime();

        if (trackLastTime == 0) {
            trackLastTime = now;
            lastHeading = pose.getHeading();
            lastPoseX = pose.getX();
            lastPoseY = pose.getY();
            return;
        }

        double dt = (now - trackLastTime) / 1_000_000_000.0;
        dt = Math.max(0.005, Math.min(0.1, dt));
        trackLastTime = now;

        double headingDiff = pose.getHeading() - lastHeading;
        headingDiff = normalizeAngle(headingDiff);
        lastHeading = pose.getHeading();
        double rawHR = headingDiff / dt;
        filteredHeadingRate += HR_FILTER * (rawHR - filteredHeadingRate);

        xVelocity += VEL_FILTER * ((pose.getX() - lastPoseX) / dt - xVelocity);
        yVelocity += VEL_FILTER * ((pose.getY() - lastPoseY) / dt - yVelocity);
        lastPoseX = pose.getX();
        lastPoseY = pose.getY();

        double predictedX = pose.getX() + xVelocity * VEL_LEAD_TIME;
        double predictedY = pose.getY() + yVelocity * VEL_LEAD_TIME;
        double dx = TargetX - predictedX;
        double dy = TargetY - predictedY;
        double distToTarget = Math.hypot(dx, dy);
        double angleToTarget = Math.atan2(dy, dx);
        double turretRad = normalizeAngle(angleToTarget - pose.getHeading());
        double odomAngle = Math.toDegrees(turretRad) * SCALE_FACTOR;
        odomAngle = clamp(odomAngle, LIMITA_STANGA_GRADE, LIMITA_DREAPTA_GRADE);

        double tangentialVel = xVelocity * Math.sin(angleToTarget) - yVelocity * Math.cos(angleToTarget);
        double bearingRate = tangentialVel / Math.max(distToTarget, MIN_DIST);
        double translationalFF = Math.toDegrees(bearingRate) * SCALE_FACTOR * TRANS_FF_GAIN;

        LLResult result = limelight.getLatestResult();
        hybridLimelightVede = false;
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                double tx = result.getTx();
                lastTx = tx;
                hybridLimelightVede = true;
                double currentAngle = turelaD.getCurrentAngle();
                double angleAtCapture = currentAngle - Math.toDegrees(filteredHeadingRate) * LL_LATENCY * SCALE_FACTOR;
                double trueTarget = angleAtCapture - tx * SCALE_FACTOR;
                double offsetError = trueTarget - odomAngle;
                llOffset += ALPHA * (offsetError - llOffset);
                double rawOutput = SERVO_KP * (odomAngle + llOffset - currentAngle);
                if (Math.abs(rawOutput) < SERVO_MAX_POWER) {
                    llIntegral += offsetError * dt;
                    llIntegral = clamp(llIntegral, -LL_MAX_INTEGRAL, LL_MAX_INTEGRAL);
                }
                llOffset = clamp(llOffset, -MAX_OFFSET, MAX_OFFSET);
            }
        }
        if (!hybridLimelightVede) {
            llOffset *= DECAY;
            llIntegral *= 0.95;
        }

        double targetAngle = odomAngle + llOffset + llIntegral * LL_KI;
        targetAngle += filteredHeadingRate * GAIN_DEG + translationalFF;
        targetAngle = clamp(targetAngle, LIMITA_STANGA_GRADE, LIMITA_DREAPTA_GRADE);

        double currentAngle = turelaD.getCurrentAngle();
        double preError = Math.abs(targetAngle - currentAngle);
        double adaptiveAlpha;
        if (preError > DISTURBANCE_THRESHOLD) adaptiveAlpha = 0.95;
        else if (preError > 5.0) adaptiveAlpha = 0.85;
        else adaptiveAlpha = T_ALPHA;

        if (!trackingInitializat) {
            smoothedTarget = targetAngle;
            trackingInitializat = true;
        } else {
            smoothedTarget += adaptiveAlpha * (targetAngle - smoothedTarget);
        }

        turelaTargetGrade = smoothedTarget;
        double posError = smoothedTarget - currentAngle;
        double measurement = currentAngle;
        double posDerivative = -(measurement - prevMeasurement) / dt;
        prevMeasurement = measurement;

        double absError = Math.abs(posError);
        if (absError < TURELA_DEADZONE && Math.abs(posDerivative) < 50.0) {
            turelaD.setPosition(0.5);
            turelaS.setPosition(0.5);
            return;
        }

        double effectiveMaxPower;
        if (absError > 20.0) effectiveMaxPower = BOOST_MAX_POWER;
        else if (absError > 10.0) effectiveMaxPower = SERVO_MAX_POWER + 0.05;
        else effectiveMaxPower = SERVO_MAX_POWER;

        double power = SERVO_KP * posError + SERVO_KD * posDerivative;
        if (absError < TURELA_DEADZONE * 2) {
            double ramp = (absError - TURELA_DEADZONE) / TURELA_DEADZONE;
            power *= Math.max(0, ramp);
        }
        if (Math.abs(power) < SERVO_MIN_POWER && absError > TURELA_DEADZONE) {
            power = Math.signum(power) * SERVO_MIN_POWER;
        }
        power = clamp(power, -effectiveMaxPower, effectiveMaxPower);
        turelaD.setPosition(0.5 - power);
        turelaS.setPosition(0.5 - power);
    }

    private void centreazaTurela() {
        double curr = turelaD.getCurrentAngle();
        if (Math.abs(curr) < TURELA_DEADZONE) {
            turelaD.setPosition(0.5);
            turelaS.setPosition(0.5);
        } else {
            double p = clamp(SERVO_KP * (0 - curr), -SERVO_MAX_POWER, SERVO_MAX_POWER);
            turelaD.setPosition(0.5 - p);
            turelaS.setPosition(0.5 - p);
        }
    }

    private void setTurelaManual(double putere) {
        turelaD.setPosition(0.5 - putere);
        turelaS.setPosition(0.5 - putere);
    }

    private Thread Turela;

    private void resetTracking() {
        trackLastTime = 0;
        lastHeading = 0;
        filteredHeadingRate = 0;
        llOffset = 0;
        llIntegral = 0;
        smoothedTarget = 0;
        trackingInitializat = false;
        hybridLimelightVede = false;
        xVelocity = 0;
        yVelocity = 0;
        lastPoseX = 0;
        lastPoseY = 0;
        prevMeasurement = 0;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    private Thread Sortare;

    private int[] cPattern = new int[3];
    private volatile double targetShooterVelocity = 1550;

    private Thread Shooter;

    private Thread Chassis;

    public void stop() {
        stop = true;
        turelaD.setPosition(0.5);
        turelaS.setPosition(0.5);
        if (limelight != null) {
            limelight.stop();
        }
    }

    @Override
    public void loop() {
        int detected = m.UltimaDetectare();
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
        telemetry.addLine("");
        telemetry.addLine("=== TURELA CR ===");
        telemetry.addData("Unghi target", "%.1f°", turelaTargetGrade);
        telemetry.addData("Encoder D", "%.1f° (%.3fV)", turelaD.getCurrentAngle(), turelaD.getEncoderVoltage());
        telemetry.addData("Encoder S", "%.3fV", turelaS.getEncoderVoltage());
        telemetry.addData("Eroare", "%.1f°", turelaTargetGrade - turelaD.getCurrentAngle());
        telemetry.addLine("");

        telemetry.addData("id", idTag);
        telemetry.addData("shooter", m.shooter.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("shooter2", m.shooter2.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Total biloace", getLoculete());
        telemetry.addData("unghi", posU);
        telemetry.addData("distanta", distantare);
        telemetry.addData("targetVelocity", targetShooterVelocity);
        telemetry.addLine("");
        telemetry.addData("Tracking", tracking ? (hybridLimelightVede ? "ODOM+LL" : "ODOM") : "OFF");
        telemetry.addData("tx", "%.2f", lastTx);
        telemetry.addData("LL offset", "%.2f", llOffset);
        telemetry.addData("Manual (trouch)", trouch ? "ON" : "OFF");
        telemetry.addLine("");
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("y", currentY);
        telemetry.addData("TargetY", TargetY);
        telemetry.update();
    }

    public void POWER(double fr1, double bl1, double br1, double fl1) {
        frontRight.setPower(fr1);
        backLeft.setPower(bl1);
        frontLeft.setPower(fl1);
        backRight.setPower(br1);
    }
}
