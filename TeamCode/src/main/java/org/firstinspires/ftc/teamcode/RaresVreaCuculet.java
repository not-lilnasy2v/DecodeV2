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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.NouHard.ServoImplExEx;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import java.util.List;

@TeleOp
@Configurable
public class RaresVreaCuculet extends OpMode {
    private DcMotorEx frontRight, frontLeft, backRight, backLeft, scula;
    private ServoImplEx bascula;
    private Limelight3A limelight;
    private ServoImplExEx turelaD;
    private ServoImplExEx turelaS;
    private Follower follower;
    double sm = 1;
    double max = 0;
    double FL, BL, BR, FR;
    sistemeTeleOp m = new sistemeTeleOp();
    private volatile boolean stop = false;
    private static double voltajeNominale = 12.68;

    private static final double TargetX = 144;
    private static volatile double TargetY = 144;

    private static final double LIMITA_STANGA_GRADE = -219.9;
    private static final double LIMITA_DREAPTA_GRADE = 218.3;
    private static final double REFERINTA_VOLTAJ_D = 0.3730;
    private static final double SCALE_FACTOR = 2.435;

    public static double ALPHA = 0.50;
    public static double MAX_OFFSET = 35.0;
    public static double DECAY = 1.0;
    public static double GAIN_DEG = 1.5;
    public static double HR_FILTER = 0.30;
    public static double T_ALPHA = 0.65;
    public static double SERVO_KP = 0.0030;
    public static double SERVO_KD = 0.00035;
    public static double SERVO_MIN_POWER = 0.05;
    public static double SERVO_MAX_POWER = 0.50;
    public static double BOOST_MAX_POWER = 0.60;
    public static double TURELA_DEADZONE = 1.5;
    public static double TRACK_VEL_FILTER = 0.22;
    public static double VEL_LEAD_TIME = 0.2;
    public static double LL_LATENCY = 0.02;
    public static double LL_KI = 0.03;
    public static double LL_MAX_INTEGRAL = 15.0;
    public static double TRANS_FF_GAIN = 0.2;
    public static double MIN_DIST = 12.0;
    public static double TURELA_OFFSET_DEG = -15.0;
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
    private volatile double trackXVel = 0, trackYVel = 0;
    private volatile double trackLastPoseX = 0, trackLastPoseY = 0;
    private volatile double prevMeasurement = 0;
    private volatile boolean tracking = true;

    private static final double offsetVelocity = 0;
    private static final double multiplicator = 1.0;
    private static final double minVelocity = 800;
    private static final double maxVelocity = 2200;
    private static final double unghiOffset = 0;
    private static final double multiplicatorUnghi = 1.0;

    private static final int distanteSampels = 5;
    private static final double velocitateToleranta = 0.07;
    private static final double[] RECOIL_OFFSETS = {0.0, 0.005, 0.012};
    private static final double MinDistantaTragere = 60;
    private static final double MaxDistantaTragere = 180;

    private static final double VELOCITY_EMA_ALPHA = 0.5;
    private volatile double smoothedVelocity = 1650;
    private volatile double smoothedHood = 0.20;

    private static final double JAM_CURRENT_THRESHOLD = 6.5;
    private static final long JAM_DEBOUNCE_MS = 250;
    private static final long JAM_REVERSE_MS = 300;
    private volatile boolean intakeJammed = false;

    private static final long AUTO_SHOOT_DWELL_MS = 80;
    private volatile boolean autoShootReady = false;
    private volatile long readySince = 0;

    private static final double BASE_FLIGHT_TIME_S = 0.35;

    private double calculateFlightTime() {
        return BASE_FLIGHT_TIME_S * (1650.0 / Math.max(800, targetShooterVelocity));
    }

    private volatile boolean rpmOK = false;
    private volatile boolean distOK = false;

    private volatile double xVelocity = 0, yVelocity = 0;
    private double lastPoseX = 0, lastPoseY = 0;
    private long lastPoseTime = 0;
    private static final double VEL_FILTER = 0.3;

    public volatile boolean Ipornit = false, IntakePornit = false,
            SortingPornit = false, SortingToggle = false,
            Touch = false, trouch = false;
    private volatile double distantare, posU;

    private volatile double DistanceLaTarget = 0;
    private volatile double autoVelocity = 1650;
    private volatile double autoHoodPosition = 0.20;
    public volatile boolean autoShootEnabled = true;
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
        return 0.0000221033 * Math.pow(x, 4) - 0.00805363 * Math.pow(x, 3)
                + 1.09057 * Math.pow(x, 2) - 58.29861 * x + 2525;
    }

    private double EcuatieHoodAngle(double x) {
        return 0.0000260977 * Math.pow(x, 2) - 0.00364962 * x + 0.320;
    }

    private double CalculeVelocitate(double distance) {
        rawVelocity = EcuatieVelocity(distance);
        double velocity = Math.max(minVelocity, Math.min(maxVelocity, rawVelocity));
        return velocity;
    }

    private double CalculareCucUnghi(double distance) {
        rawUnghidPos = EcuatieHoodAngle(distance);
        return Math.max(0.05, Math.min(0.45, rawUnghidPos));
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
        Pose pose = follower.getPose();
        long now = System.currentTimeMillis();
        if (lastPoseTime > 0) {
            double dt = (now - lastPoseTime) / 1000.0;
            if (dt > 0.001) {
                xVelocity += VEL_FILTER * ((pose.getX() - lastPoseX) / dt - xVelocity);
                yVelocity += VEL_FILTER * ((pose.getY() - lastPoseY) / dt - yVelocity);
            }
        }
        lastPoseX = pose.getX();
        lastPoseY = pose.getY();
        lastPoseTime = now;

        double flightTime = calculateFlightTime();
        double spd = Math.hypot(xVelocity, yVelocity);
        double flightScale = Math.min(1.0, spd / 8.0);
        double virtualX = TargetX - xVelocity * flightTime * flightScale;
        double virtualY = TargetY - yVelocity * flightTime * flightScale;
        double dx = virtualX - pose.getX();
        double dy = virtualY - pose.getY();
        DistanceLaTarget = Math.sqrt(dx * dx + dy * dy);

        if (TragereInRange(DistanceLaTarget)) {
            double rawV = CalculeVelocitate(DistanceLaTarget);
            double rawH = CalculareCucUnghi(DistanceLaTarget);
            smoothedVelocity = smoothedVelocity * (1 - VELOCITY_EMA_ALPHA) + rawV * VELOCITY_EMA_ALPHA;
            smoothedHood = smoothedHood * (1 - VELOCITY_EMA_ALPHA) + rawH * VELOCITY_EMA_ALPHA;
            autoVelocity = smoothedVelocity;
            autoHoodPosition = smoothedHood;
        }

        double v1 = Math.abs(m.shooter.getVelocity());
        double tolerance = targetShooterVelocity * velocitateToleranta;
        rpmOK = Math.abs(v1 - targetShooterVelocity) < tolerance;
        distOK = TragereInRange(DistanceLaTarget);

        boolean allReady = rpmOK && distOK && getLoculete() > 0 && !trageShooting;
        if (allReady) {
            if (readySince == 0) readySince = System.currentTimeMillis();
            autoShootReady = (System.currentTimeMillis() - readySince) >= AUTO_SHOOT_DWELL_MS;
        } else {
            readySince = 0;
            autoShootReady = false;
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

        try {
            scula = hardwareMap.get(DcMotorEx.class, "scula");
            frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
            backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
            frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
            backRight = hardwareMap.get(DcMotorEx.class, "backRight");
            bascula = hardwareMap.get(ServoImplEx.class, "bascula");

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
            scula.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            m.shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            telemetry.addData("INIT ERROR", e.getMessage());
            telemetry.update();
        }

        follower = Constants.createFollower(hardwareMap);
        Pose startingPose = new Pose(RobotPozitie.X, RobotPozitie.Y, RobotPozitie.heading);
        follower.setStartingPose(startingPose);
        follower.update();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();

        turelaD = ServoImplExEx.getContinuous(hardwareMap, "turelaD", "turretD");
        turelaD.setEncoderReferenceVoltage(REFERINTA_VOLTAJ_D);
        turelaD.setAngleLimits(LIMITA_STANGA_GRADE, LIMITA_DREAPTA_GRADE);
        turelaD.setDeadzone(TURELA_DEADZONE);
        turelaD.setPosition(0.5);

        turelaS = ServoImplExEx.getContinuous(hardwareMap, "turelaS", "turretS");
        turelaS.setPosition(0.5);

        turelaTargetGrade = 0;

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
        Turela.start();
    }

    private final Thread AutoShootUpdater = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                try { Thread.sleep(20); } catch (InterruptedException e) { break; }
                follower.update();
                ShooterAdaptare();
                targetShooterVelocity = autoVelocity;
                posU = autoHoodPosition;
                m.unghiD.setPosition(posU);
            }
        }
    });

    private final Thread Turela = new Thread(() -> {
        while (!stop) {
            try { Thread.sleep(5); } catch (InterruptedException e) { break; }

            follower.update();
            Pose pose = follower.getPose();

            if (tracking) {
                trackLoop(pose);
            } else {
                double curr = turelaD.getCurrentAngle();
                if (Math.abs(curr) < TURELA_DEADZONE) {
                    turelaD.setPosition(0.5);
                    turelaS.setPosition(0.5);
                } else {
                    double p = clamp(SERVO_KP * (0 - curr), -SERVO_MAX_POWER, SERVO_MAX_POWER);
                    turelaD.setPosition(0.5 - p);
                    turelaS.setPosition(0.5 - p);
                }
                resetTracking();
            }
        }
    });

    private void trackLoop(Pose pose) {
        long now = System.nanoTime();

        if (trackLastTime == 0) {
            trackLastTime = now;
            lastHeading = pose.getHeading();
            trackLastPoseX = pose.getX();
            trackLastPoseY = pose.getY();
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

        trackXVel += TRACK_VEL_FILTER * ((pose.getX() - trackLastPoseX) / dt - trackXVel);
        trackYVel += TRACK_VEL_FILTER * ((pose.getY() - trackLastPoseY) / dt - trackYVel);
        trackLastPoseX = pose.getX();
        trackLastPoseY = pose.getY();

        double spd = Math.hypot(trackXVel, trackYVel);
        double leadScale = Math.min(1.0, spd / 8.0);
        double predictedX = pose.getX() + trackXVel * VEL_LEAD_TIME * leadScale;
        double predictedY = pose.getY() + trackYVel * VEL_LEAD_TIME * leadScale;
        double dx = TargetX - predictedX;
        double dy = TargetY - predictedY;
        double distToTarget = Math.hypot(dx, dy);
        double angleToTarget = Math.atan2(dy, dx);
        double turretRad = normalizeAngle(angleToTarget - pose.getHeading());
        double odomAngle = Math.toDegrees(turretRad) * SCALE_FACTOR;
        odomAngle = clamp(odomAngle, LIMITA_STANGA_GRADE, LIMITA_DREAPTA_GRADE);

        double tangentialVel = trackXVel * Math.sin(angleToTarget) - trackYVel * Math.cos(angleToTarget);
        double bearingRate = tangentialVel / Math.max(distToTarget, MIN_DIST);
        double translationalFF = Math.toDegrees(bearingRate) * SCALE_FACTOR * TRANS_FF_GAIN;

        hybridLimelightVede = false;
        boolean llConn = limelight != null && limelight.isConnected();
        if (llConn) {
            LLResult result = limelight.getLatestResult();
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
        } else {
            llOffset = 0;
            llIntegral = 0;
        }

        double speed = Math.hypot(trackXVel, trackYVel);
        double ffScale = Math.min(1.0, speed / 8.0);
        double targetAngle = odomAngle + llOffset + llIntegral * LL_KI;
        targetAngle += (filteredHeadingRate * GAIN_DEG + translationalFF) * ffScale;
        targetAngle = clamp(targetAngle, LIMITA_STANGA_GRADE, LIMITA_DREAPTA_GRADE);

        double currentAngle = turelaD.getCurrentAngle();
        double preError = Math.abs(targetAngle - currentAngle);
        double adaptiveAlpha;
        if (preError > DISTURBANCE_THRESHOLD) adaptiveAlpha = 0.95;
        else if (preError > 5.0) adaptiveAlpha = 0.85;
        else if (speed < 5.0) adaptiveAlpha = 0.55;
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
        double marginD = LIMITA_DREAPTA_GRADE - currentAngle;
        double marginS = currentAngle - LIMITA_STANGA_GRADE;
        if (marginD < 15) {
            if (power > 0) power = -SERVO_MIN_POWER * ((15 - marginD) / 15.0);
        }
        if (marginS < 15) {
            if (power < 0) power = SERVO_MIN_POWER * ((15 - marginS) / 15.0);
        }
        turelaD.setPosition(0.5 - power);
        turelaS.setPosition(0.5 - power);
    }

    private void resetTracking() {
        trackLastTime = 0;
        lastHeading = 0;
        filteredHeadingRate = 0;
        llOffset = 0;
        llIntegral = 0;
        smoothedTarget = 0;
        trackingInitializat = false;
        hybridLimelightVede = false;
        trackXVel = 0;
        trackYVel = 0;
        trackLastPoseX = 0;
        trackLastPoseY = 0;
        prevMeasurement = 0;
    }

    private double normalizeAngle(double angle) {
        angle = angle % (2 * Math.PI);
        if (angle > Math.PI) angle -= 2 * Math.PI;
        if (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    private final Thread Butoane = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                try { Thread.sleep(10); } catch (InterruptedException e) { break; }
                if (gamepad1.right_bumper) {
                    TargetY += 1;
                }
                if (gamepad1.left_bumper) {
                    TargetY -= 1;
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
        private long jamStart = 0;

        @Override
        public void run() {
            while (!stop) {
                try { Thread.sleep(10); } catch (InterruptedException e) { break; }
                boolean needJamRecovery = false;
                synchronized (blocat) {
                    int loculete = getLoculete();

                    if (Ipornit && !trageShooting && loculete < 3 && !gamepad1.b) {
                        sugere = true;

                        if (intakeJammed) continue;

                        m.intake.setPower(1);

                        double current = m.intake.getCurrent(CurrentUnit.AMPS);
                        if (current > JAM_CURRENT_THRESHOLD) {
                            if (jamStart == 0) jamStart = System.currentTimeMillis();
                            if (System.currentTimeMillis() - jamStart >= JAM_DEBOUNCE_MS) {
                                intakeJammed = true;
                                jamStart = 0;
                                m.intake.setPower(-1);
                                gamepad1.rumble(200);
                                needJamRecovery = true;
                            }
                        } else {
                            jamStart = 0;
                        }

                        if (!needJamRecovery) {
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
                        }
                    } else if (gamepad1.b) {
                        sugere = false;
                        m.intake.setPower(-1);
                    } else {
                        sugere = false;
                        m.intake.setPower(0);
                    }
                }
                if (needJamRecovery) {
                    try { Thread.sleep(JAM_REVERSE_MS); } catch (InterruptedException ignored) {}
                    m.intake.setPower(0);
                    try { Thread.sleep(100); } catch (InterruptedException ignored) {}
                    intakeJammed = false;
                }
            }
        }
    });

    private int[] cPattern = new int[3];
    private static double targetShooterVelocity = 1650;

    private final Thread Shooter = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                try { Thread.sleep(10); } catch (InterruptedException e) { break; }
                synchronized (blocat) {
                    int loculete = getLoculete();

                    if (!trageShooting) {
                        if (loculete >= 2) {
                            m.shooter.setVelocity(targetShooterVelocity);
                            m.shooter2.setVelocity(targetShooterVelocity);
                        } else if (loculete > 0) {
                            m.shooter.setVelocity(targetShooterVelocity * 0.67);
                            m.shooter2.setVelocity(targetShooterVelocity * 0.67);
                        } else {
                            m.shooter.setVelocity(0);
                            m.shooter2.setVelocity(0);
                        }
                    }

                    boolean shootTrigger = gamepad1.y || (autoShootReady && autoShootEnabled && gamepad1.x);
                    if (shootTrigger && loculete > 0 && !sugere && !trageShooting) {
                        trageShooting = true;
                        applyVoltageCompensatedPIDF();

                        if (autoShootEnabled) {
                            ShooterAdaptare();
                            targetShooterVelocity = autoVelocity;

                            if (!TragereInRange(DistanceLaTarget)) {
                                gamepad1.rumble(100);
                                gamepad2.rumble(100);
                            }
                        }

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
                    double recoil = RECOIL_OFFSETS[Math.min(shotsFired, 2)] * velScale;
                    m.unghiD.setPosition(originalPosU + recoil);

                    double target = getTarget(s);
                    if (target != lastPos) {
                        m.sortare.setPosition(target);
                        m.kdf(m.sortareWaitMs(lastPos, target));
                    }
                    waitForShooterReady();
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

        private void waitForShooterReady() {
            double tolerance = targetShooterVelocity * velocitateToleranta;
            double currentVoltage = Math.max(9.0, m.voltageSensor.getVoltage());
            long maxTimeout = targetShooterVelocity >= 2000 ? 500 : 350;
            long timeoutMs = Math.min(maxTimeout, (long)(450 * (voltajeNominale / currentVoltage)));
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
                try { Thread.sleep(2); } catch (InterruptedException ignored) {}
            }
            if (!ready) {
                applyVoltageCompensatedPIDF();
                gamepad1.rumble(100);
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
        telemetry.addData("RPM", rpmOK ? "OK" : "NOT");
        telemetry.addData("DIST", distOK ? "OK" : "NOT");
        telemetry.addData("AUTO-FIRE", autoShootReady ? "READY" : "---");
        telemetry.addData("JAM", intakeJammed ? "JAMMED" : "ok");

        telemetry.addLine("");
        telemetry.addData("Tracking", tracking ? "ON" : "OFF");
        telemetry.addData("Limelight", hybridLimelightVede ? "VEDE" : "---");
        telemetry.addData("Unghi target", String.format("%.1f", turelaTargetGrade));
        telemetry.addData("Encoder D", String.format("%.1f", turelaD.getCurrentAngle()));
        telemetry.addData("LL offset", String.format("%.2f", llOffset));

        telemetry.addLine("");
        telemetry.addData("unghi hood", posU);
        telemetry.addData("distanta intake", distantare);
        telemetry.addData("TargetY", TargetY);
        telemetry.update();
    }

    @Override
    public void stop() {
        stop = true;
        turelaD.setPosition(0.5);
        turelaS.setPosition(0.5);
        m.shooter.setVelocity(0);
        m.shooter2.setVelocity(0);
        m.intake.setPower(0);
        if (limelight != null) limelight.stop();
    }

    public void POWER(double fr1, double bl1, double br1, double fl1) {
        frontRight.setPower(fr1);
        backLeft.setPower(bl1);
        frontLeft.setPower(fl1);
        backRight.setPower(br1);
    }
}
