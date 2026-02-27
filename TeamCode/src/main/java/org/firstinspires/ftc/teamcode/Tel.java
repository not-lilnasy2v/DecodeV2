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

    // 50 mov // 60 - > verde
    private DcMotorEx frontRight, frontLeft, backRight, backLeft,scula;
    private ServoImplEx bascula;
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;
    public Follower follower;
    volatile boolean stop;
    double sm = 1;
    double max = 0;
    double FL, BL, BR, FR;
    sistemeTeleOp m = new sistemeTeleOp();
    private static double TargetX = 3;
    private static double TargetY = 144;
    private static double IDLE_RATIO = 0.67;
    private volatile double currentY, currentX, currentH;
    private ElapsedTime pidTimer = new ElapsedTime();

    private ServoImplExEx turelaD;
    private ServoImplExEx turelaS;

    private static final double LIMITA_STANGA_GRADE = -219.9;
    private static final double LIMITA_DREAPTA_GRADE = 218.3;
    private static final double REFERINTA_VOLTAJ_D = 0.3730;
    private static final double TURELA_DEADZONE = 2.0;
    private static final double SCALE_FACTOR = 2.435;

    private volatile double llIntegral = 0;
    private volatile double llLastError = 0;
    private ElapsedTime llTimer = new ElapsedTime();
    private static double LL_KP = 0.2;
    private static double LL_KI = 0.01;
    private static double LL_KD = 0.002;
    private static double LL_MAX_INTEGRAL = 30;
    private static double LL_MAX_CORRECTION = 20.0;

    private static double FF_GAIN_DEG = 1.5;
    private volatile double lastTx = 0;
    private volatile boolean limelightVede = false;
    private volatile double lastHeading = 0;
    private volatile long lastHeadingTime = 0;

    private static double ODOM_WEIGHT = 0.05;
    private volatile double turelaTargetGrade = 0;

    private static double POS_KP = 0.00085;
    private static double POS_KD = 0.00005;
    private static double POS_MIN_POWER = 0.04;
    private static double POS_MAX_POWER = 0.15;
    private volatile double posLastError = 0;
    private ElapsedTime posTimer = new ElapsedTime();

    private volatile int settledFrames = 0;
    private static double SETTLED_THRESHOLD =  3.0;
    private static int SETTLED_REQUIRED = 3;

    private static double voltajeNominale = 12.68;
    public volatile boolean turelaTracking = false, tracking = false, Ipornit = false, IntakePornit = false, SortingPornit = false, SortingToggle = false, Touch = false, trouch = false;
    private volatile double distantare, posU;
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

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        m.shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m.shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        scula.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        follower = Constants.createFollower(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.recalibrateIMU();
        ElapsedTime calibrationTimer = new ElapsedTime();
        while (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                && calibrationTimer.milliseconds() < 1000) {
        }

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

    public void start() {
        follower.update();
        pidTimer.reset();
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
                try { Thread.sleep(10); } catch (InterruptedException e) { break; }
                posU = m.unghiD.getPosition();

                if (gamepad1.dpad_right) {
                    targetShooterVelocity = 1950;
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
                    }
                    turelaTracking = dpad_right1;
                }
                if(gamepad2.left_bumper && !gamepad2.right_bumper){
                    TargetY -= 1;
                }
                if(gamepad2.right_bumper&& !gamepad2.left_bumper) {
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
                if(Touch != gamepad1_touch){
                    if(gamepad1.touchpad){
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

                if (gamepad2.left_bumper && !sugere && !trageShooting) {
                    double lastPos = m.sortare.getPosition();
                    for (int i = 0; i < 3; i++) {
                        if (slotOcupat[i]) {
                            double targetPos;
                            if (i == 0) targetPos = Pozitii.luarea1;
                            else if (i == 1) targetPos = Pozitii.luarea2;
                            else targetPos = Pozitii.luarea3;

                            m.sortare.setPosition(targetPos);
                            double dist = Math.abs(targetPos - lastPos);
                            int moveWait = (int)(dist * 400) + 100;
                            m.kdf(moveWait);

                            m.resetareDetection();
                            slotColor[i] = m.detecteazaBiloaca();
                            lastPos = targetPos;

                            if (slotColor[i] != -1) {
                                gamepad2.rumble(150);
                            }
                        }
                    }
                    m.sortare.setPosition(Pozitii.luarea1);
                    gamepad2.rumble(300);
                }
            }
        }
    });

    private double calculOdometre() {
        double dx = TargetX - currentX;
        double dy = TargetY - currentY;
        double angleToTarget = Math.atan2(dy, dx);
        double turretAngleRad = angleToTarget - currentH;
        while (turretAngleRad > Math.PI) turretAngleRad -= 2 * Math.PI;
        while (turretAngleRad < -Math.PI) turretAngleRad += 2 * Math.PI;
        double turretAngleDeg = Math.toDegrees(turretAngleRad) * SCALE_FACTOR;
        return Math.max(LIMITA_STANGA_GRADE, Math.min(LIMITA_DREAPTA_GRADE, turretAngleDeg));
    }

    private double calculHybrid() {
        double odomAngle = calculOdometre();
        if (!limelightVede) {
            llIntegral = 0;
            llLastError = 0;
            return odomAngle;
        }
        double dt = llTimer.seconds();
        llTimer.reset();
        dt = Math.max(0.005, dt);
        double error = -lastTx * SCALE_FACTOR;
        llIntegral += error * dt;
        llIntegral = Math.max(-LL_MAX_INTEGRAL, Math.min(LL_MAX_INTEGRAL, llIntegral));
        double derivative = (error - llLastError) / dt;
        llLastError = error;
        double correction = LL_KP * error + LL_KI * llIntegral + LL_KD * derivative;
        correction = Math.max(-LL_MAX_CORRECTION, Math.min(LL_MAX_CORRECTION, correction));
        double currentAngle = turelaD.getCurrentAngle();
        double baseAngle = ODOM_WEIGHT * odomAngle + (1 - ODOM_WEIGHT) * currentAngle;
        return Math.max(LIMITA_STANGA_GRADE, Math.min(LIMITA_DREAPTA_GRADE, baseAngle + correction));
    }

    private void applyTurelaControl(double targetAngle) {
        turelaTargetGrade = targetAngle;
        double currentAngle = turelaD.getCurrentAngle();
        double error = targetAngle - currentAngle;
        double dt = posTimer.seconds();
        posTimer.reset();
        dt = Math.max(0.005, dt);
        double derivative = (error - posLastError) / dt;
        posLastError = error;
        if (Math.abs(error) < TURELA_DEADZONE) {
            turelaD.setPosition(0.5);
            turelaS.setPosition(0.5);
            return;
        }
        double power = POS_KP * error + POS_KD * derivative;
        if (Math.abs(power) < POS_MIN_POWER) {
            power = Math.signum(power) * POS_MIN_POWER;
        }
        power = Math.max(-POS_MAX_POWER, Math.min(POS_MAX_POWER, power));
        turelaD.setPosition(0.5 - power);
        turelaS.setPosition(0.5 - power);
    }

    private void centreazaTurela() {
        double currentAngle = turelaD.getCurrentAngle();
        if (Math.abs(currentAngle) < TURELA_DEADZONE) {
            turelaD.setPosition(0.5);
            turelaS.setPosition(0.5);
            return;
        }
        double power = POS_KP * (0 - currentAngle);
        if (Math.abs(power) < POS_MIN_POWER) {
            power = Math.signum(power) * POS_MIN_POWER;
        }
        power = Math.max(-POS_MAX_POWER, Math.min(POS_MAX_POWER, power));
        turelaD.setPosition(0.5 - power);
        turelaS.setPosition(0.5 - power);
    }

    private void setTurelaManual(double putere) {
        turelaD.setPosition(0.5 - putere);
        turelaS.setPosition(0.5 - putere);
    }

    private final Thread Turela = new Thread(() -> {
        while (!stop) {
            try { Thread.sleep(10); } catch (InterruptedException e) { break; }

            follower.update();
            Pose currentPose = follower.getPose();
            currentX = currentPose.getX();
            currentY = currentPose.getY();
            currentH = currentPose.getHeading();

            long now = System.nanoTime();
            double headingRate = 0;
            if (lastHeadingTime != 0) {
                double hdt = (now - lastHeadingTime) / 1_000_000_000.0;
                if (hdt > 0.005) {
                    headingRate = (currentH - lastHeading) / hdt;
                }
            }
            lastHeading = currentH;
            lastHeadingTime = now;

            updateLimelightData();

            if (tracking) {
                double targetAngle = calculHybrid();
                targetAngle += headingRate * FF_GAIN_DEG;
                targetAngle = Math.max(LIMITA_STANGA_GRADE, Math.min(LIMITA_DREAPTA_GRADE, targetAngle));
                applyTurelaControl(targetAngle);

                if (limelightVede && Math.abs(lastTx) < SETTLED_THRESHOLD) {
                    settledFrames++;
                } else {
                    settledFrames = 0;
                }
            } else if (!trouch) {
                centreazaTurela();
                resetPID();
            } else {
                double manualPutere = 0;
                if (gamepad1.left_bumper) {
                    manualPutere = -POS_MAX_POWER * 2;
                } else if (gamepad1.right_bumper) {
                    manualPutere = POS_MAX_POWER * 2;
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

    private void updateLimelightData() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                lastTx = result.getTx();
                limelightVede = true;
                return;
            }
        }
        limelightVede = false;
    }

    private void resetPID() {
        llIntegral = 0;
        llLastError = 0;
        llTimer.reset();
        posLastError = 0;
        posTimer.reset();
        settledFrames = 0;
    }

    private boolean turelaSettled() {
        if (!limelightVede) return true;
        return settledFrames >= SETTLED_REQUIRED;
    }

    private final Thread Sortare = new Thread(new Runnable() {
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
                    distantare = m.distanta.getDistance(DistanceUnit.CM);

                    if (Ipornit && !trageShooting && loculete < 3 && !gamepad1.b) {
                        sugere = true;
                        m.intake.setPower(1);
                        bascula.setPosition(Pozitii.sede);

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
                            }

                            if (getLoculete() == 3) {
                                Ipornit = false;
                                m.intake.setPower(0);
                                gamepad1.rumble(2000);

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
    private double targetShooterVelocity = 1550;

    private final Thread Shooter = new Thread(new Runnable() {
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

                    if (gamepad1.y && loculete > 0 && !sugere && !trageShooting) {
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
            long timeoutMs = Math.min(400, (long)(350 * (voltajeNominale / currentVoltage)));
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
            }
            if (!ready) {
                applyVoltageCompensatedPIDF();
                gamepad1.rumble(100);
            }
        }

        private void shootPattern() {
            double lastPos = m.sortare.getPosition();
            scula.setPower(-1);
            bascula.setPosition(Pozitii.sede);

            for (int step = 0; step < 3; step++) {
                int need = cPattern[step];
                int slotShoot = GasestePattern(need);

                if (slotShoot == -1) break;

                double target = getTarget(slotShoot);
                m.sortare.setPosition(target);

                double dist = Math.abs(target - lastPos);
                int moveWait = (int) (dist * 350) + 70;
                m.kdf(moveWait);

                applyVoltageCompensatedPIDF();
                waitForShooterReady();
                bascula.setPosition(Pozitii.lansare);
                m.kdf(120);
                bascula.setPosition(Pozitii.sede);
                m.kdf(30);

                slotOcupat[slotShoot] = false;
                slotColor[slotShoot] = -1;
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
            scula.setPower(-1);
            bascula.setPosition(Pozitii.lansareRapid);

            int[] order = {0, 2, 1};
            for (int s : order) {
                if (slotOcupat[s]) {
                    double target = getTarget(s);
                    m.sortare.setPosition(target);

                    m.kdf(95);

                    applyVoltageCompensatedPIDF();
                    waitForShooterReady();

                    slotOcupat[s] = false;
                    slotColor[s] = -1;
                    lastPos = target;
                }
            }
        }

        private double getTarget(int slot) {
            if (slot == 0) return Pozitii.aruncare1;
            if (slot == 1) return Pozitii.aruncare2;
            return Pozitii.aruncare3;
        }
    });

    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                try {
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    break;
                }
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;

                FL = (y + x + rx);
                BL = (y - x + rx);
                BR = (y + x - rx);
                FR = (y - x - rx);

                if(abs(FL)> max){
                    max = abs(FL);
                }
                if (abs(FR) > max) {
                    max = abs(FR);
                }
                if (abs(BL) > max) {
                    max = abs(BL);
                }
                if (abs(BR) > max) {
                    max = abs(BR);
                }
                if (max > 1) {
                    FL /= max;
                    FR /= max;
                    BL /= max;
                    BR /= max;
                }
                if (gamepad1.right_trigger > 0) {
                    sm = 2;
                } else {
                    if (gamepad1.left_trigger > 0) {
                        sm = 5;
                    } else {
                        sm = 1;
                    }
                }
                POWER(FR / sm, FL / sm, BR / sm, BL / sm);

            }
        }
    });

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
        telemetry.addData("Tracking", tracking ? (limelightVede ? "HYBRID" : "ODOM") : "OFF");
        telemetry.addData("tx", "%.2f", lastTx);
        telemetry.addData("Settled", turelaSettled() ? "DA" : "NU");
        telemetry.addData("Manual (trouch)", trouch ? "ON" : "OFF");
        telemetry.addLine("");
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("y", currentY);
        telemetry.addData("TargetY", TargetY);
        telemetry.update();
    }

    public void POWER(double df1, double sf1, double ds1, double ss1) {
        frontRight.setPower(df1);
        backLeft.setPower(ss1);
        frontLeft.setPower(sf1);
        backRight.setPower(ds1);
    }

//    private double normalizeAngle(double angle) {
//        while (angle > Math.PI) angle -= 2 * Math.PI;
//        while (angle < -Math.PI) angle += 2 * Math.PI;
//        return angle;
//    }
}
