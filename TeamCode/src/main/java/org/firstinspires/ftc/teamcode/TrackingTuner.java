package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.NouHard.ServoImplExEx;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "Tracking Tuner V6", group = "Tuning")
@Configurable
public class TrackingTuner extends OpMode {

    // --- LL Fusion ---
    public static double ALPHA = 0.30;
    public static double MAX_OFFSET = 35.0;
    public static double DECAY = 0.997;
    public static double LL_LATENCY = 0.02;
    public static double LL_KI = 0.03;
    public static double LL_MAX_INTEGRAL = 15.0;

    // --- Feedforward ---
    public static double GAIN_DEG = 1.0;
    public static double HR_FILTER = 0.30;
    public static double TRANS_FF_GAIN = 0.12;
    public static double VEL_FILTER = 0.22;
    public static double VEL_LEAD_TIME = 0.15;
    public static double MIN_DIST = 12.0;

    // --- Smoothing ---
    public static double T_ALPHA = 0.50;
    public static double DISTURBANCE_THRESHOLD = 17.0;
    public static double STATIONARY_ALPHA = 0.40;

    // --- PD Controller ---
    public static double SERVO_KP = 0.0030;
    public static double SERVO_KD = 0.0005;
    public static double DERIV_FILTER = 0.8;
    public static double SERVO_MIN_POWER = 0.05;
    public static double SERVO_MAX_POWER = 0.50;
    public static double BOOST_MAX_POWER = 0.60;
    public static double TURELA_DEADZONE = 2.0;

    // --- Offset & Target ---
    public static double TURELA_OFFSET_DEG = -15.0;
    public static double TARGET_X = 0;
    public static double TARGET_Y = 144;

    // --- Hardware constants ---
    private static final double LIMITA_STANGA_GRADE = -221.5;
    private static final double LIMITA_DREAPTA_GRADE = 191.1;
    private static final double REFERINTA_VOLTAJ_D = 0.5120;
    private static final double SCALE_FACTOR = 2.292;

    private DcMotorEx frontRight, frontLeft, backRight, backLeft;
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;
    private Follower follower;
    private ServoImplExEx turelaD, turelaS;

    private volatile boolean stop = false;
    private volatile boolean tracking = false;
    private boolean trackTogglePrev = false;

    private long trackLastTime = 0;
    private double lastHeading = 0;
    private double filteredHeadingRate = 0;
    private double llOffset = 0;
    private double llIntegral = 0;
    private double smoothedTarget = 0;
    private boolean trackingInitializat = false;
    private double xVelocity = 0;
    private double yVelocity = 0;
    private double lastPoseX = 0;
    private double lastPoseY = 0;
    private double prevMeasurement = 0;
    private double prevFilteredDeriv = 0;

    private volatile double telOdomAngle = 0;
    private volatile double telTargetAngle = 0;
    private volatile double telCurrentAngle = 0;
    private volatile double telError = 0;
    private volatile double telPower = 0;
    private volatile double telTx = 0;
    private volatile double telOffset = 0;
    private volatile double telIntegral = 0;
    private volatile boolean telTagVisible = false;
    private volatile double telHeadingRate = 0;
    private volatile double telTransFF = 0;
    private volatile double telAdaptiveAlpha = 0;
    private volatile double telEffectiveMaxPower = 0;
    private volatile double telDerivative = 0;
    private volatile double telFilteredDeriv = 0;
    private volatile double telVelX = 0;
    private volatile double telVelY = 0;
    private volatile double telX = 0, telY = 0, telH = 0;
    private volatile double telSpeed = 0;
    private volatile double telFfScale = 0;

    private Thread turelaThread, chassisThread;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.recalibrateIMU();
        ElapsedTime calibTimer = new ElapsedTime();
        while (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                && calibTimer.milliseconds() < 2000) {
        }

        Pose startPose = new Pose(RobotPozitie.X, RobotPozitie.Y, RobotPozitie.heading);
        follower.setStartingPose(startPose);
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
    }

    @Override
    public void init_loop() {
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.addData("Heading", "%.2f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addLine("X = toggle tracking | Stickuri = drive");
        telemetry.update();
    }

    @Override
    public void start() {
        stop = false;
        tracking = false;
        resetTracking();

        turelaThread = new Thread(() -> {
            while (!stop) {
                try { Thread.sleep(5); } catch (InterruptedException e) { break; }

                follower.update();
                Pose pose = follower.getPose();
                telX = pose.getX();
                telY = pose.getY();
                telH = pose.getHeading();

                if (tracking) {
                    trackLoop(pose);
                } else {
                    double curr = turelaD.getCurrentAngle();
                    telCurrentAngle = curr;
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
            turelaD.setPosition(0.5);
            turelaS.setPosition(0.5);
        });

        chassisThread = new Thread(() -> {
            while (!stop) {
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;

                double fl = y + x + rx;
                double bl = y - x + rx;
                double br = y + x - rx;
                double fr = y - x - rx;

                double max = Math.max(Math.max(abs(fl), abs(fr)), Math.max(abs(bl), abs(br)));
                if (max > 1) { fl /= max; fr /= max; bl /= max; br /= max; }

                double sm = gamepad1.left_trigger > 0 ? 1.5 : 0.7;
                frontLeft.setPower(fl / sm);
                frontRight.setPower(fr / sm);
                backLeft.setPower(bl / sm);
                backRight.setPower(br / sm);

                try { Thread.sleep(5); } catch (InterruptedException e) { break; }
            }
        });

        turelaThread.start();
        chassisThread.start();
    }

    private void trackLoop(Pose pose) {
        long now = System.nanoTime();

        if (trackLastTime == 0) {
            trackLastTime = now;
            lastHeading = pose.getHeading();
            lastPoseX = pose.getX();
            lastPoseY = pose.getY();
            prevMeasurement = turelaD.getCurrentAngle();
            prevFilteredDeriv = 0;
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
        telHeadingRate = filteredHeadingRate;

        xVelocity += VEL_FILTER * ((pose.getX() - lastPoseX) / dt - xVelocity);
        yVelocity += VEL_FILTER * ((pose.getY() - lastPoseY) / dt - yVelocity);
        lastPoseX = pose.getX();
        lastPoseY = pose.getY();
        telVelX = xVelocity;
        telVelY = yVelocity;

        double spd = Math.hypot(xVelocity, yVelocity);
        double leadScale = Math.min(1.0, spd / 8.0);
        double predictedX = pose.getX() + xVelocity * VEL_LEAD_TIME * leadScale;
        double predictedY = pose.getY() + yVelocity * VEL_LEAD_TIME * leadScale;
        double dx = TARGET_X - predictedX;
        double dy = TARGET_Y - predictedY;
        double distToTarget = Math.hypot(dx, dy);
        double angleToTarget = Math.atan2(dy, dx);
        double turretRad = normalizeAngle(angleToTarget - pose.getHeading());
        double odomAngle = Math.toDegrees(turretRad) * SCALE_FACTOR;
        odomAngle = clamp(odomAngle, LIMITA_STANGA_GRADE, LIMITA_DREAPTA_GRADE);
        telOdomAngle = odomAngle;

        double tangentialVel = xVelocity * Math.sin(angleToTarget) - yVelocity * Math.cos(angleToTarget);
        double bearingRate = tangentialVel / Math.max(distToTarget, MIN_DIST);
        double translationalFF = Math.toDegrees(bearingRate) * SCALE_FACTOR * TRANS_FF_GAIN;
        telTransFF = translationalFF;

        boolean limelightConnected = limelight != null && limelight.isConnected();
        telTagVisible = false;
        telTx = 0;
        if (limelightConnected) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    double tx = result.getTx();
                    telTagVisible = true;
                    telTx = tx;
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
        }
        if (!telTagVisible) {
            llOffset *= DECAY;
            llIntegral *= 0.95;
        }
        telOffset = llOffset;
        telIntegral = llIntegral;

        double speed = spd;
        double ffScale = Math.min(1.0, speed / 8.0);
        telSpeed = speed;
        telFfScale = ffScale;
        double targetAngle = odomAngle + llOffset + llIntegral * LL_KI + TURELA_OFFSET_DEG;
        targetAngle += (filteredHeadingRate * GAIN_DEG + translationalFF) * ffScale;
        targetAngle = clamp(targetAngle, LIMITA_STANGA_GRADE, LIMITA_DREAPTA_GRADE);

        double currentAngle = turelaD.getCurrentAngle();
        telCurrentAngle = currentAngle;
        double preError = Math.abs(targetAngle - currentAngle);
        double adaptiveAlpha;
        if (preError > DISTURBANCE_THRESHOLD) adaptiveAlpha = 0.95;
        else if (preError > 5.0) adaptiveAlpha = 0.85;
        else if (speed < 5.0) adaptiveAlpha = STATIONARY_ALPHA;
        else adaptiveAlpha = T_ALPHA;
        telAdaptiveAlpha = adaptiveAlpha;

        if (!trackingInitializat) {
            smoothedTarget = targetAngle;
            trackingInitializat = true;
        } else {
            smoothedTarget += adaptiveAlpha * (targetAngle - smoothedTarget);
        }
        telTargetAngle = smoothedTarget;

        double posError = smoothedTarget - currentAngle;
        telError = posError;
        double measurement = currentAngle;
        double posDerivative = -(measurement - prevMeasurement) / dt;
        prevMeasurement = measurement;
        telDerivative = posDerivative;

        double absError = Math.abs(posError);
        if (absError < TURELA_DEADZONE && Math.abs(posDerivative) < 50.0) {
            turelaD.setPosition(0.5);
            turelaS.setPosition(0.5);
            telPower = 0;
            return;
        }

        double effectiveMaxPower;
        if (absError > 20.0) effectiveMaxPower = BOOST_MAX_POWER;
        else if (absError > 10.0) effectiveMaxPower = SERVO_MAX_POWER + 0.05;
        else effectiveMaxPower = SERVO_MAX_POWER;
        telEffectiveMaxPower = effectiveMaxPower;

        double filteredDeriv = DERIV_FILTER * prevFilteredDeriv + (1.0 - DERIV_FILTER) * posDerivative;
        prevFilteredDeriv = filteredDeriv;
        telFilteredDeriv = filteredDeriv;
        double power = SERVO_KP * posError + SERVO_KD * filteredDeriv;
        if (absError < TURELA_DEADZONE * 2) {
            double ramp = (absError - TURELA_DEADZONE) / TURELA_DEADZONE;
            power *= Math.max(0, ramp);
        } else if (Math.abs(power) < SERVO_MIN_POWER && absError > TURELA_DEADZONE) {
            power = Math.signum(posError) * SERVO_MIN_POWER;
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
        telPower = power;
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
        xVelocity = 0;
        yVelocity = 0;
        lastPoseX = 0;
        lastPoseY = 0;
        prevMeasurement = 0;
        prevFilteredDeriv = 0;
    }

    @Override
    public void loop() {
        boolean xPressed = gamepad1.x;
        if (xPressed && !trackTogglePrev) {
            tracking = !tracking;
            if (tracking) resetTracking();
        }
        trackTogglePrev = xPressed;

        telemetry.addLine("=== TRACKING TUNER V6 ===");
        telemetry.addData("Tracking", tracking ? (telTagVisible ? "ODOM+LL" : "ODOM") : "OFF");
        telemetry.addLine("");

        telemetry.addLine("--- LIMELIGHT ---");
        telemetry.addData("Tag vizibil", telTagVisible ? "DA" : "NU");
        telemetry.addData("tx", "%.2f", telTx);
        telemetry.addData("LL offset", "%.2f", telOffset);
        telemetry.addData("LL integral", "%.3f", telIntegral);
        telemetry.addLine("");

        telemetry.addLine("--- TURELA ---");
        telemetry.addData("Odom angle", "%.1f", telOdomAngle);
        telemetry.addData("Target (smoothed)", "%.1f", telTargetAngle);
        telemetry.addData("Current angle", "%.1f", telCurrentAngle);
        telemetry.addData("Eroare", "%.1f", telError);
        telemetry.addData("Power", "%.3f", telPower);
        telemetry.addData("Raw Derivative", "%.1f", telDerivative);
        telemetry.addData("Filtered Derivative", "%.1f", telFilteredDeriv);
        telemetry.addData("Adaptive alpha", "%.2f", telAdaptiveAlpha);
        telemetry.addData("Max power", "%.2f", telEffectiveMaxPower);
        telemetry.addLine("");

        telemetry.addLine("--- FEEDFORWARD ---");
        telemetry.addData("Heading rate", "%.2f rad/s", telHeadingRate);
        telemetry.addData("Trans FF", "%.2f", telTransFF);
        telemetry.addData("Speed", "%.1f", telSpeed);
        telemetry.addData("FF scale", "%.2f", telFfScale);
        telemetry.addData("Vel X/Y", "%.1f / %.1f", telVelX, telVelY);
        telemetry.addLine("");

        telemetry.addLine("--- POZITIE ---");
        telemetry.addData("X", "%.1f", telX);
        telemetry.addData("Y", "%.1f", telY);
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(telH));
        telemetry.addData("Target", "(%.0f, %.0f)", TARGET_X, TARGET_Y);
        telemetry.addLine("");

        telemetry.addLine("--- CONSTANTE (FTC Dashboard) ---");
        telemetry.addData("KP / KD", "%.4f / %.5f", SERVO_KP, SERVO_KD);
        telemetry.addData("DERIV_FILTER", "%.2f", DERIV_FILTER);
        telemetry.addData("DEADZONE", "%.1f", TURELA_DEADZONE);
        telemetry.addData("ALPHA / DECAY", "%.3f / %.3f", ALPHA, DECAY);
        telemetry.addData("T_ALPHA / STAT_ALPHA", "%.2f / %.2f", T_ALPHA, STATIONARY_ALPHA);
        telemetry.addData("GAIN_DEG / TRANS_FF", "%.2f / %.2f", GAIN_DEG, TRANS_FF_GAIN);
        telemetry.addData("VEL_LEAD / LL_KI", "%.2f / %.3f", VEL_LEAD_TIME, LL_KI);
        telemetry.addData("OFFSET_DEG", "%.1f", TURELA_OFFSET_DEG);
        telemetry.update();
    }

    @Override
    public void stop() {
        stop = true;
        if (turelaThread != null) turelaThread.interrupt();
        if (chassisThread != null) chassisThread.interrupt();
        turelaD.setPosition(0.5);
        turelaS.setPosition(0.5);
        if (limelight != null) limelight.stop();
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
}
