package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.NouHard.ServoImplExEx;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;

/**
 * Tracking Test — 3 moduri avansate de turret tracking:
 *
 * MODE 1 (ODOM): Odometry-only cu double-EMA heading rate, velocity+acceleration
 *   prediction, translational feedforward, heading drift compensation,
 *   adaptive target smoothing. Zero dependenta de camera.
 *
 * MODE 2 (LIMELIGHT): Limelight-only cu EMA-filtered tx, latency compensation,
 *   PID cu derivative-on-measurement si anti-windup, tx velocity prediction,
 *   dynamic gains, graceful target-loss handling.
 *
 * MODE 3 (HYBRID): Odometry base + Limelight offset correction cu EMA accumulator,
 *   conditional integral, confidence-weighted fusion, all feedforward,
 *   seamless handoff la pierderea/regasirea target-ului.
 *
 * Comanda: right_bumper GP2 = cycle modes, dpad_down = recalibrate IMU
 */
@TeleOp(name = "Tracking Test")
@Configurable
public class TelTrackingTest extends OpMode {

    private static final double LIMITA_STANGA = -219.9;
    private static final double LIMITA_DREAPTA = 218.3;
    private static final double REFERINTA_VOLTAJ = 0.3730;
    private static final double TURELA_DEADZONE = 2.0;
    private static double SCALE_FACTOR = 2.435;
    private static double TURELA_OFFSET_DEG = -15.0;
    private static double TargetX = 0;
    private static double TargetY = 144;

    /// Odometru singur
    private static double HR_EMA_FAST = 0.30;
    private static double HR_EMA_SLOW = 0.15;
    // Velocity EMA
    private static double VEL_EMA = 0.22;
    // Acceleration EMA
    private static double ACCEL_EMA = 0.20;
    // Velocity prediction
    private static double VEL_LEAD_TIME = 0.20;
    private static double ACCEL_LEAD_TIME = 0.05;
    // Feedforward
    private static double H_FF_GAIN = 1.2;
    private static double TRANS_FF_GAIN = 0.5;
    private static double MIN_DIST = 12.0;
    // Adaptive target smoothing
    private static double SMOOTH_FAST = 0.95;
    private static double SMOOTH_MEDIUM = 0.85;
    private static double SMOOTH_SLOW = 0.25;
    private static double SMOOTH_NORMAL = 0.45;
    private static double DISTURB_THRESH = 17.0;
    // Heading drift compensation
    private static double DRIFT_WINDOW_S = 2.0;
    private static double DRIFT_MAX_COMP = 0.5;

    /// limelight singur
    private static double TX_EMA = 0.35;
    private static double LL_LATENCY = 0.02;
    private static double TX_OUTLIER_LIMIT = 20.0;
    // PID
    private static double LL_KP = 1.2;
    private static double LL_KI = 0.03;
    private static double LL_KD = 0.08;
    private static double LL_MAX_INTEGRAL = 15.0;
    private static double LL_MAX_CORRECTION = 30.0;
    // TX velocity prediction
    private static double TX_VEL_EMA = 0.25;
    private static double TX_VEL_LEAD = 0.04;
    // Target loss
    private static double LOSS_DECAY = 0.96;
    private static double LOSS_CENTER_RATE = 0.02;
//hybrid
    private static double OFFSET_EMA = 0.25;
    private static double MAX_OFFSET = 35.0;
    private static double OFFSET_DECAY_RATE = 0.97;
    private static double OFFSET_INTEGRAL_KI = 0.03;
    private static double OFFSET_MAX_INTEGRAL = 15.0;
//toate modurile
    private static double POS_KP = 0.0025;
    private static double POS_KD = 0.00010;
    private static double POS_MIN_POWER = 0.05;
    private static double POS_MAX_POWER = 0.45;
    private static double POS_BOOST_POWER = 0.55;
    private DcMotorEx frontRight, frontLeft, backRight, backLeft;
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;
    public Follower follower;
    private ServoImplExEx turelaD, turelaS;
    private List<LynxModule> hubs;
    volatile boolean stop;
    private volatile double poseX, poseY, poseH;
    // Double-EMA heading rate
    private double hrFast = 0, hrSlow = 0;
    private double prevHeading = 0;
    // Velocity (EMA filtered)
    private double velX = 0, velY = 0;
    private double prevPoseX = 0, prevPoseY = 0;
    // Acceleration (EMA filtered)
    private double accX = 0, accY = 0;
    private double prevVelX = 0, prevVelY = 0;
    // Timing
    private long lastSensorNanos = 0;

    private volatile boolean llVisible = false;
    private volatile double rawTx = 0;
    private double filteredTx = 0;
    private double prevFilteredTx = 0;
    private double txVelocity = 0;
    private double timeSinceLLSeen = 0;
    // 3-sample median filter buffer (rejects single-frame outlier spikes)
    private final double[] txMedianBuf = {0, 0, 0};
    private int txMedianIdx = 0;

    private volatile int trackingMode = 0; // 0=OFF, 1=ODOM, 2=LL, 3=HYBRID

    // Mode 1 (ODOM) state
    private double odomSmoothed = 0;
    private boolean odomInit = false;
    private double driftAccum = 0;
    private double driftWindowStart = 0;
    private double driftHeadingAtStart = 0;

    // Mode 2 (LL) state
    private double llIntegral = 0;
    private double llPrevMeas = 0;
    private double llLastTarget = 0;
    private boolean llEverSeen = false;

    // Mode 3 (HYBRID) state
    private double hybOffset = 0;
    private double hybIntegral = 0;
    private double hybSmoothed = 0;
    private boolean hybInit = false;

    // Position controller state
    private double posPrevMeas = 0;
    private volatile double turelaTarget = 0;

    // UI state
    private volatile boolean modeToggle = false;
    private volatile boolean Touch = false, trouch = false;
    double sm = 1;

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double normalizeAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private static double median3(double a, double b, double c) {
        if (a > b) { double t = a; a = b; b = t; }
        if (b > c) { b = c; }
        return Math.max(a, b);
    }
    private void updateSensors() {
        long now = System.nanoTime();
        if (lastSensorNanos == 0) {
            lastSensorNanos = now;
            Pose p = follower.getPose();
            poseX = p.getX(); poseY = p.getY(); poseH = p.getHeading();
            prevHeading = poseH;
            prevPoseX = poseX; prevPoseY = poseY;
            return;
        }

        double dt = (now - lastSensorNanos) / 1_000_000_000.0;
        dt = Math.max(0.002, Math.min(0.1, dt));
        lastSensorNanos = now;

        Pose p = follower.getPose();
        poseX = p.getX(); poseY = p.getY(); poseH = p.getHeading();

        // --- Double-EMA heading rate ---
        // Stage 1: fast response, tracks heading changes quickly
        double headDiff = normalizeAngle(poseH - prevHeading);
        prevHeading = poseH;
        double rawHR = headDiff / dt;
        hrFast += HR_EMA_FAST * (rawHR - hrFast);
        // Stage 2: slow, smooths out the fast signal for noise-free feedforward
        hrSlow += HR_EMA_SLOW * (hrFast - hrSlow);

        // --- Velocity (EMA filtered) ---
        double rawVX = (poseX - prevPoseX) / dt;
        double rawVY = (poseY - prevPoseY) / dt;
        prevPoseX = poseX; prevPoseY = poseY;
        velX += VEL_EMA * (rawVX - velX);
        velY += VEL_EMA * (rawVY - velY);

        // --- Acceleration (EMA filtered) — derivative of velocity ---
        double rawAX = (velX - prevVelX) / dt;
        double rawAY = (velY - prevVelY) / dt;
        prevVelX = velX; prevVelY = velY;
        accX += ACCEL_EMA * (rawAX - accX);
        accY += ACCEL_EMA * (rawAY - accY);

        // --- Limelight ---
        updateLimelight(dt);

        // --- Heading drift compensation ---
        updateDriftCompensation(dt);
    }

    private void updateLimelight(double dt) {
        LLResult result = limelight.getLatestResult();
        boolean seenThisFrame = false;
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
            if (fids != null && !fids.isEmpty()) {
                double tx = result.getTx();
                // Outlier rejection — ignore extreme readings
                if (Math.abs(tx) < TX_OUTLIER_LIMIT) {
                    rawTx = tx;
                    seenThisFrame = true;
                    llEverSeen = true;
                    timeSinceLLSeen = 0;
                    // Median filter (3 samples) → rejects single-frame spikes
                    txMedianBuf[txMedianIdx] = rawTx;
                    txMedianIdx = (txMedianIdx + 1) % 3;
                    double medianTx = median3(txMedianBuf[0], txMedianBuf[1], txMedianBuf[2]);
                    // EMA filter on median-filtered tx
                    filteredTx += TX_EMA * (medianTx - filteredTx);
                    // TX velocity (how fast tx is changing)
                    double rawTxVel = (filteredTx - prevFilteredTx) / Math.max(dt, 0.005);
                    prevFilteredTx = filteredTx;
                    txVelocity += TX_VEL_EMA * (rawTxVel - txVelocity);
                }
            }
        }
        llVisible = seenThisFrame;
        if (!seenThisFrame) {
            timeSinceLLSeen += dt;
            // Decay tx velocity when not seeing
            txVelocity *= 0.9;
        }
    }

    private void updateDriftCompensation(double dt) {
        double elapsed = (System.nanoTime() / 1_000_000_000.0) - driftWindowStart;
        if (elapsed >= DRIFT_WINDOW_S) {
            // Compute drift rate over the window
            double headingChange = normalizeAngle(poseH - driftHeadingAtStart);
            double driftRate = Math.toDegrees(headingChange) / elapsed;
            // Only compensate small, consistent drift (not intentional rotation)
            double speed = Math.hypot(velX, velY);
            double rotRate = Math.abs(Math.toDegrees(hrSlow));
            if (speed < 5.0 && rotRate < 5.0) {
                driftAccum = clamp(driftRate * DRIFT_MAX_COMP, -2.0, 2.0);
            } else {
                driftAccum *= 0.9;
            }
            // Reset window
            driftWindowStart = System.nanoTime() / 1_000_000_000.0;
            driftHeadingAtStart = poseH;
        }
    }

    // =====================================================================
    // MODE 1: ODOMETRY-ONLY TRACKING
    //
    // Pipeline:
    // 1. Predict future position (velocity + acceleration extrapolation)
    // 2. Compute angle to target from predicted position
    // 3. Apply heading rate feedforward (double-EMA filtered)
    // 4. Apply translational feedforward (bearing rate change)
    // 5. Apply heading drift compensation
    // 6. Adaptive target smoothing (fast response on large errors, smooth on small)
    // =====================================================================
    private double computeOdometryTarget() {
        double speed = Math.hypot(velX, velY);
        double leadScale = Math.min(1.0, speed / 8.0);

        // Predict future position: pos + vel*t + 0.5*accel*t^2
        double predX = poseX
                + velX * VEL_LEAD_TIME * leadScale
                + 0.5 * accX * ACCEL_LEAD_TIME * ACCEL_LEAD_TIME * leadScale;
        double predY = poseY
                + velY * VEL_LEAD_TIME * leadScale
                + 0.5 * accY * ACCEL_LEAD_TIME * ACCEL_LEAD_TIME * leadScale;

        // Angle to target from predicted position
        double dx = TargetX - predX;
        double dy = TargetY - predY;
        double distToTarget = Math.hypot(dx, dy);
        double angleToTarget = Math.atan2(dy, dx);
        double turretRad = normalizeAngle(angleToTarget - poseH);
        double odomAngle = Math.toDegrees(turretRad) * SCALE_FACTOR;

        // Translational feedforward (bearing rate from tangential velocity)
        double tangVel = velX * Math.sin(angleToTarget) - velY * Math.cos(angleToTarget);
        double bearingRate = tangVel / Math.max(distToTarget, MIN_DIST);
        double transFF = Math.toDegrees(bearingRate) * SCALE_FACTOR * TRANS_FF_GAIN;

        // Heading rate feedforward (double-EMA filtered = smooth, lag-free)
        double ffScale = Math.min(1.0, speed / 8.0);
        double headingFF = Math.toDegrees(hrSlow) * H_FF_GAIN * ffScale;

        // Heading drift compensation
        double driftComp = -driftAccum * SCALE_FACTOR * ffScale;

        // Combine
        double target = odomAngle + TURELA_OFFSET_DEG + headingFF + transFF + driftComp;
        target = clamp(target, LIMITA_STANGA, LIMITA_DREAPTA);

        // Adaptive smoothing
        target = adaptiveSmooth(target, odomSmoothed, odomInit, speed);
        odomSmoothed = target;
        odomInit = true;

        return target;
    }

    // =====================================================================
    // MODE 2: LIMELIGHT-ONLY TRACKING
    //
    // Pipeline:
    // 1. EMA-filtered tx (noise rejection)
    // 2. Latency compensation (heading rate × pipeline latency)
    // 3. TX velocity prediction (anticipate where target will be)
    // 4. PID cu derivative-on-measurement (no derivative kick)
    // 5. Anti-windup integral (conditional on output not saturated)
    // 6. Dynamic gains (more aggressive when far from target)
    // 7. Graceful target loss: decay correction → smooth return to center
    // =====================================================================
    private double computeLimelightTarget(double dt) {
        double currentAngle = turelaD.getCurrentAngle();

        if (!llVisible) {
            // Target lost — gradually decay correction and drift toward last target
            llIntegral *= LOSS_DECAY;
            if (timeSinceLLSeen > 0.5) {
                // Slowly return toward center (odom fallback)
                llLastTarget += (0 - llLastTarget) * LOSS_CENTER_RATE;
            }
            return clamp(llLastTarget, LIMITA_STANGA, LIMITA_DREAPTA);
        }

        // Latency compensation: where was the turret when the frame was captured?
        double angleAtCapture = currentAngle - Math.toDegrees(hrFast) * LL_LATENCY * SCALE_FACTOR;

        // TX with velocity prediction: predict where the target will be
        double predictedTx = filteredTx + txVelocity * TX_VEL_LEAD;

        // Error in turret-degrees
        double error = -predictedTx * SCALE_FACTOR;

        // Dynamic P gain: more aggressive when far from target
        double absTx = Math.abs(filteredTx);
        double dynamicKP = LL_KP;
        if (absTx > 8.0) dynamicKP *= 1.5;
        else if (absTx > 4.0) dynamicKP *= 1.2;

        // Anti-windup integral (only integrate when output is not saturated)
        double rawOutput = dynamicKP * error;
        if (Math.abs(rawOutput) < LL_MAX_CORRECTION * 0.8) {
            llIntegral += error * dt;
            llIntegral = clamp(llIntegral, -LL_MAX_INTEGRAL, LL_MAX_INTEGRAL);
        }

        // Derivative on measurement (not error) — prevents derivative kick on setpoint change
        double derivative = -(currentAngle - llPrevMeas) / Math.max(dt, 0.005);
        llPrevMeas = currentAngle;

        // PID output
        double correction = dynamicKP * error + LL_KI * llIntegral + LL_KD * derivative;
        correction = clamp(correction, -LL_MAX_CORRECTION, LL_MAX_CORRECTION);

        // Heading rate feedforward (compensate robot rotation)
        double speed = Math.hypot(velX, velY);
        double ffScale = Math.min(1.0, speed / 8.0);
        correction += Math.toDegrees(hrSlow) * H_FF_GAIN * ffScale;

        double target = currentAngle + correction;
        target = clamp(target, LIMITA_STANGA, LIMITA_DREAPTA);

        llLastTarget = target;
        return target;
    }

    // =====================================================================
    // MODE 3: HYBRID — Odometry base + Limelight correction
    //
    // Pipeline:
    // 1. Compute odom angle (velocity+accel prediction, feedforward)
    // 2. If Limelight visible:
    //    a. Compute where LL says the target is (latency-compensated)
    //    b. Offset error = LL implied target - odom target
    //    c. EMA accumulate offset (gradually corrects odom drift)
    //    d. Conditional integral (slow drift fix, only when not saturated)
    // 3. If not visible: decay offset (gradually return to odom-only)
    // 4. All feedforward (heading + translational)
    // 5. Adaptive target smoothing
    // =====================================================================
    private double computeHybridTarget(double dt) {
        double speed = Math.hypot(velX, velY);
        double leadScale = Math.min(1.0, speed / 8.0);

        // Odom base angle (same as Mode 1 but without smoothing)
        double predX = poseX + velX * VEL_LEAD_TIME * leadScale
                + 0.5 * accX * ACCEL_LEAD_TIME * ACCEL_LEAD_TIME * leadScale;
        double predY = poseY + velY * VEL_LEAD_TIME * leadScale
                + 0.5 * accY * ACCEL_LEAD_TIME * ACCEL_LEAD_TIME * leadScale;
        double dx = TargetX - predX;
        double dy = TargetY - predY;
        double distToTarget = Math.hypot(dx, dy);
        double angleToTarget = Math.atan2(dy, dx);
        double turretRad = normalizeAngle(angleToTarget - poseH);
        double odomAngle = Math.toDegrees(turretRad) * SCALE_FACTOR;
        odomAngle = clamp(odomAngle, LIMITA_STANGA, LIMITA_DREAPTA);

        // Limelight offset correction
        if (llVisible) {
            double currentAngle = turelaD.getCurrentAngle();
            // Where was turret when frame was captured?
            double angleAtCapture = currentAngle - Math.toDegrees(hrFast) * LL_LATENCY * SCALE_FACTOR;
            // What angle does limelight imply the target is at?
            double llImpliedTarget = angleAtCapture - filteredTx * SCALE_FACTOR;
            // How far off is our odom estimate?
            double offsetError = llImpliedTarget - odomAngle;

            // EMA accumulate offset
            hybOffset += OFFSET_EMA * (offsetError - hybOffset);

            // Conditional integral (only when output not saturated)
            double rawOutput = POS_KP * (odomAngle + hybOffset - currentAngle);
            if (Math.abs(rawOutput) < POS_MAX_POWER) {
                hybIntegral += offsetError * dt;
                hybIntegral = clamp(hybIntegral, -OFFSET_MAX_INTEGRAL, OFFSET_MAX_INTEGRAL);
            }

            hybOffset = clamp(hybOffset, -MAX_OFFSET, MAX_OFFSET);
        } else {
            // No limelight — decay offset and integral toward zero
            hybOffset *= OFFSET_DECAY_RATE;
            hybIntegral *= 0.95;
        }

        // Translational feedforward
        double tangVel = velX * Math.sin(angleToTarget) - velY * Math.cos(angleToTarget);
        double bearingRate = tangVel / Math.max(distToTarget, MIN_DIST);
        double transFF = Math.toDegrees(bearingRate) * SCALE_FACTOR * TRANS_FF_GAIN;

        // Heading rate feedforward
        double ffScale = Math.min(1.0, speed / 8.0);
        double headingFF = Math.toDegrees(hrSlow) * H_FF_GAIN * ffScale;

        // Drift compensation
        double driftComp = -driftAccum * SCALE_FACTOR * ffScale;

        // Combine: odom + LL offset + integral + feedforward + drift comp
        double target = odomAngle + hybOffset + hybIntegral * OFFSET_INTEGRAL_KI
                + TURELA_OFFSET_DEG + headingFF + transFF + driftComp;
        target = clamp(target, LIMITA_STANGA, LIMITA_DREAPTA);

        // Adaptive smoothing
        target = adaptiveSmooth(target, hybSmoothed, hybInit, speed);
        hybSmoothed = target;
        hybInit = true;

        return target;
    }

    // =====================================================================
    // ADAPTIVE SMOOTHING — WPILib-inspired
    // Fast response on large errors/disturbances, smooth on small errors/stationary
    // =====================================================================
    private double adaptiveSmooth(double target, double prevSmoothed, boolean initialized, double speed) {
        if (!initialized) return target;

        double currentAngle = turelaD.getCurrentAngle();
        double preError = Math.abs(target - currentAngle);

        double alpha;
        if (preError > DISTURB_THRESH) alpha = SMOOTH_FAST;
        else if (preError > 5.0) alpha = SMOOTH_MEDIUM;
        else if (speed < 5.0) alpha = SMOOTH_SLOW;
        else alpha = SMOOTH_NORMAL;

        return prevSmoothed + alpha * (target - prevSmoothed);
    }

    // =====================================================================
    // POSITION CONTROLLER — PD cu derivative-on-measurement
    // Shared by all 3 modes. Drives turret servos to target angle.
    //
    // Features:
    // - Derivative on measurement (not error) — no kick on target change
    // - Deadzone with smooth ramp (no jitter near target)
    // - Dynamic max power (boost for large errors)
    // - Minimum power enforcement (overcomes static friction)
    // =====================================================================
    private void applyTurelaControl(double targetAngle) {
        turelaTarget = targetAngle;
        double currentAngle = turelaD.getCurrentAngle();
        double error = targetAngle - currentAngle;
        double absError = Math.abs(error);

        // Deadzone — stop when close enough
        if (absError < TURELA_DEADZONE) {
            turelaD.setPosition(0.5);
            turelaS.setPosition(0.5);
            return;
        }

        // Derivative on measurement (measurement is more stable than error)
        double dt = 0.005; // approximate, could track actual dt
        double derivative = -(currentAngle - posPrevMeas) / dt;
        posPrevMeas = currentAngle;

        // Dynamic max power based on error magnitude
        double maxPower;
        if (absError > 20.0) maxPower = POS_BOOST_POWER;
        else if (absError > 10.0) maxPower = POS_MAX_POWER + 0.05;
        else maxPower = POS_MAX_POWER;

        // PD output
        double power = POS_KP * error + POS_KD * derivative;

        // Smooth ramp near deadzone (prevents oscillation)
        if (absError < TURELA_DEADZONE * 2) {
            double ramp = (absError - TURELA_DEADZONE) / TURELA_DEADZONE;
            power *= Math.max(0, ramp);
        }

        // Minimum power enforcement (overcomes static friction)
        if (Math.abs(power) < POS_MIN_POWER && absError > TURELA_DEADZONE) {
            power = Math.signum(power) * POS_MIN_POWER;
        }

        power = clamp(power, -maxPower, maxPower);
        turelaD.setPosition(0.5 - power);
        turelaS.setPosition(0.5 - power);
    }

    private void centreazaTurela() {
        applyTurelaControl(0);
    }

    // =====================================================================
    // RESET
    // =====================================================================
    private void resetAllModes() {
        // Mode 1
        odomSmoothed = 0; odomInit = false;
        driftAccum = 0;
        driftWindowStart = System.nanoTime() / 1_000_000_000.0;
        driftHeadingAtStart = poseH;
        // Mode 2
        llIntegral = 0; llPrevMeas = turelaD.getCurrentAngle();
        llLastTarget = turelaD.getCurrentAngle(); llEverSeen = false;
        filteredTx = 0; prevFilteredTx = 0; txVelocity = 0;
        txMedianBuf[0] = 0; txMedianBuf[1] = 0; txMedianBuf[2] = 0; txMedianIdx = 0;
        // Mode 3
        hybOffset = 0; hybIntegral = 0;
        hybSmoothed = 0; hybInit = false;
        // Shared
        posPrevMeas = turelaD.getCurrentAngle();
        turelaTarget = 0;
        // Sensor
        hrFast = 0; hrSlow = 0;
        velX = 0; velY = 0; accX = 0; accY = 0;
        prevVelX = 0; prevVelY = 0;
        timeSinceLLSeen = 999;
    }

    // =====================================================================
    // THREADS
    // =====================================================================
    private final Thread TrackingThread = new Thread(() -> {
        while (!stop) {
            try { Thread.sleep(5); } catch (InterruptedException e) { break; }

            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            follower.update();
            updateSensors();

            double dt = 0.005;

            if (trackingMode > 0) {
                double targetAngle;
                switch (trackingMode) {
                    case 1: targetAngle = computeOdometryTarget(); break;
                    case 2: targetAngle = computeLimelightTarget(dt); break;
                    case 3: targetAngle = computeHybridTarget(dt); break;
                    default: targetAngle = 0; break;
                }
                applyTurelaControl(targetAngle);
            } else if (!trouch) {
                centreazaTurela();
            } else {
                double manualPutere = 0;
                if (gamepad1.left_bumper) manualPutere = -POS_MAX_POWER * 2;
                else if (gamepad1.right_bumper) manualPutere = POS_MAX_POWER * 2;
                if (manualPutere != 0) {
                    turelaD.setPosition(0.5 - manualPutere);
                    turelaS.setPosition(0.5 - manualPutere);
                } else {
                    turelaD.setPosition(0.5);
                    turelaS.setPosition(0.5);
                }
            }
        }
        turelaD.setPosition(0.5);
        turelaS.setPosition(0.5);
    }, "Tracking");

    private final Thread ButtonThread = new Thread(() -> {
        while (!stop) {
            try { Thread.sleep(10); } catch (InterruptedException e) { break; }

            boolean bumper = gamepad2.right_bumper;
            if (modeToggle != bumper) {
                if (bumper) {
                    trackingMode = (trackingMode + 1) % 4;
                    resetAllModes();
                    gamepad2.rumble(150);
                }
                modeToggle = bumper;
            }

            boolean gp1Touch = gamepad1.touchpad;
            if (Touch != gp1Touch) {
                if (gp1Touch) trouch = !trouch;
                Touch = gp1Touch;
            }

            if (gamepad2.dpad_down && pinpoint != null) {
                pinpoint.recalibrateIMU();
                gamepad2.rumble(200);
            }
            if (gamepad2.dpad_right && follower != null) {
                Pose cur = follower.getPose();
                follower.setPose(new Pose(cur.getX(), cur.getY(), 0));
                gamepad2.rumble(200);
            }
        }
    }, "Buttons");

    private final Thread ChassisThread = new Thread(() -> {
        while (!stop) {
            try { Thread.sleep(5); } catch (InterruptedException e) { break; }
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            double fl = y + x + rx, bl = y - x + rx;
            double br = y + x - rx, fr = y - x - rx;
            double max = Math.max(1, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                    Math.max(Math.abs(bl), Math.abs(br))));
            fl /= max; fr /= max; bl /= max; br /= max;
            if (gamepad1.right_trigger > 0) sm = 2;
            else if (gamepad1.left_trigger > 0) sm = 5;
            else sm = 1;
            frontLeft.setPower(fl / sm); frontRight.setPower(fr / sm);
            backLeft.setPower(bl / sm); backRight.setPower(br / sm);
        }
    }, "Chassis");

    // =====================================================================
    // OPMODE LIFECYCLE
    // =====================================================================
    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

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
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.recalibrateIMU();
        ElapsedTime ct = new ElapsedTime();
        while (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                && ct.milliseconds() < 1000) {}

        Pose startPose = new Pose(RobotPozitie.X, RobotPozitie.Y, RobotPozitie.heading);
        follower.setStartingPose(startPose);
        follower.update();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        turelaD = ServoImplExEx.getContinuous(hardwareMap, "turelaD", "turretD");
        turelaD.setEncoderReferenceVoltage(REFERINTA_VOLTAJ);
        turelaD.setAngleLimits(LIMITA_STANGA, LIMITA_DREAPTA);
        turelaD.setDeadzone(TURELA_DEADZONE);
        turelaD.setPosition(0.5);

        turelaS = ServoImplExEx.getContinuous(hardwareMap, "turelaS", "turretS");
        turelaS.setPosition(0.5);
    }

    @Override
    public void start() {
        stop = false;
        follower.update();
        driftWindowStart = System.nanoTime() / 1_000_000_000.0;
        driftHeadingAtStart = follower.getPose().getHeading();
        ChassisThread.start();
        ButtonThread.start();
        TrackingThread.start();
    }

    @Override
    public void loop() {
        double curAngle = turelaD.getCurrentAngle();
        double error = turelaTarget - curAngle;

        telemetry.addData("MODE", getModeName());
        telemetry.addData("[GP2 RB]", "OFF > ODOM > LIMELIGHT > HYBRID");
        telemetry.addLine("");

        telemetry.addData("Target", "%.1f deg", turelaTarget);
        telemetry.addData("Current", "%.1f deg", curAngle);
        telemetry.addData("Error", "%.1f deg", error);
        telemetry.addLine("");

        if (trackingMode == 1) {
            // Odom-specific telemetry
            telemetry.addData("HR (fast)", "%.1f deg/s", Math.toDegrees(hrFast));
            telemetry.addData("HR (slow)", "%.1f deg/s", Math.toDegrees(hrSlow));
            telemetry.addData("Speed", "%.1f in/s", Math.hypot(velX, velY));
            telemetry.addData("Accel", "%.1f in/s2", Math.hypot(accX, accY));
            telemetry.addData("Drift comp", "%.2f deg", driftAccum);
        } else if (trackingMode == 2) {
            // Limelight-specific telemetry
            telemetry.addData("LL", llVisible ? "VEDE" : String.format("NU VEDE (%.1fs)", timeSinceLLSeen));
            telemetry.addData("Raw tx", "%.2f", rawTx);
            telemetry.addData("Filtered tx", "%.2f", filteredTx);
            telemetry.addData("TX vel", "%.2f/s", txVelocity);
            telemetry.addData("PID I", "%.2f", llIntegral);
        } else if (trackingMode == 3) {
            // Hybrid-specific telemetry
            telemetry.addData("LL", llVisible ? "VEDE" : "NU VEDE");
            telemetry.addData("Offset", "%.2f deg", hybOffset);
            telemetry.addData("Integral", "%.2f", hybIntegral);
            telemetry.addData("HR (slow)", "%.1f deg/s", Math.toDegrees(hrSlow));
            telemetry.addData("Speed", "%.1f in/s", Math.hypot(velX, velY));
        }

        telemetry.addLine("");
        telemetry.addData("Pos", "%.1f, %.1f", poseX, poseY);
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(poseH));
        telemetry.addData("Manual", trouch ? "ON" : "OFF");
        telemetry.update();
    }

    @Override
    public void stop() {
        stop = true;
        turelaD.setPosition(0.5);
        turelaS.setPosition(0.5);
        if (limelight != null) limelight.stop();
    }

    private String getModeName() {
        switch (trackingMode) {
            case 1: return "ODOM (odometry-only)";
            case 2: return llVisible ? "LIMELIGHT (tracking)" : "LIMELIGHT (lost)";
            case 3: return llVisible ? "HYBRID (fused)" : "HYBRID (odom fallback)";
            default: return "OFF";
        }
    }
}
