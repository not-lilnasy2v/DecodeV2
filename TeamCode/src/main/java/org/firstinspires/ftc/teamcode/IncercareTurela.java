package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.NouHard.RTPAxon;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "Incercare Turela ")
@Configurable
public class IncercareTurela extends LinearOpMode {

    public static double LIMITA_STANGA_GRADE = -219.9;
    public static double LIMITA_DREAPTA_GRADE = 218.3;
    public static double REFERINTA_VOLTAJ_D = 0.3730;
    public static double SCALE_FACTOR = 2.435;

    public static double KP = 0.0027;
    public static double KI = 0.00010;
    public static double KD = 0.00008;
    public static double MAX_INTEGRAL = 80.0;
    public static double SERVO_MAX_POWER = 0.50;

    public static double MANUAL_NUDGE_DEG = 2.0;

    public static double DRIVE_NORMAL = 0.7;
    public static double DRIVE_PRECISION = 2.7;
    public static double STRAFE_MULT = 1.1;

    public static double TARGET_X = 144.0;
    public static double TARGET_Y = 144.0;
    public static double LL_ALPHA = 0.25;
    public static double LL_DECAY = 0.97;
    public static double MAX_LL_OFFSET = 35.0;
    public static double VEL_LEAD_TIME = 0.20;
    public static double VEL_FILTER = 0.22;

    private Limelight3A limelight;
    private RTPAxon turela;
    private TelemetryManager telemetryM;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private Follower follower;
    private final sistemeTeleOp m = new sistemeTeleOp();

    private volatile boolean stopFlag = false;
    private volatile boolean tracking = false;
    private volatile boolean resetRequested = false;
    private volatile boolean recenterRequested = false;

    private volatile double  lastTx = 0;
    private volatile boolean ttVede = false;
    private volatile double  llOffset = 0;
    private volatile double  odomAngleDeg = 0;
    private volatile double  poseX = 0, poseY = 0, poseHeading = 0;

    private double xVelocity = 0, yVelocity = 0;
    private double lastPoseX = Double.NaN, lastPoseY = Double.NaN;
    private long   lastTrackTime = 0;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();

        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        turela = RTPAxon.getDual(hardwareMap, "turelaD", "turelaS", "turretD");
        turela.setEncoderReferenceVoltage(REFERINTA_VOLTAJ_D);
        turela.setAngleLimits(LIMITA_STANGA_GRADE, LIMITA_DREAPTA_GRADE);
        turela.setMaxPower(SERVO_MAX_POWER);
        turela.setMaxIntegralSum(MAX_INTEGRAL);
        turela.setPidCoeffs(KP, KI, KD);
        turela.setTargetRotation(turela.getTotalRotation());

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(RobotPozitie.X, RobotPozitie.Y, RobotPozitie.heading));
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();
        if (isStopRequested()) return;

        Thread chassisThread = new Thread(this::chassis, "Chassis");
        Thread turretThread  = new Thread(this::turret,  "Turret");
        chassisThread.setDaemon(true);
        turretThread.setDaemon(true);
        chassisThread.start();
        turretThread.start();

        boolean prevX = false;

        while (opModeIsActive()) {
            boolean x = gamepad2.x;
            if (x && !prevX) {
                tracking = !tracking;
                resetRequested = true;
            }
            prevX = x;

            if (gamepad2.b) recenterRequested = true;

            telemetryM.debug("Tracking: " + (tracking ? "ON" : "OFF"));
            telemetryM.debug("Vede tag: " + ttVede + (ttVede ? " (LL+ODOM)" : " (ODOM)"));
            telemetryM.debug(String.format("tx: %.2f", lastTx));
            telemetryM.debug(String.format("Pose: X=%.1f Y=%.1f H=%.1f deg",
                    poseX, poseY, Math.toDegrees(poseHeading)));
            telemetryM.debug(String.format("Odom angle: %.2f deg",      odomAngleDeg));
            telemetryM.debug(String.format("LL offset:  %.2f deg",      llOffset));
            telemetryM.debug(String.format("Target:     %.2f deg",      turela.getTargetRotation()));
            telemetryM.debug(String.format("Total:      %.2f deg",      turela.getTotalRotation()));
            telemetryM.debug(String.format("Curent:     %.2f deg",      turela.getCurrentAngle()));
            telemetryM.debug(String.format("Eroare:     %.2f deg",
                    turela.getTargetRotation() - turela.getTotalRotation()));
            telemetryM.debug(String.format("Power:      %.3f", turela.getPower()));
            telemetryM.update(telemetry);

            m.Nkdf(20_000_000);
        }

        stopFlag = true;
        try { chassisThread.join(500); } catch (InterruptedException ignored) {}
        try { turretThread.join(500);  } catch (InterruptedException ignored) {}

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        turela.stop();
        limelight.stop();
    }

    private void chassis() {
        while (!stopFlag && opModeIsActive()) {
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x * STRAFE_MULT;
            double rx =  gamepad1.right_stick_x;

            double fl = y + x + rx;
            double bl = y - x + rx;
            double fr = y - x - rx;
            double br = y + x - rx;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                                  Math.max(Math.abs(bl), Math.abs(br)));
            if (max > 1) { fl /= max; bl /= max; fr /= max; br /= max; }

            double sm = gamepad1.left_trigger > 0 ? DRIVE_PRECISION : DRIVE_NORMAL;

            frontLeft.setPower(fl  / sm);
            backLeft.setPower(bl   / sm);
            frontRight.setPower(fr / sm);
            backRight.setPower(br  / sm);

            m.Nkdf(5_000_000);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void turret() {
        while (!stopFlag && opModeIsActive()) {
            turela.setKP(KP);
            turela.setKD(KD);
            turela.setMaxPower(SERVO_MAX_POWER);
            turela.setMaxIntegralSum(MAX_INTEGRAL);

            follower.update();
            Pose pose = follower.getPose();
            poseX = pose.getX();
            poseY = pose.getY();
            poseHeading = pose.getHeading();

            if (resetRequested) {
                turela.resetPID();
                resetRequested = false;
                lastTrackTime = 0;
                lastPoseX = Double.NaN;
                lastPoseY = Double.NaN;
                xVelocity = 0;
                yVelocity = 0;
            }
            if (recenterRequested) {
                turela.setTargetRotation(0);
                recenterRequested = false;
            }

            if (tracking) {
                long now = System.nanoTime();
                double dt = lastTrackTime == 0 ? 0.005 : (now - lastTrackTime) / 1e9;
                if (dt < 0.001) dt = 0.001;
                if (dt > 0.1)   dt = 0.1;
                lastTrackTime = now;

                if (Double.isNaN(lastPoseX)) {
                    lastPoseX = poseX;
                    lastPoseY = poseY;
                }

                xVelocity += VEL_FILTER * ((poseX - lastPoseX) / dt - xVelocity);
                yVelocity += VEL_FILTER * ((poseY - lastPoseY) / dt - yVelocity);
                lastPoseX = poseX;
                lastPoseY = poseY;

                double predX = poseX + xVelocity * VEL_LEAD_TIME;
                double predY = poseY + yVelocity * VEL_LEAD_TIME;
                double dx = TARGET_X - predX;
                double dy = TARGET_Y - predY;
                double angleToTarget = Math.atan2(dy, dx);
                double turretRad = normalizeAngle(angleToTarget - poseHeading);
                odomAngleDeg = Math.toDegrees(turretRad) * SCALE_FACTOR;

                LLResult r = limelight.getLatestResult();
                boolean vede = false;
                if (r != null && r.isValid()) {
                    List<LLResultTypes.FiducialResult> fids = r.getFiducialResults();
                    if (fids != null && !fids.isEmpty()) {
                        double tx = r.getTx();
                        lastTx = tx;
                        vede = true;
                        double currentAngle = turela.getCurrentAngle();
                        double trueTarget   = currentAngle - tx * SCALE_FACTOR;
                        double offsetError  = trueTarget - odomAngleDeg;
                        llOffset += LL_ALPHA * (offsetError - llOffset);
                        if (llOffset >  MAX_LL_OFFSET) llOffset =  MAX_LL_OFFSET;
                        if (llOffset < -MAX_LL_OFFSET) llOffset = -MAX_LL_OFFSET;
                    }
                }
                if (!vede) llOffset *= LL_DECAY;
                ttVede = vede;

                double targetAngle = odomAngleDeg + llOffset;
                if (targetAngle < LIMITA_STANGA_GRADE)  targetAngle = LIMITA_STANGA_GRADE;
                if (targetAngle > LIMITA_DREAPTA_GRADE) targetAngle = LIMITA_DREAPTA_GRADE;
                turela.updateTargetRotation(targetAngle);
            } else {
                ttVede = false;
                if (gamepad1.dpad_left)  turela.changeTargetRotation(-MANUAL_NUDGE_DEG);
                if (gamepad1.dpad_right) turela.changeTargetRotation(+MANUAL_NUDGE_DEG);
                lastTrackTime = 0;
                lastPoseX = Double.NaN;
                lastPoseY = Double.NaN;
            }

            turela.update();

            m.Nkdf(5_000_000);
        }
        turela.stop();
    }

    private static double normalizeAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
