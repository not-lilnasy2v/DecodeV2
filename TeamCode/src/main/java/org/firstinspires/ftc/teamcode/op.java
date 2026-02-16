package org.firstinspires.ftc.teamcode;
import static java.lang.Math.abs;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
@Configurable
public class op extends OpMode {
    private DcMotorEx frontRight, frontLeft, backRight, backLeft,scula;
    private ServoImplEx bascula;
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;

    public Follower follower;
    volatile boolean stop;
    double sm = 1;
    double max = 0;
    double FL, BL, BR, FR;
    private volatile boolean imuRecalibrating = false;
    sistemeTeleOp m = new sistemeTeleOp();
    private static double TargetX = 147;
    private static double TargetY = 144;
    public static double DEADBAND = 0.001;
    private double ultimaPozitie = 0.5;
    private ElapsedTime pidTimer = new ElapsedTime();
    private static double voltajeNominale = 12.68;
    public volatile boolean turelaTracking = false, tracking = false, Ipornit = false, IntakePornit = false, SortingPornit = false, SortingToggle = false,Touch = false,trouch=false;
    private double ledistante, posU;
    int idTag = RobotPozitie.idTag;
    private final ElapsedTime sortTimer = new ElapsedTime();
    private boolean scanActive = false;
    private boolean lbPrev = false;
    private final ElapsedTime scanTimer = new ElapsedTime();
    private int scanStep = 0;
    private double scanLastPos = Pozitii.luarea1;
    private boolean scanWaitingMove = false;
    private static final int SCAN_MOVE_MS = 220;
    private int scanWaitMs = SCAN_MOVE_MS;
    private boolean sortWait = false;
    private enum ShootMode { NONE, RAPID, PATTERN }
    private enum ShootState { IDLE, MOVE_WAIT, FIRE_ON, FIRE_OFF }
    private ShootMode shootMode = ShootMode.NONE;
    private ShootState shootState = ShootState.IDLE;
    private final ElapsedTime shootTimer = new ElapsedTime();
    private double lastSortPos = Pozitii.luarea1;
    private int currentStep = 0, currentSlot = -1, moveWaitMs = 0;
    private boolean yPrev = false;
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

    private volatile boolean sugere = false;
    private volatile boolean trageShooting = false;
    private final Object blocat = new Object();
    private volatile double turelaPos = 0.5;

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
        bascula = hardwareMap.get(ServoImplEx.class, "bascula");

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
        scula.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        follower = Constants.createFollower(hardwareMap);

        Pose startingPose = new Pose(RobotPozitie.X, RobotPozitie.Y, RobotPozitie.heading);
        follower.setStartingPose(startingPose);
        follower.update();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.recalibrateIMU();
        ElapsedTime calibrationTimer = new ElapsedTime();
        while (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                && calibrationTimer.milliseconds() < 1000) {
            try { Thread.sleep(10); } catch (InterruptedException ignored) {}
        }

        /*while (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                && calibrationTimer.milliseconds() < 1000) {
        }*/

        applyVoltageCompensatedPIDF();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();

        m.turelaS.setPosition(0.5);
        m.turelaD.setPosition(0.5);
        turelaPos = 0.5;
    }
    private void updateButoane(){
        posU = m.unghiS.getPosition();

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

        boolean lbNow = gamepad2.left_bumper;
        boolean lbPressed = lbNow && !lbPrev;
        lbPrev = lbNow;

        if (lbPressed && !scanActive && !sugere && !trageShooting && !sortWait) {
            scanActive = true;
            scanStep = 0;
            scanLastPos = m.sortare.getPosition();
            scanWaitingMove = false;
            scanTimer.reset();
        }

        if (scanActive) {
            if (scanStep >= 3) {
                m.sortare.setPosition(Pozitii.luarea1);
                scanActive = false;
                gamepad2.rumble(300);
                return;
            }
            if (!slotOcupat[scanStep]) {
                scanStep++;
                scanWaitingMove = false;
                return;
            }
            double targetPos =
                    (scanStep == 0) ? Pozitii.luarea1 :
                            (scanStep == 1) ? Pozitii.luarea2 :
                                    Pozitii.luarea3;

            //muta servo si pornește așteptarea
            if (!scanWaitingMove) {
                m.sortare.setPosition(targetPos);
                double dist = Math.abs(targetPos - scanLastPos);
                scanWaitMs = (int)(dist * 550) + 150;

                scanTimer.reset();
                scanWaitingMove = true;
                scanLastPos = targetPos;
                return;
            }

            // dupa wait citeste culoarea
            if (scanTimer.milliseconds() < scanWaitMs) return;

            m.resetareDetection();
            int c = m.detecteazaBiloaca();
            slotColor[scanStep] = c;

            if (c != -1) gamepad2.rumble(150);

            //urmatorul slot
            scanStep++;
            scanWaitingMove = false;
            return;
        }
    }
    private volatile double currentH, currentX,currentY;

    private void updateTurela() {
        if (tracking) {
            double dx = TargetX - currentX;
            double dy = TargetY - currentY;

            double unghiLaTarget = Math.atan2(dy, dx);
            double turretAngleRad = unghiLaTarget - currentH;
            turretAngleRad = normalizeAngle(turretAngleRad);
            double turretAngleDeg = -Math.toDegrees(turretAngleRad);

            double odomPos = m.turelaS.angleToPosition(turretAngleDeg);
            odomPos = Math.max(0.0, Math.min(1.0, odomPos));

            if (Math.abs(odomPos - ultimaPozitie) > DEADBAND) {
                ultimaPozitie = odomPos;
                m.turelaS.setPosition(odomPos);
                m.turelaD.setPosition(odomPos);
            }

        } else if (!trouch) {
            m.turelaS.setPosition(0.5);
            m.turelaD.setPosition(0.5);
        } else {
            if (gamepad1.left_bumper) {
                double pos = m.turelaS.getPosition() - 0.035;
                m.turelaS.setPosition(pos);
                m.turelaD.setPosition(pos);
            } else if (gamepad1.right_bumper) {
                double pos = m.turelaS.getPosition() + 0.035;
                m.turelaS.setPosition(pos);
                m.turelaD.setPosition(pos);
            }
        }
    }
    private void updateSortare(){
        if (scanActive) return;

        if (sortWait) {
            if (sortTimer.milliseconds() < 350) return;
            sortWait = false;
        }
        int loculete = getLoculete();

        ledistante = m.distanta.getDistance(DistanceUnit.CM);

        if (Ipornit && !trageShooting && loculete < 3 && !gamepad1.b) {
            sugere = true;
            m.intake.setPower(1);


            if (m.bilaPrezenta(ledistante)) {
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
                    sortWait = true;
                    sortTimer.reset();
                    return;

                } else if (Math.abs(servoPos - Pozitii.luarea2) < 0.1 && !slotOcupat[1]) {
                    slotOcupat[1] = true;
                    slotColor[1] = detectedColor;
                    if (!slotOcupat[2]) {
                        m.sortare.setPosition(Pozitii.luarea3);
                    } else if (!slotOcupat[0]) {
                        m.sortare.setPosition(Pozitii.luarea1);
                    }
                    sortWait = true;
                    sortTimer.reset();
                    return;

                } else if (Math.abs(servoPos - Pozitii.luarea3) < 0.1 && !slotOcupat[2]) {
                    slotOcupat[2] = true;
                    slotColor[2] = detectedColor;
                    sortWait = true;
                    sortTimer.reset();
                    return;

                }

                if (getLoculete() == 3) {
                    Ipornit = false;
                    m.intake.setPower(0);
                    gamepad1.rumble(500);

                    if (SortingPornit) {
                        int primaCuloare = primaBilaPattern();
                        int slotShoot = BilaCuCuloare(primaCuloare);
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
        } else if(gamepad1.b){
            sugere = false;
            m.intake.setPower(-1);
        }
        else{
            sugere = false;
            m.intake.setPower(0);
        }
    }
    private int[] cPattern = new int[3];
    private double targetShooterVelocity = 1650;

    private void updateShooter() {
        m.shooter.setVelocity(targetShooterVelocity);
        m.shooter2.setVelocity(targetShooterVelocity);

        int loculete = getLoculete();
        boolean yNow = gamepad1.y;
        boolean yPressed = yNow && !yPrev;
        yPrev = yNow;

        if (shootState == ShootState.IDLE) {
            if (yPressed && loculete > 0 && !sugere && !trageShooting) {
                trageShooting = true;
                applyVoltageCompensatedPIDF();

                if (SortingPornit) {
                    fillPatternFromIdTag();
                    shootMode = ShootMode.PATTERN;
                    SortingPornit = false;
                } else {
                    shootMode = ShootMode.RAPID;
                }
                scula.setPower(-1);

                currentStep = 0;

                if (!beginNextShot()) {
                    finishShooting();
                }
            }
            return;
        }

        switch (shootState) {
            case MOVE_WAIT:
                if (shootTimer.milliseconds() >= moveWaitMs) {
                    m.Saruncare.setPosition(Pozitii.lansare);
                    shootTimer.reset();
                    shootState = ShootState.FIRE_ON;
                }
                break;

            case FIRE_ON:
                if (shootTimer.milliseconds() >= 130) {
                    m.Saruncare.setPosition(Pozitii.coborare);
                    shootTimer.reset();
                    shootState = ShootState.FIRE_OFF;
                }
                break;

            case FIRE_OFF:
                if (shootTimer.milliseconds() >= 90) {
                    //slotul gol
                    if (currentSlot >= 0 && currentSlot < 3) {
                        slotOcupat[currentSlot] = false;
                        slotColor[currentSlot] = -1;
                    }

                    // urm pas
                    currentStep++;
                    if (!beginNextShot()) {
                        finishShooting();
                    }
                }
                break;

            default:
                // safety
                finishShooting();
                break;
        }
    }
    private void fillPatternFromIdTag() {
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

    private boolean beginNextShot() {
        if (getLoculete() <= 0) return false;
        currentSlot = pickSlotForCurrentStep();
        if (currentSlot == -1) return false;
        double target = getAruncarePos(currentSlot);
        m.sortare.setPosition(target);
        double dist = Math.abs(target - lastSortPos);
        moveWaitMs = (int) (dist * 400) + 100;
        lastSortPos = target;

        shootTimer.reset();
        shootState = ShootState.MOVE_WAIT;
        return true;
    }

    private int pickSlotForCurrentStep() {
        if (shootMode == ShootMode.PATTERN) {
            if (currentStep >= 3) return -1;
            int need = cPattern[currentStep];
            return findSlotByNeedColor(need);
        }
        int[] order = {0, 2, 1};
        if (currentStep >= order.length) return -1;
        int s = order[currentStep];
        if (s >= 0 && s < 3 && slotOcupat[s]) return s;
        return BilaCuCuloare(-1);
    }

    private int findSlotByNeedColor(int needColor) {
        if (needColor == -1) return BilaCuCuloare(-1);

        for (int i = 0; i < 3; i++) {
            if (slotOcupat[i] && slotColor[i] == needColor) return i;
        }
        return BilaCuCuloare(-1);
    }

    private void finishShooting() {
        scula.setPower(0);
        m.sortare.setPosition(Pozitii.luarea1);
        trageShooting = false;
        Ipornit = true;
        shootState = ShootState.IDLE;
        shootMode = ShootMode.NONE;
    }

    private void updateChassis(){
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

    @Override
    public void stop() {
        stop = true;

        scula.setPower(0);
        m.intake.setPower(0);
        m.shooter.setPower(0);
        m.shooter2.setPower(0);

        if (limelight != null) limelight.stop();
    }

    @Override
    public void loop() {
        ///update odometrie o singura data pe ciclu
        follower.update();
        Pose p = follower.getPose();
        currentX = p.getX();
        currentY = p.getY();
        currentH = p.getHeading();

        /// logica din threaduri
        updateButoane();
        updateChassis();
        updateTurela();
        updateSortare();
        updateShooter();

        int detected = m.detecteazaBiloaca();
        bascula.setPosition(0.62);
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

        telemetry.addData("id", idTag);
        telemetry.addData("shooter", m.shooter.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("shooter2", m.shooter2.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Total biloace", getLoculete());
        telemetry.addData("turela", m.turelaD.getPosition());
        telemetry.addData("unghi", posU);
        telemetry.addData("distanta", ledistante);
        telemetry.addData("targetVelocity", targetShooterVelocity);
        telemetry.addLine("");
        telemetry.addData("Tracking", tracking ? "ON (Odometry)" : "OFF");
        telemetry.addLine("");
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(currentH));
        telemetry.addData("Pinpoint Status", pinpoint != null ? pinpoint.getDeviceStatus().toString() : "N/A");
        telemetry.addLine("dpad_down=Recalibrate IMU | dpad_right=Reset Heading");
        telemetry.addData("y", currentY);
        double v1 = m.shooter.getVelocity();
        double v2 = m.shooter2.getVelocity();
        double vAvg = (v1 + v2) / 2.0;
        telemetry.addData("mode1", m.shooter.getMode());
        telemetry.addData("mode2", m.shooter2.getMode());

        telemetry.addData("Fly v1", v1);
        telemetry.addData("Fly v2", v2);
        telemetry.addData("Fly avg", vAvg);
        telemetry.addData("shooter", m.shooter.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Hood pos", m.unghiS.getPosition());
        telemetry.update();
    }
    public void POWER(double fr1, double bl1, double br1, double fl1) {
        frontRight.setPower(fr1);
        backLeft.setPower(bl1);
        backRight.setPower(br1);
        frontLeft.setPower(fl1);
    }
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}