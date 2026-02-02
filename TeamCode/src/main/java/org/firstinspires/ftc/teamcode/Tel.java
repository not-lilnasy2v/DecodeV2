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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;


@TeleOp(name = "Main")
@Configurable
public class Tel extends OpMode {

    // 50 mov // 60 - > verde
    private DcMotorEx frontRight, frontLeft, backRight, backLeft;
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
    private double currentY, currentX;
    public static double DEADBAND = 0.001;
    private double ultimaPozitie = 0.5;
    private ElapsedTime pidTimer = new ElapsedTime();

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
    private volatile double turelaPos = 0.5;
    private volatile boolean lastTrouch = false;
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

        m.turelaS.setPosition(0.5);
        m.turelaD.setPosition(0.5);
        turelaPos = 0.5;
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
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    break;
                }
                posU = m.unghiS.getPosition();

                // Turela toggle
                boolean dpad_right1 = gamepad2.right_bumper;
                if (turelaTracking != dpad_right1) {
                    if (gamepad2.right_bumper) {
                        tracking = !tracking;
                    }
                    turelaTracking = dpad_right1;
                }
                boolean gamepad1_touch = gamepad1.touchpad;
                if (Touch != gamepad1_touch) {
                    if (gamepad1.touchpad) {
                        trouch = !trouch;
                    }
                    Touch = gamepad1_touch;
                }

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
                            int moveWait = (int) (dist * 550) + 150;
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

    private final Thread Turela = new Thread(new Runnable() {
        @Override
        public void run() {
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
                double currentH = currentPose.getHeading();

                if (tracking) {
//                    if (currentY > 70){
//                        TargetX = 1.25;
//                    }
//                    else TargetX = 4.25;
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
        }
    });

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
    private double targetShooterVelocity = 1650;

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

                    m.shooter.setVelocity(targetShooterVelocity);
                    m.shooter2.setVelocity(targetShooterVelocity);

                    if (gamepad1.y && loculete > 0 && !sugere && !trageShooting) {
                        trageShooting = true;
                        applyVoltageCompensatedPIDF();

                        if (SortingPornit) {
                            Pattern();
                            shootPattern();
                            SortingPornit = false;
                        } else {
                            rapidFireShoot();
                        }

                        m.sortare.setPosition(Pozitii.luarea1);
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

        private void shootPattern() {
            double lastPos = m.sortare.getPosition();

            for (int step = 0; step < 3; step++) {
                int need = cPattern[step];
                int slotShoot = GasestePattern(need);

                if (slotShoot == -1) break;

                double target = getTarget(slotShoot);
                m.sortare.setPosition(target);

                double dist = Math.abs(target - lastPos);
                int moveWait = (int) (dist * 400) + 100;
                m.kdf(moveWait);

                m.Saruncare.setPosition(Pozitii.lansare);
                m.kdf(130);
                m.Saruncare.setPosition(Pozitii.coborare);
                m.kdf(90);

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

            if (slotOcupat[0]) {//1 2 3
                double target = getTarget(0);
                m.sortare.setPosition(target);

                double dist = Math.abs(target - lastPos);
                int moveWait = (int) (dist * 400) + 100;
                m.kdf(moveWait);

                m.Saruncare.setPosition(Pozitii.lansare);
                m.kdf(130);
                m.Saruncare.setPosition(Pozitii.coborare);
                m.kdf(90);

                slotOcupat[0] = false;
                slotColor[0] = -1;
                lastPos = target;
            }
            if (slotOcupat[2]) {//1 2 3
                double target = getTarget(2);
                m.sortare.setPosition(target);

                double dist = Math.abs(target - lastPos);
                int moveWait = (int) (dist * 400) + 100;
                m.kdf(moveWait);

                m.Saruncare.setPosition(Pozitii.lansare);
                m.kdf(130);
                m.Saruncare.setPosition(Pozitii.coborare);
                m.kdf(90);

                slotOcupat[2] = false;
                slotColor[2] = -1;
                lastPos = target;
            }

            if (slotOcupat[1]) {//1 2 3
                double target = getTarget(1);
                m.sortare.setPosition(target);

                double dist = Math.abs(target - lastPos);
                int moveWait = (int) (dist * 400) + 100;
                m.kdf(moveWait);

                m.Saruncare.setPosition(Pozitii.lansare);
                m.kdf(130);
                m.Saruncare.setPosition(Pozitii.coborare);
                m.kdf(90);

                slotOcupat[1] = false;
                slotColor[1] = -1;
                lastPos = target;
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

                max = abs(FL);
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
        telemetry.addData("id", idTag);
        telemetry.addData("shooter", m.shooter.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("shooter2", m.shooter2.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Total biloace", getLoculete());
        telemetry.addData("turela", m.turelaD.getPosition());
        telemetry.addData("unghi", posU);
        telemetry.addData("distanta", distantare);
        telemetry.addData("targetVelocity", targetShooterVelocity);
        telemetry.addLine("");
        telemetry.addData("Tracking", tracking ? "ON (Odometry)" : "OFF");
        telemetry.addLine("");
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Pinpoint Status", pinpoint != null ? pinpoint.getDeviceStatus().toString() : "N/A");
        telemetry.addLine("dpad_down=Recalibrate IMU | dpad_left=Reset Heading");
        telemetry.addData("y", currentY);
        telemetry.update();
    }

    public void POWER(double df1, double sf1, double ds1, double ss1) {
        frontRight.setPower(df1);
        backLeft.setPower(ss1);
        frontLeft.setPower(sf1);
        backRight.setPower(ds1);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
