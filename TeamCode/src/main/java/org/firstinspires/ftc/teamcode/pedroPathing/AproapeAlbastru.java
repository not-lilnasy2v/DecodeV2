package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Pozitii;
import org.firstinspires.ftc.teamcode.RobotPozitie;
import org.firstinspires.ftc.teamcode.pop;
import org.firstinspires.ftc.teamcode.sistemeAuto;

@Autonomous(name = "Departe Albastru")
public class AproapeAlbastru extends OpMode {
    sistemeAuto n = new sistemeAuto();
    public Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private static final double TARGET_X = 0;
    private static final double TARGET_Y = 127;
    private static final double SHOOTER_VEL = 2000;
    private static final double GATE_CYCLE_DEADLINE = 21.0;
    private static final double GATE_WAIT_DEADLINE = 27.0;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose shootingPose = new Pose(59.39160839160837, 24.083916083916087, Math.toRadians(90));
    private final Pose colectare = new Pose(14.34965034965035, 35.34965034965035, Math.toRadians(180));
    private final Pose controlPoint = new Pose(64.36713286713285, 37.29370629370628);
    private final Pose tragere2 = new Pose(60.54545454545453, 24.020979020979027, Math.toRadians(90));
    private final Pose controlTragere = new Pose(41.59790209790209, 18.96153846153846, Math.toRadians(90));
    private final Pose luare3 = new Pose(46.013986013986, 12.93006993006996, Math.toRadians(180));
    private final Pose aluat3 = new Pose(9.050699300699305, 10.583916083916108, Math.toRadians(180));
    private final Pose tras3 = new Pose(59.55944055944056, 24.36363636363636, Math.toRadians(90));
    private final Pose iesirea = new Pose(34.71226672976966, 10.97982064609837, Math.toRadians(90));
    private final Pose FutBujia = new Pose(11.423076923076923, 33.75, Math.toRadians(125));
    private final Pose BucoasaBujia = new Pose(60.365384615384606, 23.673076923076913, Math.toRadians(125));

    private Path laShooting;
    private PathChain iale, tragere, iesireas, treiluat, treialuat, treiTras, ColectareGate, FlociBujia;

    private boolean TragereInProgres = false;
    private int ShootingStare = 0;
    private int ShootSlot = 0;
    private int idTag = 0;
    private double originalPosU = 0;
    private static final double[] RECOIL_OFFSETS = {0.0, 0.005, 0.012};

    private volatile boolean[] slotOcupat = new boolean[3];
    private volatile boolean intakePornit = false;
    private volatile boolean stop = false;
    private Thread IntakeThread;
    private Thread TrackingThread;
    private final Object slot = new Object();

    private int getLoculete() {
        synchronized (slot) {
            int count = 0;
            for (boolean occupied : slotOcupat) {
                if (occupied) count++;
            }
            return count;
        }
    }

    private double getAruncarePos(int s) {
        if (s == 0) return Pozitii.aruncare1;
        if (s == 1) return Pozitii.aruncare2;
        return Pozitii.aruncare3;
    }

    public void buildPaths() {
        laShooting = new Path(new BezierLine(startPose, shootingPose));
        laShooting.setConstantHeadingInterpolation(startPose.getHeading());

        iale = follower.pathBuilder()
                .addPath(new BezierCurve(shootingPose, controlPoint, colectare))
                .setTangentHeadingInterpolation()
                .setBrakingStrength(1.5)
                .build();
        tragere = follower.pathBuilder()
                .addPath(new BezierCurve(colectare, controlTragere, tragere2))
                .setLinearHeadingInterpolation(colectare.getHeading(), tragere2.getHeading())
                .setHeadingConstraint(2)
                .setBrakingStrength(1.0)
                .setTimeoutConstraint(50)
                .build();
        treiluat = follower.pathBuilder()
                .addPath(new BezierLine(tragere2, luare3))
                .setLinearHeadingInterpolation(tragere2.getHeading(), luare3.getHeading())
                .setHeadingConstraint(2)
                .setTimeoutConstraint(50)
                .build();
        treialuat = follower.pathBuilder()
                .addPath(new BezierLine(luare3, aluat3))
                .setLinearHeadingInterpolation(luare3.getHeading(), aluat3.getHeading())
                .setHeadingConstraint(2)
                .setTimeoutConstraint(50)
                .build();
        treiTras = follower.pathBuilder()
                .addPath(new BezierLine(aluat3, tras3))
                .setLinearHeadingInterpolation(aluat3.getHeading(), tras3.getHeading())
                .setHeadingConstraint(2)
                .setBrakingStrength(1.0)
                .setTimeoutConstraint(50)
                .build();
        ColectareGate = follower.pathBuilder()
                .addPath(new BezierLine(tras3, FutBujia))
                .setLinearHeadingInterpolation(tras3.getHeading(), FutBujia.getHeading())
                .build();
        FlociBujia = follower.pathBuilder()
                .addPath(new BezierLine(FutBujia, BucoasaBujia))
                .setLinearHeadingInterpolation(FutBujia.getHeading(), BucoasaBujia.getHeading())
                .build();
        iesireas = follower.pathBuilder()
                .addPath(new BezierLine(tras3, iesirea))
                .setLinearHeadingInterpolation(tras3.getHeading(), iesirea.getHeading())
                .build();
    }

    private void pregatireShooter() {
        n.applyVoltageCompensatedPIDF();
        n.shooter.setVelocity(SHOOTER_VEL);
        n.shooter2.setVelocity(SHOOTER_VEL);
        n.unghiD.setPosition(pop.posUnghi);
    }

    private void TragereLaPupitru() {
        switch (ShootingStare) {
            case 0:
                ShootSlot = 0;
                originalPosU = pop.posUnghi;
                n.scula.setPower(-1);
                n.bascula.setPosition(Pozitii.lansareRapid);
                n.applyVoltageCompensatedPIDF();
                n.shooter.setVelocity(SHOOTER_VEL);
                n.shooter2.setVelocity(SHOOTER_VEL);
                n.unghiD.setPosition(originalPosU);
                idTag = n.detectIdTag();
                ShootingStare = 1;
                break;

            case 1:
                if (ShootSlot <= 2) {
                    ShootingStare = 2;
                } else {
                    ShootingStare = 6;
                }
                break;

            case 2: {
                double velScale = SHOOTER_VEL / 1550.0;
                double recoil = RECOIL_OFFSETS[Math.min(ShootSlot, 2)] * velScale;
                n.unghiD.setPosition(originalPosU + recoil);
                n.sortare.setPosition(getAruncarePos(ShootSlot));
                actionTimer.resetTimer();
                ShootingStare = 3;
                break;
            }

            case 3:
                if (actionTimer.getElapsedTimeSeconds() >= 0.08) {
                    boolean tagVisible = n.isTagVisible();
                    double tx = Math.abs(n.getLimelightTx());
                    double v1 = Math.abs(n.shooter.getVelocity());
                    double v2 = Math.abs(n.shooter2.getVelocity());
                    boolean velocityOk = v1 >= SHOOTER_VEL * 0.95 && v2 >= SHOOTER_VEL * 0.95;
                    if ((tagVisible && tx < 1.0 && velocityOk) || actionTimer.getElapsedTimeSeconds() >= 0.40) {
                        ShootingStare = 4;
                    }
                }
                break;

            case 4:
                actionTimer.resetTimer();
                ShootingStare = 5;
                break;

            case 5:
                if (actionTimer.getElapsedTimeSeconds() >= 0.08) {
                    synchronized (slot) {
                        slotOcupat[ShootSlot] = false;
                    }
                    ShootSlot++;
                    ShootingStare = 1;
                }
                break;

            case 6:
                n.unghiD.setPosition(originalPosU);
                n.bascula.setPosition(Pozitii.sede);
                n.scula.setPower(0);
                n.intake.setPower(0);
                n.sortare.setPosition(Pozitii.luarea1);
                actionTimer.resetTimer();
                ShootingStare = 7;
                break;

            case 7:
                if (actionTimer.getElapsedTimeSeconds() >= 0.03) {
                    n.shooter.setVelocity(0);
                    n.shooter2.setVelocity(0);
                    ShootingStare = 8;
                }
                break;

            case 8:
                synchronized (slot) {
                    slotOcupat[0] = false;
                    slotOcupat[1] = false;
                    slotOcupat[2] = false;
                }
                TragereInProgres = false;
                ShootingStare = 0;
                break;
        }
    }

    private void startTracking() {
        TrackingThread = new Thread(() -> {
            while (!stop) {
                try { Thread.sleep(10); } catch (InterruptedException e) { break; }
                n.tracks(follower, TARGET_X, TARGET_Y);
            }
            n.turelaD.setPosition(0.5);
            n.turelaS.setPosition(0.5);
        });
        TrackingThread.start();
    }

    private void Intake() {
        IntakeThread = new Thread(new Runnable() {
            private boolean ballBeingProcessed = false;
            private long lastDistReadTime = 0;

            @Override
            public void run() {
                while (!stop) {
                    try { Thread.sleep(5); } catch (InterruptedException e) { break; }
                    long now = System.currentTimeMillis();
                    if (now - lastDistReadTime >= 50) {
                        n.cachedDistanta = n.distanta.getDistance(DistanceUnit.CM);
                        lastDistReadTime = now;
                    }
                    double leDistanta = n.cachedDistanta;
                    int loculete = getLoculete();
                    if (intakePornit && loculete < 3) {
                        n.intake.setPower(1);

                        if (leDistanta < 20 && !ballBeingProcessed) {
                            ballBeingProcessed = true;

                            double servoPos = n.sortare.getPosition();

                            if (Math.abs(servoPos - Pozitii.luarea1) < 0.1 && !slotOcupat[0]) {
                                slotOcupat[0] = true;
                                if (!slotOcupat[1]) {
                                    n.sortare.setPosition(Pozitii.luarea2);
                                } else if (!slotOcupat[2]) {
                                    n.sortare.setPosition(Pozitii.luarea3);
                                }
                                n.kdf(80);
                            } else if (Math.abs(servoPos - Pozitii.luarea2) < 0.1 && !slotOcupat[1]) {
                                slotOcupat[1] = true;
                                if (!slotOcupat[2]) {
                                    n.sortare.setPosition(Pozitii.luarea3);
                                } else if (!slotOcupat[0]) {
                                    n.sortare.setPosition(Pozitii.luarea1);
                                }
                                n.kdf(80);
                            } else if (Math.abs(servoPos - Pozitii.luarea3) < 0.1 && !slotOcupat[2]) {
                                slotOcupat[2] = true;
                                if (!slotOcupat[0]) {
                                    n.sortare.setPosition(Pozitii.luarea1);
                                } else if (!slotOcupat[1]) {
                                    n.sortare.setPosition(Pozitii.luarea2);
                                }
                                n.kdf(80);
                            }

                        } else if (leDistanta >= 20 && ballBeingProcessed) {
                            ballBeingProcessed = false;
                        }
                    } else if (!intakePornit) {
                        n.intake.setPower(0);
                        ballBeingProcessed = false;
                    }
                }
            }
        });
    }

    private void parcheazaDinPozCurenta() {
        Pose cur = follower.getPose();
        PathChain parkPath = follower.pathBuilder()
                .addPath(new BezierLine(cur, iesirea))
                .setLinearHeadingInterpolation(cur.getHeading(), iesirea.getHeading())
                .build();
        follower.followPath(parkPath, false);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                idTag = n.detectIdTag();
                follower.followPath(laShooting);
                pregatireShooter();
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= 3.0) {
                    follower.holdPoint(shootingPose);
                    setPathState(2);
                }
                break;

            case 2:
                if (!TragereInProgres) {
                    TragereInProgres = true;
                    ShootingStare = 0;
                }
                TragereLaPupitru();
                if (!TragereInProgres) {
                    setPathState(3);
                }
                break;

            case 3:
                synchronized (slot) {
                    slotOcupat[0] = false;
                    slotOcupat[1] = false;
                    slotOcupat[2] = false;
                }
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(iale, 0.7, false);
                setPathState(5);
                break;

            case 5:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= 4.0) {
                    follower.holdPoint(colectare);
                    actionTimer.resetTimer();
                    setPathState(6);
                }
                break;

            case 6:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 1.5) {
                    intakePornit = false;
                    if (getLoculete() > 0) {
                        pregatireShooter();
                        follower.followPath(tragere, 0.7, false);
                        setPathState(8);
                    } else {
                        parcheazaDinPozCurenta();
                        setPathState(70);
                    }
                }
                break;

            case 8:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= 4.0) {
                    follower.holdPoint(tragere2);
                    setPathState(9);
                }
                break;

            case 9:
                if (!TragereInProgres) {
                    TragereInProgres = true;
                    ShootingStare = 0;
                }
                TragereLaPupitru();
                if (!TragereInProgres) {
                    setPathState(10);
                }
                break;

            case 10:
                synchronized (slot) {
                    slotOcupat[0] = false;
                    slotOcupat[1] = false;
                    slotOcupat[2] = false;
                }
                n.sortare.setPosition(Pozitii.luarea1);
                follower.followPath(treiluat);
                setPathState(11);
                break;

            case 11:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= 3.5) {
                    follower.holdPoint(luare3);
                    n.intake.setPower(-1);
                    actionTimer.resetTimer();
                    setPathState(12);
                }
                break;

            case 12:
                if (actionTimer.getElapsedTimeSeconds() >= 0.15) {
                    n.intake.setPower(0);
                    intakePornit = true;
                    follower.followPath(treialuat);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= 3.5) {
                    follower.holdPoint(aluat3);
                    actionTimer.resetTimer();
                    setPathState(14);
                }
                break;

            case 14:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 1.5) {
                    intakePornit = false;
                    if (getLoculete() > 0) {
                        pregatireShooter();
                        follower.followPath(treiTras);
                        setPathState(15);
                    } else {
                        parcheazaDinPozCurenta();
                        setPathState(70);
                    }
                }
                break;

            case 15:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= 3.5) {
                    follower.holdPoint(tras3);
                    setPathState(16);
                }
                break;

            case 16:
                if (!TragereInProgres) {
                    TragereInProgres = true;
                    ShootingStare = 0;
                }
                TragereLaPupitru();
                if (!TragereInProgres) {
                    setPathState(50);
                }
                break;

            case 50: {
                synchronized (slot) {
                    slotOcupat[0] = false;
                    slotOcupat[1] = false;
                    slotOcupat[2] = false;
                }
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                Pose cur = follower.getPose();
                PathChain toGate = follower.pathBuilder()
                        .addPath(new BezierLine(cur, FutBujia))
                        .setLinearHeadingInterpolation(cur.getHeading(), FutBujia.getHeading())
                        .build();
                follower.followPath(toGate);
                setPathState(51);
                break;
            }

            case 51:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= 3.5) {
                    follower.holdPoint(FutBujia);
                    actionTimer.resetTimer();
                    setPathState(52);
                }
                break;

            case 52:
                if (opmodeTimer.getElapsedTimeSeconds() >= GATE_WAIT_DEADLINE) {
                    intakePornit = false;
                    parcheazaDinPozCurenta();
                    setPathState(70);
                    break;
                }
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 1.5) {
                    intakePornit = false;
                    if (getLoculete() > 0) {
                        pregatireShooter();
                        follower.followPath(FlociBujia);
                        setPathState(53);
                    } else {
                        parcheazaDinPozCurenta();
                        setPathState(70);
                    }
                }
                break;

            case 53:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= 3.5) {
                    follower.holdPoint(BucoasaBujia);
                    setPathState(54);
                }
                break;

            case 54:
                if (!TragereInProgres) {
                    TragereInProgres = true;
                    ShootingStare = 0;
                }
                TragereLaPupitru();
                if (!TragereInProgres) {
                    if (opmodeTimer.getElapsedTimeSeconds() < GATE_CYCLE_DEADLINE) {
                        setPathState(50);
                    } else {
                        parcheazaDinPozCurenta();
                        setPathState(70);
                    }
                }
                break;

            case 70:
                n.shooter.setVelocity(0);
                n.shooter2.setVelocity(0);
                intakePornit = false;
                if (TrackingThread != null) TrackingThread.interrupt();
                n.turelaD.setPosition(0.5);
                n.turelaS.setPosition(0.5);
                if (!follower.isBusy()) {
                    follower.holdPoint(iesirea);
                    setPathState(-1);
                }
                break;

            default:
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        if (opmodeTimer.getElapsedTimeSeconds() >= 29.5) {
            n.shooter.setVelocity(0);
            n.shooter2.setVelocity(0);
            n.intake.setPower(0);
            n.scula.setPower(0);
            stop = true;
            if (TrackingThread != null) TrackingThread.interrupt();
            n.turelaD.setPosition(0.5);
            n.turelaS.setPosition(0.5);
            requestOpModeStop();
            return;
        }
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Bile", getLoculete());
        telemetry.addData("Timp", "%.1f", opmodeTimer.getElapsedTimeSeconds());
        for (int i = 0; i < 3; i++) {
            telemetry.addData("Slot " + (i + 1), slotOcupat[i] ? "plin" : "gol");
        }
        telemetry.addData("ID", idTag);
        telemetry.addData("LL tx", "%.2f", n.getLimelightTx());
        telemetry.addData("Tag", n.isTagVisible() ? "DA" : "NU");
        telemetry.update();
    }

    @Override
    public void init() {
        n.initsisteme(hardwareMap);

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.recalibrateIMU();
        ElapsedTime calibTimer = new ElapsedTime();
        while (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                && calibTimer.milliseconds() < 2000) {
        }

        buildPaths();
        follower.setStartingPose(startPose);
        follower.update();

        n.limelight.pipelineSwitch(1);
    }

    @Override
    public void init_loop() {
        idTag = n.detectIdTag();
        telemetry.addData("Limelight ID", idTag);
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        TragereInProgres = false;
        stop = false;
        intakePornit = false;

        slotOcupat[0] = true;
        slotOcupat[1] = true;
        slotOcupat[2] = true;

        n.sortare.setPosition(Pozitii.luarea1);
        n.limelight.pipelineSwitch(0);
        n.turelaOffsetDeg = -25.0;
        n.resetTurelaPID();

        Intake();
        IntakeThread.start();
        startTracking();
    }

    @Override
    public void stop() {
        stop = true;

        if (TrackingThread != null) {
            TrackingThread.interrupt();
        }
        if (IntakeThread != null) {
            IntakeThread.interrupt();
        }

        Pose currentPose = follower.getPose();
        RobotPozitie.X = currentPose.getX();
        RobotPozitie.Y = currentPose.getY();
        RobotPozitie.heading = currentPose.getHeading();
        RobotPozitie.idTag = idTag;

        n.shooter.setVelocity(0);
        n.shooter2.setVelocity(0);
        n.intake.setPower(0);
        n.scula.setPower(0);
        n.turelaD.setPosition(0.5);
        n.turelaS.setPosition(0.5);
        n.bascula.setPosition(0.5807);
    }
}
