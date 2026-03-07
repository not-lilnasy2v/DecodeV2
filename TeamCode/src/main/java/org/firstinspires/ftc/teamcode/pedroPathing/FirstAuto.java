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
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Pozitii;
import org.firstinspires.ftc.teamcode.RobotPozitie;
import org.firstinspires.ftc.teamcode.pop;
import org.firstinspires.ftc.teamcode.sistemeAuto;

@Autonomous
public class FirstAuto extends OpMode {
    sistemeAuto n = new sistemeAuto();
    public Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private static final double TARGET_X = 0;
    private static final double TARGET_Y = 144;

    private final Pose startPose = new Pose(24.503496503496507, 128.8951048951049, Math.toRadians(142));
    private final Pose tragere1 = new Pose(60.72027972027973, 95.74825174825179, Math.toRadians(180));
    private final Pose aduna1 = new Pose(59.3076923076923, 89.93706293706293, Math.toRadians(180));
    private final Pose aluat1 = new Pose(23.55244755244755, 89.05804195804195, Math.toRadians(180));
    private final Pose tras1 = new Pose(58.72027972027973, 95.74825174825179, Math.toRadians(180));
    private final Pose aduna2 = new Pose(70.97403296004696, 62.21209786223014, Math.toRadians(180));
    private final Pose aluat2 = new Pose(18.69230769230769, 62.75055940069167, Math.toRadians(180));
    private final Pose curburaMiti  = new Pose(47.99352046554845,70.20678735353181,Math.toRadians(180));
    private final Pose tras2 = new Pose(60.72027972027973, 95.74825174825179, Math.toRadians(180));
    private final Pose tras3 = new Pose(60.72027972027973, 95.74825174825179, Math.toRadians(180));
    private final Pose returnToBase = new Pose(28.790209790209786,93.66433566433567,Math.toRadians(180));
    private final Pose deschideCombinat = new Pose(17.269230769230774, 68.76923076923079, Math.toRadians(150));
    private final Pose CPdeschideCombinat = new Pose(50.66346153846153, 64.85576923076923);
    private final Pose lansareGate = new Pose(60.69584837545127, 95.60351291307968,Math.toRadians(180));
    private final Pose CPlansareGate = new Pose(68.69685504026661, 61.81897389613998);
    private final Pose preGate = new Pose(25.250000000000004, 74.13461538461539, Math.toRadians(180));
    private PathChain scorePreload;
    private PathChain collectare1, trasUnu, collectare2, trasDoi, /* collectare3, trasTrei, */ returnarea, DuceGate, Trage_Gata;
    private boolean TragereInProgres = false;
    private int ShootingStare = 0;
    private int idTag = 0;
    private volatile boolean[] slotOcupat = new boolean[3];
    private synchronized int getLoculete() {
        int count = 0;
        for (boolean occupied : slotOcupat) {
            if (occupied) count++;
        }
        return count;
    }
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, tragere1))
                .setLinearHeadingInterpolation(startPose.getHeading(), tragere1.getHeading())
                .setGlobalDeceleration(3.5)
                .setBrakingStart(1.8)
                .build();

        collectare1 = follower.pathBuilder()
                .addPath(new BezierCurve(tras3, aduna1, aluat1))
                .setLinearHeadingInterpolation(tras3.getHeading(), aluat1.getHeading())
                .build();

        trasUnu = follower.pathBuilder()
                .addPath(new BezierLine(aluat1, tras1))
                .setLinearHeadingInterpolation(aluat1.getHeading(), tras1.getHeading())
                .setTranslationalConstraint(1)
                .setTimeoutConstraint(50)
                .setGlobalDeceleration(3.5)
                .setBrakingStart(1.8)
                .build();

        collectare2 = follower.pathBuilder()
                .addPath(new BezierCurve(tragere1, aduna2, aluat2))
                .setLinearHeadingInterpolation(tragere1.getHeading(), aluat2.getHeading())
                .build();

        trasDoi = follower.pathBuilder()
                .addPath(new BezierCurve(aluat2,curburaMiti ,tras2))
                .setLinearHeadingInterpolation(aluat2.getHeading(), tras2.getHeading())
                .setTranslationalConstraint(1)
                .setGlobalDeceleration(3.5)
                .setBrakingStart(1.8)
                .setTimeoutConstraint(70)
                .build();
        // collectare3 = follower.pathBuilder()
        //         .addPath(new BezierCurve(lansareGate, aduna3,aluat3))
        //         .setLinearHeadingInterpolation(lansareGate.getHeading(), aluat3.getHeading())
        //         .build();

        // trasTrei = follower.pathBuilder()
        //         .addPath(new BezierLine(aluat3, tras3))
        //         .setLinearHeadingInterpolation(aluat3.getHeading(), tras3.getHeading())
        //         .setTranslationalConstraint(1)p
        //         .setTimeoutConstraint(100)
        //         .setBrakingStrength(2.8)
        //         .build();
        returnarea=  follower.pathBuilder()
                .addPath(new BezierLine(tras1,returnToBase))
                .setLinearHeadingInterpolation(tras1.getHeading(),returnToBase.getHeading())
                .build();

        Trage_Gata = follower.pathBuilder()
                .addPath(new BezierCurve(deschideCombinat,CPlansareGate,lansareGate))
                .setLinearHeadingInterpolation(deschideCombinat.getHeading(),lansareGate.getHeading())
                .setGlobalDeceleration(3.5)
                .setBrakingStart(1.8)
                .build();

        DuceGate= follower.pathBuilder()
                .addPath(new BezierCurve(tragere1,CPdeschideCombinat, preGate))
                .setLinearHeadingInterpolation(tragere1.getHeading(), preGate.getHeading())
                .build();
    }

    private void pregatireShooter() {
        n.applyVoltageCompensatedPIDF();
        n.shooter.setVelocity(1550);
        n.shooter2.setVelocity(1550);
        n.unghiD.setPosition(pop.posUnghi);
        n.sortare.setPosition(Pozitii.aruncare1);
    }

    private int ShootSlot = 2;
    private static final int[] SHOOT_ORDER = {0, 2, 1};
    private static final double[] SHOOT_POS = {Pozitii.aruncare1, Pozitii.aruncare3, Pozitii.aruncare2};

    private void TragereLaPupitru() {
        switch (ShootingStare) {
            case 0:
                ShootSlot = 0;
                n.scula.setPower(-1);
                n.bascula.setPosition(Pozitii.lansareRapid);
                n.applyVoltageCompensatedPIDF();
                n.shooter.setVelocity(1550);
                n.shooter2.setVelocity(1550);
                n.unghiD.setPosition(pop.posUnghi);
                idTag = n.detectIdTag();
                actionTimer.resetTimer();
                ShootingStare = 5;
                break;
            case 5:
                double minWait = ShootSlot == 0 ? 0.15 : 0.03;
                double maxWait = ShootSlot == 0 ? 0.40 : 0.20;
                if (actionTimer.getElapsedTimeSeconds() >= minWait) {
                    boolean tagVisible = n.isTagVisible();
                    double tx = Math.abs(n.getLimelightTx());
                    boolean ready;
                    if (ShootSlot == 0) {
                        double v1 = Math.abs(n.shooter.getVelocity());
                        double v2 = Math.abs(n.shooter2.getVelocity());
                        ready = tagVisible && tx < 1.0
                                && v1 >= 1550 * 0.96 && v1 <= 1550 * 1.03
                                && v2 >= 1550 * 0.96 && v2 <= 1550 * 1.03;
                    } else {
                        ready = tagVisible && tx < 1.0;
                    }
                    if (ready || actionTimer.getElapsedTimeSeconds() >= maxWait) {
                        slotOcupat[SHOOT_ORDER[ShootSlot]] = false;
                        ShootSlot++;
                        if (ShootSlot > 2) {
                            n.bascula.setPosition(Pozitii.sede);
                            n.scula.setPower(0);
                            n.shooter.setVelocity(0);
                            n.shooter2.setVelocity(0);
                            n.sortare.setPosition(Pozitii.luarea1);
                            TragereInProgres = false;
                            ShootingStare = 0;
                        } else {
                            n.sortare.setPosition(SHOOT_POS[ShootSlot]);
                            actionTimer.resetTimer();
                        }
                    }
                }
                break;
        }
    }

    private volatile boolean intakePornit = false;
    private volatile boolean stop = false;
    private Thread Intake;
    private Thread TrackingT;

    private void Intake() {
        Intake = new Thread(new Runnable() {
            private boolean ballBeingProcessed = false;
            private long lastDistReadTime = 0;

            @Override
            public void run() {
                while (!stop) {
                    try { Thread.sleep(10); } catch (InterruptedException e) { break; }
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

    private void startTracking() {
        TrackingT = new Thread(() -> {
            while (!stop) {
                try { Thread.sleep(5); } catch (InterruptedException e) { break; }
                n.tracks(follower, TARGET_X, TARGET_Y);
            }
            n.turelaD.setPosition(0.5);
            n.turelaS.setPosition(0.5);
        });
        TrackingT.start();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                pregatireShooter();
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.holdPoint(tragere1);
                    TragereInProgres = true;
                    ShootingStare = 0;
                    setPathState(2);
                }
                break;

            case 2:
                TragereLaPupitru();
                if (!TragereInProgres) {
                    setPathState(3);
                }
                break;

            case 3:
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(collectare2);
                setPathState(4);
                break;

            case 4:
                if (getLoculete() >= 3) {
                    intakePornit = false;
                    setPathState(6);
                } else if (!follower.isBusy()) {
                    follower.holdPoint(aluat2);
                    actionTimer.resetTimer();
                    setPathState(5);
                }
                break;

            case 5:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 0.8) {
                    intakePornit = false;
                    setPathState(6);
                }
                break;

            case 6:
                intakePornit = false;
                pregatireShooter();
                follower.followPath(trasDoi);
                setPathState(7);
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.holdPoint(tras2);
                    TragereInProgres = true;
                    ShootingStare = 0;
                    setPathState(8);
                }
                break;

            case 8:
                TragereLaPupitru();
                if (!TragereInProgres) {
                    setPathState(9);
                }
                break;

            case 9:
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(DuceGate);
                setPathState(10);
                break;

            case 10:
                if (getLoculete() >= 3) {
                    intakePornit = false;
                    pregatireShooter();
                    follower.followPath(Trage_Gata);
                    setPathState(12);
                } else if (!follower.isBusy()) {
                    follower.holdPoint(deschideCombinat);
                    actionTimer.resetTimer();
                    setPathState(11);
                }
                break;

            case 11:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 2.5) {
                    intakePornit = false;
                    pregatireShooter();
                    follower.followPath(Trage_Gata);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    follower.holdPoint(lansareGate);
                    TragereInProgres = true;
                    ShootingStare = 0;
                    setPathState(13);
                }
                break;

            case 13:
                TragereLaPupitru();
                if (!TragereInProgres) {
                    setPathState(30);
                }
                break;

            case 15:
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(collectare1);
                setPathState(16);
                break;

            case 16:
                if (getLoculete() >= 3) {
                    intakePornit = false;
                    setPathState(18);
                } else if (!follower.isBusy()) {
                    follower.holdPoint(aluat1);
                    actionTimer.resetTimer();
                    setPathState(17);
                }
                break;

            case 17:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 0.7) {
                    intakePornit = false;
                    setPathState(18);
                }
                break;

            case 18:
                intakePornit = false;
                pregatireShooter();
                follower.followPath(trasUnu);
                setPathState(19);
                break;

            case 19:
                if (!follower.isBusy()) {
                    follower.holdPoint(tras1);
                    TragereInProgres = true;
                    ShootingStare = 0;
                    setPathState(20);
                }
                break;

            case 20:
                TragereLaPupitru();
                if (!TragereInProgres) {
                    follower.followPath(returnarea, false);
                    setPathState(21);
                }
                break;

            case 30:
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(DuceGate);
                setPathState(31);
                break;

            case 31:
                if (getLoculete() >= 3) {
                    intakePornit = false;
                    pregatireShooter();
                    follower.followPath(Trage_Gata);
                    setPathState(33);
                } else if (!follower.isBusy()) {
                    follower.holdPoint(deschideCombinat);
                    actionTimer.resetTimer();
                    setPathState(32);
                }
                break;

            case 32:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 2.5) {
                    intakePornit = false;
                    pregatireShooter();
                    follower.followPath(Trage_Gata);
                    setPathState(33);
                }
                break;

            case 33:
                if (!follower.isBusy()) {
                    follower.holdPoint(lansareGate);
                    TragereInProgres = true;
                    ShootingStare = 0;
                    setPathState(34);
                }
                break;

            case 34:
                TragereLaPupitru();
                if (!TragereInProgres) {
                    setPathState(15);
                }
                break;

            case 21:
                if (!follower.isBusy()) {
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
            requestOpModeStop();
            return;
        }

        follower.update();

        telemetry.addData("pathState", pathState);
        telemetry.addData("shootState", ShootingStare);
        telemetry.addData("servo", n.sortare.getPosition());
        telemetry.addData("slot", slotOcupat[0] + " " + slotOcupat[1] + " " + slotOcupat[2]);
        telemetry.addData("LL tx", "%.2f°", n.getLimelightTx());
        telemetry.addData("Tag vizibil", n.isTagVisible() ? "DA" : "NU");
        telemetry.addData("Timp", "%.1f s", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
        autonomousPathUpdate();
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
    }

    @Override
    public void init_loop() {
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

        n.resetTurelaPID();
        n.applyVoltageCompensatedPIDF();
        n.shooter.setVelocity(1550);
        n.shooter2.setVelocity(1550);
        Intake();
        Intake.start();
        startTracking();
    }

    @Override
    public void stop() {
        stop = true;

        if (TrackingT != null) {
            TrackingT.interrupt();
        }
        if (Intake != null) {
            Intake.interrupt();
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
        n.bascula.setPosition(0.5807);
    }
}
