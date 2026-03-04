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
    private final Pose ARatat1 = new Pose(71.87014034916132,68.10259670399529, Math.toRadians(180));
    private final Pose tras1 = new Pose(58.72027972027973, 95.74825174825179, Math.toRadians(180));
    private final Pose aduna2 = new Pose(70.97403296004696, 62.21209786223014, Math.toRadians(180));
    private final Pose aluat2 = new Pose(19.69230769230769, 62.75055940069167, Math.toRadians(180));
    private final Pose ARatat2 = new Pose(71.54895104895104,48.56293706293705, Math.toRadians(180));
    private final Pose curburaMiti  = new Pose(47.99352046554845,70.20678735353181,Math.toRadians(180));
    private final Pose tras2 = new Pose(60.72027972027973, 95.74825174825179, Math.toRadians(180));
    private final Pose aduna3 = new Pose(62.65384615384616, 36.17307692307692, Math.toRadians(180));
    private final Pose aluat3 = new Pose(15.81118881118881, 38.34825174825171, Math.toRadians(180));
    private final Pose tras3 = new Pose(60.72027972027973, 95.74825174825179, Math.toRadians(180));
    private final Pose returnToBase = new Pose(28.790209790209786,93.66433566433567,Math.toRadians(180));
    private final Pose deschideCombinat = new Pose(18.038461538461544, 65.96153846153848, Math.toRadians(120));
    private final Pose CPdeschideCombinat = new Pose(57.41346153846154, 89.77884615384615);
    private final Pose CPdeschideCombinat2 = new Pose(35.52884615384617, 40.73076923076924);
    private final Pose lansareGate = new Pose(60.69584837545127, 95.60351291307968,Math.toRadians(180));
    private final Pose CPlansareGate = new Pose(34.867154956956405, 82.78252568730909);
    private Path scorePreload;
    private PathChain collectare1, trasUnu, collectare2, trasDoi, collectare3, trasTrei, returnarea, DuceGate, Trage_Gata;
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
        scorePreload = new Path(new BezierLine(startPose, tragere1));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), tragere1.getHeading());

        collectare1 = follower.pathBuilder()
                .addPath(new BezierCurve(tras3, aduna1, aluat1))
                .setLinearHeadingInterpolation(tras3.getHeading(), aluat1.getHeading())
                .build();

        trasUnu = follower.pathBuilder()
                .addPath(new BezierLine(aluat1, tras1))
                .setLinearHeadingInterpolation(aluat1.getHeading(), tras1.getHeading())
                .setTranslationalConstraint(1)
                .setTimeoutConstraint(50)
                .build();

        collectare2 = follower.pathBuilder()
                .addPath(new BezierCurve(tragere1, aduna2, aluat2))
                .setLinearHeadingInterpolation(tragere1.getHeading(), aluat2.getHeading())
                .build();

        trasDoi = follower.pathBuilder()
                .addPath(new BezierCurve(aluat2,curburaMiti ,tras2))
                .setLinearHeadingInterpolation(aluat2.getHeading(), tras2.getHeading())
                .setTranslationalConstraint(1)
                .setTimeoutConstraint(70)
                .build();
        collectare3 = follower.pathBuilder()
                .addPath(new BezierCurve(lansareGate, aduna3,aluat3))
                .setLinearHeadingInterpolation(lansareGate.getHeading(), aluat3.getHeading())
                .build();

        trasTrei = follower.pathBuilder()
                .addPath(new BezierLine(aluat3, tras3))
                .setLinearHeadingInterpolation(aluat3.getHeading(), tras3.getHeading())
                .setTranslationalConstraint(1)
                .setTimeoutConstraint(100)
                .build();
        returnarea=  follower.pathBuilder()
                .addPath(new BezierLine(tras1,returnToBase))
                .setLinearHeadingInterpolation(tras1.getHeading(),returnToBase.getHeading())
                .build();

        Trage_Gata = follower.pathBuilder()
                .addPath(new BezierCurve(deschideCombinat,CPlansareGate,lansareGate))
                .setLinearHeadingInterpolation(deschideCombinat.getHeading(),lansareGate.getHeading())
                .build();

        DuceGate= follower.pathBuilder()
                .addPath(new BezierCurve(tragere1,CPdeschideCombinat,CPdeschideCombinat2, deschideCombinat))
                .setLinearHeadingInterpolation(tragere1.getHeading(),deschideCombinat.getHeading())
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
                ShootingStare = 3;
                break;

            case 3:
                if (ShootSlot <= 2) {
                    ShootingStare = 4;
                } else {
                    ShootingStare = 18;
                }
                break;

            case 4:
                double target;
                if (ShootSlot == 0) target = Pozitii.aruncare1;
                else if (ShootSlot == 1) target = Pozitii.aruncare2;
                else target = Pozitii.aruncare3;
                n.sortare.setPosition(target);
                actionTimer.resetTimer();
                ShootingStare = 5;
                break;

            case 5:
                if (actionTimer.getElapsedTimeSeconds() >= 0.10) {
                    boolean tagVisible = n.isTagVisible();
                    double tx = Math.abs(n.getLimelightTx());
                    double v1 = Math.abs(n.shooter.getVelocity());
                    double v2 = Math.abs(n.shooter2.getVelocity());
                    boolean velocityOk = v1 >= 1550 * 0.95 && v2 >= 1550 * 0.95;
                    if ((tagVisible && tx < 1.0 && velocityOk) || actionTimer.getElapsedTimeSeconds() >= 0.85) {
                        ShootingStare = 7;
                    }
                }
                break;

            case 7:
                actionTimer.resetTimer();
                ShootingStare = 8;
                break;

            case 8:
                if (actionTimer.getElapsedTimeSeconds() >= 0.07) {
                    ShootingStare = 9;
                }
                break;

            case 9:
                actionTimer.resetTimer();
                ShootingStare = 10;
                break;

            case 10:
                if (actionTimer.getElapsedTimeSeconds() >= 0.03) {
                    slotOcupat[ShootSlot] = false;
                    ShootSlot++;
                    ShootingStare = 3;
                }
                break;

            case 18:
                n.bascula.setPosition(Pozitii.sede);
                n.scula.setPower(0);
                n.intake.setPower(0);
                n.sortare.setPosition(Pozitii.luarea1);
                actionTimer.resetTimer();
                ShootingStare = 19;
                break;

            case 19:
                if (actionTimer.getElapsedTimeSeconds() >= 0.15) {
                    n.shooter.setVelocity(0);
                    n.shooter2.setVelocity(0);
                    ShootingStare = 20;
                }
                break;

            case 20:
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;
                TragereInProgres = false;
                ShootingStare = 0;
                break;
        }
    }

    private volatile boolean intakePornit = false;
    private volatile boolean stop = false;
    private Thread IntakeThread;
    private Thread TrackingThread;

    private void Intake() {
        IntakeThread = new Thread(new Runnable() {
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
                                n.kdf(150);

                            } else if (Math.abs(servoPos - Pozitii.luarea2) < 0.1 && !slotOcupat[1]) {
                                slotOcupat[1] = true;
                                if (!slotOcupat[2]) {
                                    n.sortare.setPosition(Pozitii.luarea3);
                                } else if (!slotOcupat[0]) {
                                    n.sortare.setPosition(Pozitii.luarea1);
                                }
                                n.kdf(150);

                            } else if (Math.abs(servoPos - Pozitii.luarea3) < 0.1 && !slotOcupat[2]) {
                                slotOcupat[2] = true;
                                if (!slotOcupat[0]) {
                                    n.sortare.setPosition(Pozitii.luarea1);
                                } else if (!slotOcupat[1]) {
                                    n.sortare.setPosition(Pozitii.luarea2);
                                }
                                n.kdf(150);
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
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(collectare2);
                setPathState(4);
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.holdPoint(aluat2);
                    actionTimer.resetTimer();
                    setPathState(5);
                }
                break;

            case 5:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 1.5) {
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
                    setPathState(8);
                }
                break;

            case 8:
                if (!TragereInProgres) {
                    TragereInProgres = true;
                    ShootingStare = 0;
                }
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
                if (!follower.isBusy()) {
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
                    setPathState(13);
                }
                break;

            case 13:
                if (!TragereInProgres) {
                    TragereInProgres = true;
                    ShootingStare = 0;
                }
                TragereLaPupitru();
                if (!TragereInProgres) {
                    setPathState(15);
                }
                break;

            case 15:
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(collectare3);
                setPathState(16);
                break;

            case 16:
                if (!follower.isBusy()) {
                    follower.holdPoint(aluat3);
                    actionTimer.resetTimer();
                    setPathState(17);
                }
                break;

            case 17:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 1.5) {
                    intakePornit = false;
                    setPathState(18);
                }
                break;

            case 18:
                intakePornit = false;
                pregatireShooter();
                follower.followPath(trasTrei);
                setPathState(19);
                break;

            case 19:
                if (!follower.isBusy()) {
                    follower.holdPoint(tras3);
                    setPathState(20);
                }
                break;

            case 20:
                if (!TragereInProgres) {
                    TragereInProgres = true;
                    ShootingStare = 0;
                }
                TragereLaPupitru();
                if (!TragereInProgres) {
                    setPathState(30);
                }
                break;

            case 30:
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(collectare1);
                setPathState(31);
                break;

            case 31:
                if (!follower.isBusy()) {
                    follower.holdPoint(aluat1);
                    actionTimer.resetTimer();
                    setPathState(32);
                }
                break;

            case 32:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 0.9) {
                    intakePornit = false;
                    setPathState(33);
                }
                break;

            case 33:
                intakePornit = false;
                pregatireShooter();
                follower.followPath(trasUnu);
                setPathState(34);
                break;

            case 34:
                if (!follower.isBusy()) {
                    follower.holdPoint(tras1);
                    setPathState(35);
                }
                break;

            case 35:
                if (!TragereInProgres) {
                    TragereInProgres = true;
                    ShootingStare = 0;
                }
                TragereLaPupitru();
                if (!TragereInProgres) {
                    setPathState(21);
                }
                break;

            case 21:
                follower.followPath(returnarea, false);
                setPathState(22);
                break;

            case 22:
                if (!follower.isBusy()) {
                    follower.holdPoint(returnToBase);
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
        n.bascula.setPosition(0.5807);
    }
}