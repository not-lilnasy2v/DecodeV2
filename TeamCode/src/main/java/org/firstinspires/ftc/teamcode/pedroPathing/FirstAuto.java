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
    private static final double TARGET_Y = 157;

    private final Pose startPose = new Pose(24.503496503496507, 128.8951048951049, Math.toRadians(142));
    private final Pose tragere1 = new Pose(60.72027972027973, 95.74825174825179, Math.toRadians(180));
    private final Pose aduna1 = new Pose(59.3076923076923, 89.93706293706293, Math.toRadians(180));
    private final Pose aluat1 = new Pose(23.55244755244755, 89.05804195804195, Math.toRadians(180));
    private final Pose ARatat1 = new Pose(71.87014034916132,68.10259670399529, Math.toRadians(180));
    private final Pose tras1 = new Pose(58.72027972027973, 95.74825174825179, Math.toRadians(180));
    private final Pose aduna2 = new Pose(70.97403296004696, 62.21209786223014, Math.toRadians(180));
    private final Pose aluat2 = new Pose(16.69230769230769, 62.75055940069167, Math.toRadians(180));
    private final Pose ARatat2 = new Pose(71.54895104895104,48.56293706293705, Math.toRadians(180));
    private final Pose curburaMiti  = new Pose(47.99352046554845,70.20678735353181,Math.toRadians(180));
    private final Pose tras2 = new Pose(60.72027972027973, 95.74825174825179, Math.toRadians(180));
    private final Pose aduna3 = new Pose(91.4685314685315, 35.02797202797204, Math.toRadians(180));
    private final Pose aluat3 = new Pose(15.81118881118881, 38.34825174825171, Math.toRadians(180));
    private final Pose tras3 = new Pose(60.72027972027973, 95.74825174825179, Math.toRadians(180));
    private final Pose returnToBase = new Pose(28.790209790209786,93.66433566433567,Math.toRadians(180));



    private Path scorePreload;
    private PathChain collectare1, Miss1, trasUnu, collectare2, Miss2, trasDoi, collectare3, trasTrei,returnarea;

    private boolean TragereInProgres = false;
    private boolean shooterPreparado = false;
    private int ballshoot = 0;
    private int ShootingStare = 0;
    private int flushSlot = 2;
    private int flushRound = 0;
    private long velocityCheckStart = 0;

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
                .addPath(new BezierCurve(tragere1, aduna1, aluat1))
                .setLinearHeadingInterpolation(tragere1.getHeading(), aluat1.getHeading())
                .build();

        Miss1 = follower.pathBuilder()
                .addPath(new BezierCurve(aluat1, ARatat1, aluat2))
                .setLinearHeadingInterpolation(aluat1.getHeading(), aluat2.getHeading())
                .build();

        trasUnu = follower.pathBuilder()
                .addPath(new BezierLine(aluat1, tras1))
                .setLinearHeadingInterpolation(aluat1.getHeading(), tras1.getHeading())
                .setTranslationalConstraint(1)
                .setTimeoutConstraint(50)
                .build();

        collectare2 = follower.pathBuilder()
                .addPath(new BezierCurve(tras1, aduna2, aluat2))
                .setLinearHeadingInterpolation(tras1.getHeading(), aluat2.getHeading())
                .build();

        Miss2 = follower.pathBuilder()
                .addPath(new BezierCurve(aluat2, ARatat2, aluat3))
                .setLinearHeadingInterpolation(aluat2.getHeading(), aluat3.getHeading())
                .build();

        trasDoi = follower.pathBuilder()
                .addPath(new BezierCurve(aluat2,curburaMiti ,tras2))
                .setLinearHeadingInterpolation(aluat2.getHeading(), tras2.getHeading())
                .setTranslationalConstraint(1)
                .setBrakingStrength(1)
                .setTimeoutConstraint(70)
                .build();
        collectare3 = follower.pathBuilder()
                .addPath(new BezierCurve(tras2, aduna3, aluat3))
                .setLinearHeadingInterpolation(tras2.getHeading(), aluat3.getHeading())
                .build();

        trasTrei = follower.pathBuilder()
                .addPath(new BezierLine(aluat3, tras3))
                .setLinearHeadingInterpolation(aluat3.getHeading(), tras3.getHeading())
                .setTranslationalConstraint(1)
                .setBrakingStrength(1)
                .setTimeoutConstraint(100)
                .build();
        returnarea=  follower.pathBuilder()
                .addPath(new BezierLine(tras3,returnToBase))
                .setLinearHeadingInterpolation(tras3.getHeading(),returnToBase.getHeading())
                .build();
    }

    private void pregatireShooter() {
        if (!shooterPreparado) {
            n.applyVoltageCompensatedPIDF();
            n.shooter.setVelocity(1550);
            n.shooter2.setVelocity(1550);
            n.unghiD.setPosition(pop.posUnghi);
            shooterPreparado = true;
        }
    }

    private int ShootSlot = 2;
    private static final double SHOOTER_VEL = 1550;

    private void TragereLaPupitru() {
        switch (ShootingStare) {
            case 0:
                ballshoot = 3;
                ShootSlot = 2;
                n.scula.setPower(-1);
                n.bascula.setPosition(Pozitii.lansare);
                if (!shooterPreparado) {
                    n.applyVoltageCompensatedPIDF();
                    n.shooter.setVelocity(SHOOTER_VEL);
                    n.shooter2.setVelocity(SHOOTER_VEL);
                    n.unghiD.setPosition(pop.posUnghi);
                    actionTimer.resetTimer();
                    ShootingStare = 1;
                } else {
                    ShootingStare = 3;
                }
                track();
                break;

            case 1:
                track();
                if (actionTimer.getElapsedTimeSeconds() >= 0.35) {
                    actionTimer.resetTimer();
                    ShootingStare = 2;
                }
                break;

            case 2:
                track();
                if (actionTimer.getElapsedTimeSeconds() >= 1.0) {
                    ShootingStare = 3;
                }
                break;

            case 3:
                if (ShootSlot >= 0) {
                    ShootingStare = 4;
                } else {
                    flushSlot = 2;
                    flushRound = 0;
                    n.intake.setPower(1);
                    ShootingStare = 11;
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
                track();
                if (actionTimer.getElapsedTimeSeconds() >= 0.20) {
                    velocityCheckStart = System.currentTimeMillis();
                    ShootingStare = 6;
                }
                break;

            case 6:
                track();
                double v1 = Math.abs(n.shooter.getVelocity());
                double v2 = Math.abs(n.shooter2.getVelocity());
                double tol = SHOOTER_VEL * 0.03;
                if ((Math.abs(v1 - SHOOTER_VEL) < tol && Math.abs(v2 - SHOOTER_VEL) < tol)
                        || (System.currentTimeMillis() - velocityCheckStart) > 500) {
                    ShootingStare = 7;
                }
                break;

            case 7:
                actionTimer.resetTimer();
                ShootingStare = 8;
                break;

            case 8:
                if (actionTimer.getElapsedTimeSeconds() >= 0.13) {
                    ShootingStare = 9;
                }
                break;

            case 9:
                actionTimer.resetTimer();
                ShootingStare = 10;
                break;

            case 10:
                if (actionTimer.getElapsedTimeSeconds() >= 0.09) {
                    slotOcupat[ShootSlot] = false;
                    ShootSlot--;
                    ShootingStare = 3;
                }
                break;

            case 11:
                double flushTarget;
                if (flushSlot == 0) flushTarget = Pozitii.aruncare1;
                else if (flushSlot == 1) flushTarget = Pozitii.aruncare2;
                else flushTarget = Pozitii.aruncare3;
                n.sortare.setPosition(flushTarget);
                actionTimer.resetTimer();
                ShootingStare = 12;
                break;

            case 12:
                track();
                if (actionTimer.getElapsedTimeSeconds() >= 0.25) {
                    ShootingStare = 13;
                }
                break;

            case 13:
                actionTimer.resetTimer();
                ShootingStare = 14;
                break;

            case 14:
                if (actionTimer.getElapsedTimeSeconds() >= 0.13) {
                    ShootingStare = 15;
                }
                break;

            case 15:
                actionTimer.resetTimer();
                ShootingStare = 16;
                break;

            case 16:
                if (actionTimer.getElapsedTimeSeconds() >= 0.09) {
                    flushSlot--;
                    if (flushSlot >= 0) {
                        ShootingStare = 11;
                    } else {
                        actionTimer.resetTimer();
                        ShootingStare = 17;
                    }
                }
                break;

            case 17:
                if (actionTimer.getElapsedTimeSeconds() >= 0.3) {
                    double dist = n.distanta.getDistance(DistanceUnit.CM);
                    if (dist < 20 && flushRound < 2) {
                        flushRound++;
                        flushSlot = 2;
                        ShootingStare = 11;
                    } else {
                        ShootingStare = 18;
                    }
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
                    n.shooter.setVelocity(750);
                    n.shooter2.setVelocity(750);
                    ShootingStare = 20;
                }
                break;

            case 20:
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;
                TragereInProgres = false;
                shooterPreparado = false;
                ShootingStare = 0;
                break;
        }
    }

    private volatile boolean intakePornit = false;
    private volatile boolean stop = false;
    private Thread IntakeThread;

    private void Intake() {
        IntakeThread = new Thread(new Runnable() {
            private boolean ballBeingProcessed = false;

            @Override
            public void run() {
                while (!stop) {
                    try { Thread.sleep(10); } catch (InterruptedException e) { break; }
                    int loculete = getLoculete();
                    if (intakePornit && loculete < 3) {
                        n.intake.setPower(1);

                        double leDistanta = n.distanta.getDistance(DistanceUnit.CM);

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


    private void track() {
        n.tracks(follower, TARGET_X, TARGET_Y);
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
                track();
                if (!TragereInProgres) {
                    TragereInProgres = true;
                    ShootingStare = 0;
                }
                TragereLaPupitru();
                if (!TragereInProgres) {
                    actionTimer.resetTimer();
                    setPathState(23);
                }
                break;

            case 3:
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(collectare1,0.85,false);
                setPathState(4);
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.holdPoint(aluat1);
                    actionTimer.resetTimer();
                    setPathState(5);
                }
                break;

            case 5:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 0.9) {
                    intakePornit = false;
                    if (getLoculete() == 0) {
                        setPathState(24);
                    } else {
                        setPathState(6);
                    }
                }
                break;

            case 6:
                intakePornit = false;
                pregatireShooter();
                follower.followPath(trasUnu);
                setPathState(7);
                break;

            case 7:
                track();
                if (!follower.isBusy()) {
                    follower.holdPoint(tras1);
                    setPathState(8);
                }
                break;

            case 8:
                track();
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
                follower.followPath(collectare2,0.85,false);
                setPathState(10);
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.holdPoint(aluat2);
                    actionTimer.resetTimer();
                    setPathState(11);
                }
                break;

            case 11:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 1.5) {
                    intakePornit = false;
                    if (getLoculete() == 0) {
                        setPathState(27);
                    } else {
                        setPathState(12);
                    }
                }
                break;

            case 12:
                pregatireShooter();
                intakePornit = false;
                follower.followPath(trasDoi);
                setPathState(13);
                break;

            case 13:
                track();
                if (!follower.isBusy()) {
                    follower.holdPoint(tras2);
                    setPathState(14);
                }
                break;

            case 14:
                track();
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
                follower.followPath(collectare3,0.86,false);
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
                    if (getLoculete() == 0) {
                        setPathState(21);
                    } else {
                        setPathState(18);
                    }
                }
                break;

            case 18:
                pregatireShooter();
                intakePornit = false;
                follower.followPath(trasTrei);
                setPathState(19);
                break;

            case 19:
                track();
                if (!follower.isBusy()) {
                    follower.holdPoint(tras3);
                    setPathState(20);
                }
                break;

            case 20:
                track();
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
                follower.followPath(returnarea);
                setPathState(22);
                break;

            case 22:
                if (!follower.isBusy()) {
                    follower.holdPoint(returnToBase);
                    setPathState(-1);
                }
                break;

            case 23:
                if (actionTimer.getElapsedTimeSeconds() >= 0.5) {
                    setPathState(3);
                }
                break;

            case 24:
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;

                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(Miss1);
                setPathState(25);
                break;

            case 25:
                if (!follower.isBusy()) {
                    follower.holdPoint(aluat2);
                    actionTimer.resetTimer();
                    setPathState(26);
                }
                break;

            case 26:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 1.5) {
                    intakePornit = false;
                    if (getLoculete() == 0) {
                        setPathState(27);
                    } else {
                        setPathState(12);
                    }
                }
                break;

            case 27:
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(Miss2);
                setPathState(28);
                break;

            case 28:
                if (!follower.isBusy()) {
                    follower.holdPoint(aluat3);
                    actionTimer.resetTimer();
                    setPathState(29);
                }
                break;

            case 29:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 1.5) {
                    intakePornit = false;
                    if (getLoculete() == 0) {
                        setPathState(21);
                    } else {
                        setPathState(18);
                    }
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
        follower.update();

        telemetry.addData("servo", n.sortare.getPosition());
        telemetry.addData("slot",slotOcupat);
        telemetry.addData("LL tx", "%.2f°", n.getLimelightTx());
        telemetry.addData("Tag vizibil", n.isTagVisible() ? "DA" : "NU");
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
        buildPaths();
        follower.setStartingPose(startPose);

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
    }

    @Override
    public void stop() {
        stop = true;

        Pose currentPose = follower.getPose();
        RobotPozitie.X = currentPose.getX();
        RobotPozitie.Y = currentPose.getY();
        RobotPozitie.heading = currentPose.getHeading();

        n.shooter.setVelocity(0);
        n.shooter2.setVelocity(0);
        n.intake.setPower(0);
        n.scula.setPower(0);
        n.bascula.setPosition(0.5807);
    }
}