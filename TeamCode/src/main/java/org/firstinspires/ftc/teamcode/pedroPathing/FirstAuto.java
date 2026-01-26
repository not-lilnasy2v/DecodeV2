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
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private static final double TARGET_X = 0;
    private static final double TARGET_Y = 154;
    private static final double SHOOTER_TARGET_VELOCITY = 1550;
    private static final double VELOCITY_TOLERANCE = 0.05;

    private final Pose startPose = new Pose(24.503496503496507, 128.8951048951049, Math.toRadians(142));
    private final Pose tragere1 = new Pose(63.72027972027973, 95.74825174825179, Math.toRadians(180));
    private final Pose aduna1 = new Pose(59.3076923076923, 89.93706293706293, Math.toRadians(180));
    private final Pose aluat1 = new Pose(23.55244755244755, 89.05804195804195, Math.toRadians(180));
    private final Pose ARatat1 = new Pose(71.87014034916132,68.10259670399529, Math.toRadians(180));
    private final Pose tras1 = new Pose(62.72027972027973, 95.74825174825179, Math.toRadians(180));
    private final Pose aduna2 = new Pose(70.97403296004696, 62.21209786223014, Math.toRadians(180));
    private final Pose aluat2 = new Pose(17.69230769230769, 62.75055940069167, Math.toRadians(180));
    private final Pose ARatat2 = new Pose(71.54895104895104,48.56293706293705, Math.toRadians(180));
    private final Pose curburaMiti  = new Pose(47.99352046554845,70.20678735353181,Math.toRadians(180));
    private final Pose tras2 = new Pose(62.72027972027973, 95.74825174825179, Math.toRadians(180));
    private final Pose aduna3 = new Pose(68.47552447552445, 40.06993006993007, Math.toRadians(180));
    private final Pose aluat3 = new Pose(16.210489510489513, 40.03076923076926, Math.toRadians(180));
    private final Pose tras3 = new Pose(60.72027972027973, 95.74825174825179, Math.toRadians(180));
    private final Pose returnToBase = new Pose(25.790209790209786,83.66433566433567,Math.toRadians(180));



    private Path scorePreload;
    private PathChain collectare1, Miss1, trasUnu, collectare2, Miss2, trasDoi, collectare3, trasTrei,returnarea;

    private boolean TragereInProgres = false;
    private boolean shooterPreparado = false;
    private int ballsToShoot = 0;
    private int ShootingStare = 0;

    private volatile boolean[] slotOcupat = new boolean[3];


    private int getLoculete() {
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
                .setTimeoutConstraint(100)
                .build();
        returnarea=  follower.pathBuilder()
                .addPath(new BezierLine(tras3,returnToBase))
                .setLinearHeadingInterpolation(tras3.getHeading(),returnToBase.getHeading())
                .build();
    }

    private void pregatireShooter() {
        if (!shooterPreparado) {
            PIDFCoefficients pid = new PIDFCoefficients(n.SkP, n.SkI, n.SkD, n.SkF );
            n.shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
            n.shooter2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
            n.shooter.setVelocity(1550);
            n.shooter2.setVelocity(1550);
            n.unghiS.setPosition(pop.posUnghi);
            n.unghiD.setPosition(pop.posUnghi);
            shooterPreparado = true;
        }
    }

    private int currentShootSlot = 2;

    private void TragereLaPupitru() {
        switch (ShootingStare) {
            case 0:
                ballsToShoot = getLoculete();
                if (!shooterPreparado) {
                    PIDFCoefficients pid = new PIDFCoefficients(n.SkP, n.SkI, n.SkD, n.SkF);
                    n.shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
                    n.shooter2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
                    n.shooter.setVelocity(1550);
                    n.shooter2.setVelocity(1550);
                    n.unghiS.setPosition(pop.posUnghi);
                    n.unghiD.setPosition(pop.posUnghi);
                    actionTimer.resetTimer();
                    ShootingStare = 1;
                } else {
                    currentShootSlot = 2;
                    actionTimer.resetTimer();
                    ShootingStare = 13;
                }
                track();
                break;

            case 1:
                track();
                if (actionTimer.getElapsedTimeSeconds() >= 0.35) {
                    currentShootSlot = 2;
                    actionTimer.resetTimer();
                    ShootingStare = 13;
                }
                break;

            case 13:
                track();
                if (actionTimer.getElapsedTimeSeconds() >= 0.45) {
                    ShootingStare = 2;
                }
                break;

            case 2:
                if (ballsToShoot > 0 && currentShootSlot >= 0) {
                    ShootingStare = 3;
                } else {
                    ShootingStare = 10;
                }
                break;

            case 3:
                double target;
                if (currentShootSlot == 0) {
                    target = Pozitii.aruncare1;
                } else if (currentShootSlot == 1) {
                    target = Pozitii.aruncare2;
                } else {
                    target = Pozitii.aruncare3;
                }
                n.sortare.setPosition(target);
                actionTimer.resetTimer();
                ShootingStare = 4;
                break;

            case 4:
                track();
                if (actionTimer.getElapsedTimeSeconds() >= 0.40) {
                    ShootingStare = 5;
                }
                break;

            case 5:
                n.Saruncare.setPosition(Pozitii.lansare);
                actionTimer.resetTimer();
                ShootingStare = 6;
                break;

            case 6:
                if (actionTimer.getElapsedTimeSeconds() >= 0.20) {
                    ShootingStare = 7;
                }
                break;

            case 7:
                n.Saruncare.setPosition(Pozitii.coborare);
                actionTimer.resetTimer();
                ShootingStare = 8;
                break;

            case 8:
                if (actionTimer.getElapsedTimeSeconds() >= 0.20) {
                    slotOcupat[currentShootSlot] = false;
                    currentShootSlot--;
                    ballsToShoot--;
                    ShootingStare = 2;
                }
                break;

            case 10:
                n.sortare.setPosition(Pozitii.luarea1);
                actionTimer.resetTimer();
                ShootingStare = 11;
                break;

            case 11:
                if (actionTimer.getElapsedTimeSeconds() >= 0.15) {
                    n.shooter.setVelocity(750);
                    n.shooter2.setVelocity(750);
                    ShootingStare = 12;
                }
                break;

            case 12:
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;
                TragereInProgres = false;
                shooterPreparado = false;
                ShootingStare = 0;
                break;
        }
    }

    private volatile boolean ballDetected = false;
    private volatile boolean intakePornit = false;
    private volatile boolean stop = false;
    private Thread IntakeThread;

    private void Intake() {
        IntakeThread = new Thread(new Runnable() {
            private boolean ballBeingProcessed = false;

            @Override
            public void run() {
                while (!stop) {
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


private void untrack(){
        n.turelaD.setPosition(0.23);
        n.turelaS.setPosition(0.23);
}
    private void track() {
//        n.tracks(follower, TARGET_X, TARGET_Y);
        n.turelaS.setPosition(0.21);
        n.turelaD.setPosition(0.21);
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
                    setPathState(25);
                }
                break;

            case 25:
                if (actionTimer.getElapsedTimeSeconds() >= 0.5) {
                    setPathState(3);
                }
                break;

            case 3:
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(collectare1,0.75,false);
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
                        setPathState(50);
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
                follower.followPath(collectare2);
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
                        setPathState(60);
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
                untrack();
                if (!follower.isBusy()) {
                    follower.holdPoint(tras2);
                    setPathState(14);
                }
                break;

            case 14:
                untrack();
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

            case 50:
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;

                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(Miss1);
                setPathState(52);
                break;

            case 52:
                if (!follower.isBusy()) {
                    follower.holdPoint(aluat2);
                    actionTimer.resetTimer();
                    setPathState(53);
                }
                break;

            case 53:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 1.5) {
                    intakePornit = false;
                    if (getLoculete() == 0) {
                        setPathState(60);
                    } else {
                        setPathState(12);
                    }
                }
                break;

            case 60:
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(Miss2);
                setPathState(62);
                break;

            case 62:
                if (!follower.isBusy()) {
                    follower.holdPoint(aluat3);
                    actionTimer.resetTimer();
                    setPathState(63);
                }
                break;

            case 63:
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
    }
}