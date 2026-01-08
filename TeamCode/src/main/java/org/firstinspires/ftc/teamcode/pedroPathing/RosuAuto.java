package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Pozitii;
import org.firstinspires.ftc.teamcode.RobotPozitie;
import org.firstinspires.ftc.teamcode.pop;
import org.firstinspires.ftc.teamcode.sistemeAuto;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.logging.XMLFormatter;

@Autonomous
public class RosuAuto extends OpMode {
    sistemeAuto n = new sistemeAuto();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Limelight3A limelight3A;

    private static final double TARGET_X = 144;
    private static final double TARGET_Y = 144;

    private final Pose startPose = new Pose(120.35855054036871, 127.40143772311603, Math.toRadians(37));
    private final Pose tragere1 = new Pose(89.0150129590689, 91.19448383784047, Math.toRadians(0));
//    private final Pose aduna1 = new Pose(100.54120005868258, 88.14210963861314,Math.toRadians(0));
    private final Pose aluat1 = new Pose(119.51322803071056, 90.17702577143136, Math.toRadians(0));
    private final Pose ARatat1 = new Pose(80.55254387312878, 67.32287653839902);
    private final Pose tras1 = new Pose(88.84737639982394, 88.84737639982394, Math.toRadians(0));
    private final Pose aduna2 = new Pose(82.77822876424275, 78.35111741405447);
    private final Pose aluat2 = new Pose(126.35141082693531,60.36681500317866, Math.toRadians(0));
    private final Pose ARatat2 = new Pose(78.26223776223776, 48.31818181818181);
    private final Pose tras2 = new Pose(88.84737639982394, 91.02684727859551, Math.toRadians(0));
    private final Pose aduna3 = new Pose(84.44525404665265, 41.74600224949872);
    private final Pose aluat3 = new Pose(124.94850114920044, 39.39478703115066 , Math.toRadians(0));
    private final Pose tras3 = new Pose(88.72727272727273,91.1958041958042,Math.toRadians(0));


    private Path scorePreload;
    private PathChain luat1, Miss1, Recorectat1, trasUnu, ceva2, luat2, Miss2, Recorectat2, trasDoi, ceva3, luat3, trasTrei;

    private boolean TragereInProgres = false;
    private int ballsShot = 0;
    private int ShootingStare = 0;
    private boolean shooterPreparado = false;

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
//        ceva1 = follower.pathBuilder()
//                .addPath(new BezierCurve( tragere1, aduna1))
//                .setLinearHeadingInterpolation(tragere1.getHeading(),aduna1.getHeading())
//                .build();
        luat1 = follower.pathBuilder()
                .addPath(new BezierLine(tragere1, aluat1))
                .setLinearHeadingInterpolation(tragere1.getHeading(), aluat1.getHeading())
                .build();
        Miss1 = follower.pathBuilder()
                .addPath(new BezierCurve(aluat1,ARatat1))
                .setLinearHeadingInterpolation(aluat1.getHeading(),ARatat1.getHeading())
                .build();
        Recorectat1 = follower.pathBuilder()
                .addPath(new BezierLine(ARatat1, aluat2))
                .setLinearHeadingInterpolation(ARatat1.getHeading(),aluat2.getHeading())
                .build();
        trasUnu = follower.pathBuilder()
                .addPath(new BezierLine(aluat1, tras1))
                .setLinearHeadingInterpolation(aluat1.getHeading(), tras1.getHeading())
                .build();
        ceva2 = follower.pathBuilder()
                .addPath(new BezierCurve(tras1,aduna2))
                .setLinearHeadingInterpolation(tras1.getHeading(),aduna2.getHeading())
                .build();
        luat2 = follower.pathBuilder()
                .addPath(new BezierLine(aduna2,aluat2))
                .setLinearHeadingInterpolation(tras1.getHeading(),aluat2.getHeading())
                .build();
        Miss2 = follower.pathBuilder()
                .addPath(new BezierCurve(aluat2, ARatat2))
                .setLinearHeadingInterpolation(aluat2.getHeading(), ARatat2.getHeading())
                .build();
        Recorectat2 = follower.pathBuilder()
                .addPath(new BezierLine(ARatat2, aluat3))
                .setLinearHeadingInterpolation(ARatat2.getHeading(), aluat3.getHeading())
                .build();
        trasDoi = follower.pathBuilder()
                .addPath(new BezierLine(aluat2,tras2))
                .setLinearHeadingInterpolation(aluat2.getHeading(),tras2.getHeading())
                .build();
        ceva3 = follower.pathBuilder()
                .addPath(new BezierCurve(tras2,aduna3))
                .setLinearHeadingInterpolation(tras2.getHeading(),aduna3.getHeading())
                .build();
        luat3 = follower.pathBuilder()
                .addPath(new BezierLine(aduna3, aluat3))
                .setLinearHeadingInterpolation(tras2.getHeading(), aluat3.getHeading())
                .build();
        trasTrei = follower.pathBuilder()
                .addPath(new BezierLine(aluat3, tras3))
                .setLinearHeadingInterpolation(aluat3.getHeading(), tras3.getHeading())
                .build();
    }

    private void pregatireShooter() {
        if (!shooterPreparado) {
            PIDFCoefficients pid = new PIDFCoefficients(n.SkP, n.SkI, n.SkD, n.SkF);
            n.shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
            n.shooter.setVelocity(1500);
            n.unghiS.setPosition(pop.posUnghi);
            n.unghiD.setPosition(pop.posUnghi);
            shooterPreparado = true;
        }
    }

    private int currentShootSlot = 2;

    private void TragereLaPupitru() {
        switch (ShootingStare) {
            case 0:
                if (!shooterPreparado) {
                    PIDFCoefficients pid = new PIDFCoefficients(n.SkP, n.SkI, n.SkD, n.SkF);
                    n.shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
                    n.shooter.setVelocity(1500);
                    n.unghiS.setPosition(pop.posUnghi);
                    n.unghiD.setPosition(pop.posUnghi);
                    actionTimer.resetTimer();
                    ShootingStare = 1;
                } else {
                    ballsShot = 0;
                    currentShootSlot = 2;
                    ShootingStare = 2;
                }
                trackTargetWithOdometry();
                break;

            case 1:
                if (actionTimer.getElapsedTimeSeconds() >= 0.5) {
                    ballsShot = 0;
                    currentShootSlot = 2;
                    ShootingStare = 2;
                }
                break;

            case 2:
                while (currentShootSlot >= 0 && !slotOcupat[currentShootSlot]) {
                    currentShootSlot--;
                }

                if (currentShootSlot >= 0 && slotOcupat[currentShootSlot]) {
                    ShootingStare = 3;
                } else {
                    ShootingStare = 10;
                }
                break;

            case 3:
                if (currentShootSlot == 0) {
                    n.sortare.setPosition(Pozitii.aruncare1);
                } else if (currentShootSlot == 1) {
                    n.sortare.setPosition(Pozitii.aruncare2);
                } else if (currentShootSlot == 2) {
                    n.sortare.setPosition(Pozitii.aruncare3);
                }
                actionTimer.resetTimer();
                ShootingStare = 4;
                break;

            case 4:
                if (actionTimer.getElapsedTimeSeconds() >= 0.95) {
                    ShootingStare = 5;
                }
                break;

            case 5:
                n.Saruncare.setPosition(Pozitii.lansare);
                actionTimer.resetTimer();
                ShootingStare = 6;
                break;

            case 6:
                if (actionTimer.getElapsedTimeSeconds() >= 0.15) {
                    ShootingStare = 7;
                }
                break;

            case 7:
                n.Saruncare.setPosition(Pozitii.coborare);
                actionTimer.resetTimer();
                ShootingStare = 8;
                break;

            case 8:
                if (actionTimer.getElapsedTimeSeconds() >= 0.15) {
                    slotOcupat[currentShootSlot] = false;
                    ballsShot++;
                    currentShootSlot--;
                    ShootingStare = 2;
                }
                break;

            case 10:
                n.sortare.setPosition(Pozitii.luarea1);
                actionTimer.resetTimer();
                ShootingStare = 11;
                break;

            case 11:
                if (actionTimer.getElapsedTimeSeconds() >= 0.3) {
                    n.shooter.setVelocity(750);
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

    private volatile boolean intakePornit = false;
    private volatile boolean stop = false;
    private volatile boolean ballDetected = false;
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
                                n.kdf(50);
                                if (!slotOcupat[1]) {
                                    n.sortare.setPosition(Pozitii.luarea2);
                                } else if (!slotOcupat[2]) {
                                    n.sortare.setPosition(Pozitii.luarea3);
                                }

                            } else if (Math.abs(servoPos - Pozitii.luarea2) < 0.1 && !slotOcupat[1]) {
                                slotOcupat[1] = true;
                                n.kdf(50);
                                if (!slotOcupat[2]) {
                                    n.sortare.setPosition(Pozitii.luarea3);
                                } else if (!slotOcupat[0]) {
                                    n.sortare.setPosition(Pozitii.luarea1);
                                }

                            } else if (Math.abs(servoPos - Pozitii.luarea3) < 0.1 && !slotOcupat[2]) {
                                slotOcupat[2] = true;
                                n.kdf(50);
                                if (!slotOcupat[0]) {
                                    n.sortare.setPosition(Pozitii.luarea1);
                                } else if (!slotOcupat[1]) {
                                    n.sortare.setPosition(Pozitii.luarea2);
                                }
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


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.holdPoint(tragere1);
                    setPathState(2);
                }

                break;

            case 2:
                trackTargetWithOdometry();
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
                    // Skip ceva1, go directly to intake
                    slotOcupat[0] = false;
                    slotOcupat[1] = false;
                    slotOcupat[2] = false;
                    n.sortare.setPosition(Pozitii.luarea1);
                    intakePornit = true;
                    follower.followPath(luat1);
                    setPathState(4);
                }
                break;

            case 30:
                if (!follower.isBusy()) {
                    slotOcupat[0] = false;
                    slotOcupat[1] = false;
                    slotOcupat[2] = false;
                    n.sortare.setPosition(Pozitii.luarea1);
                    intakePornit = true;
                    follower.followPath(luat1);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.holdPoint(aluat1);
                    actionTimer.resetTimer();
                    setPathState(5);
                }
                break;

            case 5:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 0.5) {
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
                trackTargetWithOdometry();
                if (!follower.isBusy()) {
                    follower.holdPoint(tras1);
                    setPathState(8);
                }
                break;

            case 8:
                trackTargetWithOdometry();
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
                follower.followPath(ceva2);
                setPathState(90);
                break;

            case 90:
                if (!follower.isBusy()) {
                    slotOcupat[0] = false;
                    slotOcupat[1] = false;
                    slotOcupat[2] = false;
                    n.sortare.setPosition(Pozitii.luarea1);
                    intakePornit = true;
                    follower.followPath(luat2);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.holdPoint(aluat2);
                    actionTimer.resetTimer();
                    setPathState(11);
                }
                break;

            case 11:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 0.5) {
                    intakePornit = false;
                    if (getLoculete() == 0) {
                        setPathState(60);
                    } else {
                        setPathState(12);
                    }
                }
                break;

            case 12:
                intakePornit = false;
                pregatireShooter();
                follower.followPath(trasDoi);
                setPathState(13);
                break;

            case 13:
                trackTargetWithOdometry();
                if (!follower.isBusy()) {
                    follower.holdPoint(tras2);
                    setPathState(14);
                }
                break;

            case 14:
                trackTargetWithOdometry();

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
                follower.followPath(ceva3);
                setPathState(150);
                break;

            case 150:
                if (!follower.isBusy()) {
                    slotOcupat[0] = false;
                    slotOcupat[1] = false;
                    slotOcupat[2] = false;
                    n.sortare.setPosition(Pozitii.luarea1);
                    intakePornit = true;
                    follower.followPath(luat3);
                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    follower.holdPoint(aluat3);
                    actionTimer.resetTimer();
                    setPathState(17);
                }
                break;

            case 17:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 0.5) {
                    intakePornit = false;
                    if (getLoculete() == 0) {
                        setPathState(21);
                    } else {
                        setPathState(18);
                    }
                }
                break;

            case 18:
                intakePornit = false;
                pregatireShooter();
                follower.followPath(trasTrei);
                setPathState(19);
                break;

            case 19:
                trackTargetWithOdometry();
                if (!follower.isBusy()) {
                    follower.holdPoint(tras3);
                    setPathState(20);
                }
                break;

            case 20:
                trackTargetWithOdometry();

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
                setPathState(-1);
                break;

            case 50:
                follower.followPath(Miss1);
                setPathState(51);
                break;

            case 51:
                if (!follower.isBusy()) {
                    slotOcupat[0] = false;
                    slotOcupat[1] = false;
                    slotOcupat[2] = false;
                    n.sortare.setPosition(Pozitii.luarea1);
                    intakePornit = true;
                    follower.followPath(Recorectat1);
                    setPathState(52);
                }
                break;

            case 52:
                if (!follower.isBusy()) {
                    follower.holdPoint(aluat2);
                    actionTimer.resetTimer();
                    setPathState(53);
                }
                break;

            case 53:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 0.5) {
                    intakePornit = false;
                    if (getLoculete() == 0) {
                        setPathState(60);
                    } else {
                        setPathState(12);
                    }
                }
                break;
            case 60:
                follower.followPath(Miss2);
                setPathState(61);
                break;

            case 61:
                if (!follower.isBusy()) {
                    slotOcupat[0] = false;
                    slotOcupat[1] = false;
                    slotOcupat[2] = false;
                    n.sortare.setPosition(Pozitii.luarea1);
                    intakePornit = true;
                    follower.followPath(Recorectat2);
                    setPathState(62);
                }
                break;

            case 62:
                if (!follower.isBusy()) {
                    follower.holdPoint(aluat3);
                    actionTimer.resetTimer();
                    setPathState(63);
                }
                break;

            case 63:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 0.5) {
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


    private void trackTargetWithOdometry() {
        n.trackTargetWithOdometry(follower, TARGET_X, TARGET_Y);
    }



    @Override
    public void loop() {
        follower.update();
        telemetry.addData("slots",slotOcupat[2]);
        telemetry.addData("slot1",slotOcupat[1]);
        telemetry.addData("slot2",slotOcupat[2]);
        telemetry.update();
        autonomousPathUpdate();

    }

    @Override
    public void init() {
        n.initsisteme(hardwareMap);

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(1);
        limelight3A.start();

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
        RobotPozitie.turelaPosition = n.turela.getCurrentPosition();
        RobotPozitie.idTag = n.idTag;
        RobotPozitie.X = currentPose.getX();
        RobotPozitie.Y = currentPose.getY();
        RobotPozitie.heading = currentPose.getHeading();

        n.shooter.setVelocity(0);
        n.intake.setPower(0);
    }
}