package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Pozitii;
import org.firstinspires.ftc.teamcode.pop;
import org.firstinspires.ftc.teamcode.sistemeAuto;

@Autonomous
public class RosuAuto extends OpMode {
    sistemeAuto n = new sistemeAuto();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;


    private final Pose startPose = new Pose(124.53146853146853, 126.37762237762237, Math.toRadians(42));
    private final Pose tragere1 = new Pose(76.86713286713287, 76.6993006993007, Math.toRadians(42));
    private final Pose aduna1 = new Pose(84.58741258741259, 71.83216783216783);
    private final Pose aluat1 = new Pose(117.81818181818183, 88.61538461538461, Math.toRadians(0));
    private final Pose tras2 = new Pose(76.53146853146853, 76.36363636363636, Math.toRadians(42));


    private Path scorePreload;
    private PathChain luat1, tras1;

    private boolean isShootingInProgress = false;
    private int currentBallShot = 0;
    private int shootingSubState = 0;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, tragere1));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), tragere1.getHeading());

        luat1 = follower.pathBuilder()
                .addPath(new BezierLine(aduna1, aluat1))
                .setLinearHeadingInterpolation(tragere1.getHeading(), aluat1.getHeading())
                .build();

        tras1 = follower.pathBuilder()
                .addPath(new BezierLine(aluat1, tras2))
                .setLinearHeadingInterpolation(aluat1.getHeading(), tras2.getHeading())
                .build();
    }

    private void shootingStateMachine() {
        switch (shootingSubState) {
            case 0:
                telemetry.addLine("Starting shooter motor...");
                PIDFCoefficients pid = new PIDFCoefficients(n.SkP, n.SkI, n.SkD, n.SkF);
                n.shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
                n.shooter.setVelocity(1750);
                n.tracking();
                actionTimer.resetTimer();
                shootingSubState = 1;
                break;

            case 1:
                if (actionTimer.getElapsedTimeSeconds() >= 0.5) {
                    currentBallShot = 0;
                    shootingSubState = 2;
                }
                break;

            case 2:
                if (n.loculete > 0) {
                    shootingSubState = 3;
                } else {
                    shootingSubState = 10;
                }
                break;

            case 3:
                int shootPos = -1;
                if (n.loculete >= 3) {
                    shootPos = 2;
                } else if (n.loculete >= 2) {
                    shootPos = 1;
                } else if (n.loculete >= 1) {
                    shootPos = 0;
                }

                if (shootPos == 0) {
                    n.sortare.setPosition(Pozitii.aruncare1);
                } else if (shootPos == 1) {
                    n.sortare.setPosition(Pozitii.aruncare2);
                } else if (shootPos == 2) {
                    n.sortare.setPosition(Pozitii.aruncare3);
                }

                actionTimer.resetTimer();
                shootingSubState = 4;
                break;

            case 4:
                if (actionTimer.getElapsedTimeSeconds() >= 0.95) {
                    shootingSubState = 5;
                }
                break;

            case 5:
                n.Saruncare.setPosition(Pozitii.lansare);
                actionTimer.resetTimer();
                shootingSubState = 6;
                break;

            case 6:
                if (actionTimer.getElapsedTimeSeconds() >= 0.15) {
                    shootingSubState = 7;
                }
                break;

            case 7:
                n.Saruncare.setPosition(Pozitii.coborare);
                actionTimer.resetTimer();
                shootingSubState = 8;
                break;

            case 8:
                if (actionTimer.getElapsedTimeSeconds() >= 0.15) {
                    n.loculete--;
                    currentBallShot++;
                    shootingSubState = 2;
                }
                break;

            case 10:
                n.sortare.setPosition(Pozitii.luarea1);
                actionTimer.resetTimer();
                shootingSubState = 11;
                break;

            case 11:
                if (actionTimer.getElapsedTimeSeconds() >= 0.3) {
                    n.shooter.setVelocity(0);
                    shootingSubState = 12;
                }
                break;

            case 12:
                isShootingInProgress = false;
                shootingSubState = 0;
                break;
        }
    }

    private int collectionSubState = 0;
    private boolean isCollecting = false;

    private void collectionStateMachine() {
        switch (collectionSubState) {
            case 0:
                n.sortare.setPosition(Pozitii.luarea1);
                n.unghiS.setPosition(pop.posUnghi);
                n.unghiD.setPosition(pop.posUnghi);
                actionTimer.resetTimer();
                collectionSubState = 1;
                break;

            case 1:
                if (actionTimer.getElapsedTimeSeconds() >= 0.5) {
                    n.intake.setPower(1.0);
                    actionTimer.resetTimer();
                    collectionSubState = 2;
                }
                break;

            case 2:
                if (actionTimer.getElapsedTimeSeconds() >= 3.0) {
                    n.intake.setPower(0);
                    n.loculete = 3;
                    isCollecting = false;
                    collectionSubState = 0;
                }
                break;
        }
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
                if (!isShootingInProgress) {
                    isShootingInProgress = true;
                    shootingSubState = 0;
                }

                shootingStateMachine();

                if (!isShootingInProgress) {
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
                follower.followPath(luat1);
                setPathState(4);
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.holdPoint(aluat1);
                    setPathState(5);
                }
                break;

            case 5:
                if (!isCollecting) {
                    isCollecting = true;
                    collectionSubState = 0;
                }

                collectionStateMachine();

                if (!isCollecting) {
                    setPathState(6);
                }
                break;

            case 6:
                follower.followPath(tras1);
                setPathState(7);
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.holdPoint(tras2);
                    setPathState(8);
                }
                break;

            case 8:
                if (!isShootingInProgress) {
                    isShootingInProgress = true;
                    shootingSubState = 0;
                }

                shootingStateMachine();

                if (!isShootingInProgress) {
                    setPathState(9);
                }
                break;

            case 9:
                setPathState(-1);
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
        isShootingInProgress = false;

    }

    @Override
    public void stop() {
        n.shooter.setVelocity(0);
        n.intake.setPower(0);

    }
}