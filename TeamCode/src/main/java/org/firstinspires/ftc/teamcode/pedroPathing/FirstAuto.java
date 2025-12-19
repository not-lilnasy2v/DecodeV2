package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
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

@Autonomous
public class FirstAuto extends OpMode {
    sistemeAuto n = new sistemeAuto();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private static final double TARGET_X = 0;
    private static final double TARGET_Y = 144;
    private static final double TICKS_PER_DEGREE = 1;
    private static final double MAX_TURRET_ANGLE = 90;
    private static final double MIN_TURRET_ANGLE = -90;
    private static final double TURRET_POWER = 1;

    private final Pose startPose = new Pose(24.503496503496507, 128.8951048951049, Math.toRadians(142));
    private final Pose tragere1 = new Pose(55.552447552447546, 96.15384615384616, Math.toRadians(140));
    private final Pose aduna1 = new Pose(68.3076923076923, 86.93706293706293);
    private final Pose aluat1 = new Pose(27.55244755244755, 89.95804195804195, Math.toRadians(180));
    private final Pose tras1 = new Pose(55.552447552447546, 94.15384615384616, Math.toRadians(140));
    private final Pose aduna2 = new Pose(72.67132867132867, 65.95804195804196);
    private final Pose aluat2 = new Pose(22.034965034965033, 63.77622377622377, Math.toRadians(180));
    private final Pose tras2 = new Pose(66.12587412587412, 90.62937062937063, Math.toRadians(140));

    private final Pose aduna3 = new Pose(65.95804195804196, 43.8041958041958);
    private final Pose aluat3 = new Pose(23.363636363636363, 40.78321678321679, Math.toRadians(180));
    private final Pose tras3 = new Pose(66.12587412587412, 85.55944055944055, Math.toRadians(140));



    private Path scorePreload;
    private PathChain luat1, trasUnu,luat2,trasDoi,luat3,trasTrei;

    private boolean tragereInrogres = false;
    private int s_aTras = 0;
    private int stareaShooter = 0;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, tragere1));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), tragere1.getHeading());

        luat1 = follower.pathBuilder()
                .addPath(new BezierLine(aduna1, aluat1))
                .setLinearHeadingInterpolation(tragere1.getHeading(), aluat1.getHeading())
                .build();

        trasUnu = follower.pathBuilder()
                .addPath(new BezierLine(aluat1, tras1))
                .setLinearHeadingInterpolation(aluat1.getHeading(), tras1.getHeading())
                .build();
        luat2 = follower.pathBuilder()
                .addPath(new BezierLine(aduna2, aluat2))
                .setLinearHeadingInterpolation(tras1.getHeading(),aluat2.getHeading())
                .build();
        trasDoi = follower.pathBuilder()
                .addPath(new BezierLine(aluat2,tras2))
                .setLinearHeadingInterpolation(aluat2.getHeading(),tras2.getHeading())
                .build();
        luat3 = follower.pathBuilder()
                .addPath(new BezierLine(aduna3,aluat3))
                .setLinearHeadingInterpolation(tras2.getHeading(), aluat3.getHeading())
                .build();
        trasTrei = follower.pathBuilder()
                .addPath(new BezierLine(aluat3,tras3))
                .setLinearHeadingInterpolation(aluat3.getHeading(),tras3.getHeading())
                .build();

    }

    private void TragereLaPupitru() {
        switch (stareaShooter) {
            case 0:
                PIDFCoefficients pid = new PIDFCoefficients(n.SkP, n.SkI, n.SkD, n.SkF);
                n.shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
                n.shooter.setVelocity(1500);
                n.unghiS.setPosition(pop.posUnghi);
                n.unghiD.setPosition(pop.posUnghi);
                trackTargetWithOdometry();
                actionTimer.resetTimer();
                stareaShooter = 1;
                break;

            case 1:
                if (actionTimer.getElapsedTimeSeconds() >= 0.5) {
                    s_aTras = 0;
                    stareaShooter = 2;
                }
                break;

            case 2:
                if (n.loculete > 3) n.loculete = 3;
                if (n.loculete < 0) n.loculete = 0;

                if (n.loculete > 0) {
                    stareaShooter = 3;
                } else {
                    stareaShooter = 10;
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
                stareaShooter = 4;
                break;

            case 4:
                if (actionTimer.getElapsedTimeSeconds() >= 0.95) {
                    stareaShooter = 5;
                }
                break;

            case 5:
                n.Saruncare.setPosition(Pozitii.lansare);
                actionTimer.resetTimer();
                stareaShooter = 6;
                break;

            case 6:
                if (actionTimer.getElapsedTimeSeconds() >= 0.15) {
                    stareaShooter = 7;
                }
                break;

            case 7: 
                n.Saruncare.setPosition(Pozitii.coborare);
                actionTimer.resetTimer();
                stareaShooter = 8;
                break;

            case 8:
                if (actionTimer.getElapsedTimeSeconds() >= 0.15) {
                    n.loculete--;
                    s_aTras++;
                    stareaShooter = 2;
                }
                break;

            case 10:
                n.sortare.setPosition(Pozitii.luarea1);
                actionTimer.resetTimer();
                stareaShooter = 11;
                break;

            case 11:
                if (actionTimer.getElapsedTimeSeconds() >= 0.3) {
                    n.shooter.setVelocity(0);
                    stareaShooter = 12;
                }
                break;

            case 12:
                n.loculete = 0;
                tragereInrogres = false;
                stareaShooter = 0;
                break;
        }
    }

    private volatile boolean intakePornit = false;
    private volatile boolean stopThread = false;
    private Thread IntakeThread;

    private void Intake() {
        IntakeThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!stopThread) {
                    if (intakePornit && n.loculete < 3) {
                        n.intake.setPower(1);

                        double imata = n.distanta.getDistance(DistanceUnit.CM);

                        if (imata < 20) {
                            if (n.loculete == 0) {
                                n.kdf(150);
                                n.loculete = 1;
                                n.sortare.setPosition(Pozitii.luarea2);
                            } else if (n.loculete == 1) {
                                n.kdf(250);
                                n.loculete = 2;
                                n.sortare.setPosition(Pozitii.luarea3);
                            } else if (n.loculete == 2) {
                                n.kdf(350);
                                n.loculete = 3;
                            }
                        }
                    } else if (!intakePornit) {
                        n.intake.setPower(0);
                    }

                    try {
                        Thread.sleep(50);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            }
        });
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private void setTurretPosition(double angleDegrees) {
        int targetTicks = (int) (angleDegrees * TICKS_PER_DEGREE);
        n.turela.setTargetPosition(-targetTicks);
        n.turela.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        n.turela.setPower(TURRET_POWER);
    }

    private void trackTargetWithOdometry() {
        Pose currentPose = follower.getPose();
        double robotX = currentPose.getX();
        double robotY = currentPose.getY();
        double robotHeading = currentPose.getHeading();

        double dx = TARGET_X - robotX;
        double dy = TARGET_Y - robotY;
        double angleToTarget = Math.atan2(dy, dx);

        double turretAngleRad = angleToTarget - robotHeading;
        turretAngleRad = normalizeAngle(turretAngleRad);

        double turretAngleDeg = Math.toDegrees(turretAngleRad);
        turretAngleDeg = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, turretAngleDeg));

        setTurretPosition(turretAngleDeg);
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

                if (!tragereInrogres) {
                    tragereInrogres = true;
                    stareaShooter = 0;
                }

                TragereLaPupitru();

                if (!tragereInrogres) {
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
                n.loculete = 0;
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(luat1);
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
                if (n.loculete >= 3 || actionTimer.getElapsedTimeSeconds() >= 5.0) {
                    intakePornit = false;
                    setPathState(6);
                }
                break;

            case 6:
                intakePornit = false;
                follower.followPath(trasUnu);
                setPathState(7);
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.holdPoint(tras1);
                    setPathState(8);
                }
                break;

            case 8:
                trackTargetWithOdometry();

                if (!tragereInrogres) {
                    tragereInrogres = true;
                    stareaShooter = 0;
                }

                TragereLaPupitru();

                if (!tragereInrogres) {
                    setPathState(9);
                }
                break;

            case 9:
                n.loculete = 0;
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(luat2);
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
                if (n.loculete >= 3 || actionTimer.getElapsedTimeSeconds() >= 5.0) {
                    intakePornit = false;
                    setPathState(12);
                }
                break;

            case 12:
                intakePornit = false;
                follower.followPath(trasDoi);
                setPathState(13);
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.holdPoint(tras2);
                    setPathState(14);
                }
                break;

            case 14:
                trackTargetWithOdometry();

                if (!tragereInrogres) {
                    tragereInrogres = true;
                    stareaShooter = 0;
                }

                TragereLaPupitru();

                if (!tragereInrogres) {
                    setPathState(15);
                }
                break;

            case 15:
                n.loculete = 0;
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(luat3);
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
                if (n.loculete >= 3 || actionTimer.getElapsedTimeSeconds() >= 5.0) {
                    intakePornit = false;
                    setPathState(18);
                }
                break;

            case 18:
                intakePornit = false;
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
                trackTargetWithOdometry();

                if (!tragereInrogres) {
                    tragereInrogres = true;
                    stareaShooter = 0;
                }

                TragereLaPupitru();

                if (!tragereInrogres) {
                    setPathState(21);
                }
                break;

            case 21:
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
        tragereInrogres = false;
        stopThread = false;
        intakePornit = false;
        Intake();
        IntakeThread.start();
    }

    @Override
    public void stop() {
        stopThread = true;

        Pose currentPose = follower.getPose();
        RobotPozitie.X = currentPose.getX();
        RobotPozitie.Y = currentPose.getY();
        RobotPozitie.heading = currentPose.getHeading();

        n.shooter.setVelocity(0);
        n.intake.setPower(0);
    }
}