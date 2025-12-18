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
public class RosuAuto extends OpMode {
    sistemeAuto n = new sistemeAuto();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private static final double TARGET_X = 144;
    private static final double TARGET_Y = 144;
    private static final double TICKS_PER_DEGREE = 1.35;
    private static final double MAX_TURRET_ANGLE = 90;
    private static final double MIN_TURRET_ANGLE = -90;
    private static final double TURRET_POWER = 1;


    private final Pose startPose = new Pose(124.53146853146853, 126.37762237762237, Math.toRadians(42));
    private final Pose tragere1 = new Pose(76.86713286713287, 76.6993006993007, Math.toRadians(42));
    private final Pose aduna1 = new Pose(84.58741258741259, 71.83216783216783);
    private final Pose aluat1 = new Pose(130.81818181818183, 89.61538461538461, Math.toRadians(0));
    private final Pose tras2 = new Pose(76.53146853146853, 76.36363636363636, Math.toRadians(42));


    private Path scorePreload;
    private PathChain luat1, tras1;

    private boolean TragereInProgres = false;
    private int BilaTrasa = 0;
    private int ShootingStare = 0;

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

    private void TragereLaPupitru() {
        switch (ShootingStare) {
            case 0:
                PIDFCoefficients pid = new PIDFCoefficients(n.SkP, n.SkI, n.SkD, n.SkF);
                n.shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
                n.shooter.setVelocity(1800);
                n.unghiS.setPosition(pop.posUnghi);
                n.unghiD.setPosition(pop.posUnghi);
                trackTargetWithOdometry();
                actionTimer.resetTimer();
                ShootingStare = 1;
                break;

            case 1:
                if (actionTimer.getElapsedTimeSeconds() >= 0.5) {
                    BilaTrasa = 0;
                    ShootingStare = 2;
                }
                break;

            case 2:
                if (n.loculete > 3) n.loculete = 3;
                if (n.loculete < 0) n.loculete = 0;

                if (n.loculete > 0) {
                    ShootingStare = 3;
                } else {
                    ShootingStare = 10;
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
                    n.loculete--;
                    BilaTrasa++;
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
                    n.shooter.setVelocity(0);
                    ShootingStare = 12;
                }
                break;

            case 12:
                n.loculete = 0;
                TragereInProgres = false;
                ShootingStare = 0;
                break;
        }
    }

    private volatile boolean intakePornit = false;
    private volatile boolean stop = false;
    private Thread IntakeThread;

    private void Intake() {
        IntakeThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!stop) {
                    if (intakePornit && n.loculete < 3) {
                        n.intake.setPower(1);

                        double imata = n.distanta.getDistance(DistanceUnit.CM);

                        if (imata < 20) {
                            if (n.loculete == 0) {
                                n.kdf(150);
                                n.loculete = 1;
                                n.sortare.setPosition(Pozitii.luarea2);
                            } else if (n.loculete == 1) {
                                n.kdf(150);
                                n.loculete = 2;
                                n.sortare.setPosition(Pozitii.luarea3);
                            } else if (n.loculete == 2) {
                                n.kdf(150);
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
                    setPathState(3);
                }
                break;

            case 3:
                n.loculete = 0;
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.setMaxPower(0.45);
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
                // Wait until 3 balls collected or 5 second timeout
                if (n.loculete >= 3 || actionTimer.getElapsedTimeSeconds() >= 5.0) {
                    intakePornit = false;
                    setPathState(6);
                }
                break;

            case 6:
                intakePornit = false;
                follower.setMaxPower(1.0);
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
        TragereInProgres = false;
        stop = false;
        intakePornit = false;
        Intake();
        IntakeThread.start();
    }

    @Override
    public void stop() {
        stop = true;

        // Save robot position for TeleOp
        Pose currentPose = follower.getPose();
        RobotPozitie.X = currentPose.getX();
        RobotPozitie.Y = currentPose.getY();
        RobotPozitie.heading = currentPose.getHeading();

        n.shooter.setVelocity(0);
        n.intake.setPower(0);
    }
}