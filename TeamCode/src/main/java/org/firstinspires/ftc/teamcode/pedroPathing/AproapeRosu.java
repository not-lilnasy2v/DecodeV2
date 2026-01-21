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

import org.firstinspires.ftc.teamcode.Pozitii;
import org.firstinspires.ftc.teamcode.RobotPozitie;
import org.firstinspires.ftc.teamcode.pop;
import org.firstinspires.ftc.teamcode.sistemeAuto;

@Autonomous(name = "Departe Rosu")
public class AproapeRosu extends OpMode {
    sistemeAuto n = new sistemeAuto();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private static final double TARGET_X = 144;
    private static final double TARGET_Y = 144;

    // Turret servo constants (Axon Max 355 degrees)
    private static final double TURRET_SERVO_CENTER = 0.5;
    private static final double TURRET_DEGREES_RANGE = 355.0;
    private static final double MAX_TURRET_ANGLE = 110;
    private static final double MIN_TURRET_ANGLE = -110;

    private final Pose startPose = new Pose(88, 8,Math.toRadians(90));
    private final Pose shootingPose = new Pose(82.53216783216781, 22.069930069930074, Math.toRadians(90));

    private final Pose parkingPose = new Pose(100, 8, Math.toRadians(90));

    private Path laShooting;
    private PathChain laParc;

    private boolean TragereInProgres = false;
    private int ShootingStare = 0;

    public void buildPaths() {
        laShooting = new Path(new BezierLine(startPose, shootingPose));
        laShooting.setConstantHeadingInterpolation(startPose.getHeading());

//        laParc = new Path(new BezierLine(shootingPose,parkingPose));
//        
    }

    private void TragereLaPupitru() {
        switch (ShootingStare) {
            case 0:
                PIDFCoefficients pid = new PIDFCoefficients(n.SkP, n.SkI, n.SkD, n.SkF);
                n.shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
                n.shooter.setVelocity(2200);
                n.unghiS.setPosition(pop.posUnghi);
                n.unghiD.setPosition(pop.posUnghi);
                actionTimer.resetTimer();
                ShootingStare = 1;
                break;

            case 1:
                if (actionTimer.getElapsedTimeSeconds() >= 0.5) {
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

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
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

        double posS = n.turelaS.angleToPosition(turretAngleDeg);
        double posD = n.turelaD.angleToPosition(turretAngleDeg);

        n.turelaS.setPosition(posS);
        n.turelaD.setPosition(posD);

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(laShooting);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.holdPoint(shootingPose);
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
                    setPathState(3);
                }
                break;

            case 3:
                if (actionTimer.getElapsedTimeSeconds() >= 0.3) {
                    setPathState(4);
                }
                break;

            case 4:
                follower.followPath(laParc);
                setPathState(5);
                break;


            case 5:
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

        telemetry.addData("pose", follower.getPose());
        telemetry.addData("Path State", pathState);
        telemetry.addData("Loculete", n.loculete);
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
        n.loculete = 3;
        n.sortare.setPosition(Pozitii.luarea1);
    }

    @Override
    public void stop() {
        Pose currentPose = follower.getPose();
        RobotPozitie.X = currentPose.getX();
        RobotPozitie.Y = currentPose.getY();
        RobotPozitie.heading = currentPose.getHeading();

        n.shooter.setVelocity(0);
        n.intake.setPower(0);
    }
}
