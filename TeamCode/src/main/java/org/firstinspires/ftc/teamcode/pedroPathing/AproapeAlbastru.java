package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Pozitii;
import org.firstinspires.ftc.teamcode.RobotPozitie;
import org.firstinspires.ftc.teamcode.pop;
import org.firstinspires.ftc.teamcode.sistemeAuto;

@Autonomous(name = "Aproape Auto Albastru")
public class AproapeAlbastru extends OpMode {
    sistemeAuto n = new sistemeAuto();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // Turret tracking - Blue target
    private static final double TARGET_X = 0;
    private static final double TARGET_Y = 144;
    private static final double TICKS_PER_DEGREE = 1.35;
    private static final double MAX_TURRET_ANGLE = 90;
    private static final double MIN_TURRET_ANGLE = -90;
    private static final double TURRET_POWER = 1;

    // Positions
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose shootingPose = new Pose(72.25135703457383, 75.10137415032521, Math.toRadians(142));
    private final Pose parkPose = new Pose(104.94048608733925, 34.030417135312256, Math.toRadians(90));

    // Paths
    private Path toShooting, toPark;

    // Shooting state
    private boolean TragereInProgres = false;
    private int ShootingStare = 0;

    public void buildPaths() {
        toShooting = new Path(new BezierLine(startPose, shootingPose));
        toShooting.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(142));

        toPark = new Path(new BezierLine(shootingPose, parkPose));
        toPark.setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(90));
    }

    private void TragereLaPupitru() {
        switch (ShootingStare) {
            case 0:
                PIDFCoefficients pid = new PIDFCoefficients(n.SkP, n.SkI, n.SkD, n.SkF);
                n.shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
                n.shooter.setVelocity(1800);
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
                // Go to shooting position
                follower.followPath(toShooting);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.holdPoint(shootingPose);
                    setPathState(2);
                }
                break;

            case 2:
                // Track target and shoot all preloaded balls
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
                // Wait a moment before parking
                if (actionTimer.getElapsedTimeSeconds() >= 0.3) {
                    setPathState(4);
                }
                break;

            case 4:
                // Go to parking zone
                follower.followPath(toPark);
                setPathState(5);
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.holdPoint(parkPose);
                    setPathState(6);
                }
                break;

            case 6:
                // Done - stay parked
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
        n.loculete = 3;  // Preloaded with 3 balls
        n.sortare.setPosition(Pozitii.luarea1);
    }

    @Override
    public void stop() {
        // Save robot position for TeleOp
        Pose currentPose = follower.getPose();
        RobotPozitie.X = currentPose.getX();
        RobotPozitie.Y = currentPose.getY();
        RobotPozitie.heading = currentPose.getHeading();

        n.shooter.setVelocity(0);
        n.intake.setPower(0);
    }
}
