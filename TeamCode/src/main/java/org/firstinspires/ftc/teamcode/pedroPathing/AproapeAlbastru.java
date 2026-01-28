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

@Autonomous(name = "Departe Albastru")
public class AproapeAlbastru extends OpMode {
    sistemeAuto n = new sistemeAuto();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose shootingPose = new Pose(59.39160839160837, 24.083916083916087, Math.toRadians(90));

    private final Pose colectare = new Pose(16.34965034965035, 36.34965034965035, Math.toRadians(180));
    private final Pose controlPoint = new Pose(64.36713286713285, 37.29370629370628);
    private final Pose tragere2 = new Pose(60.54545454545453, 24.020979020979027, Math.toRadians(90));
    private final Pose controlTragere = new Pose(41.59790209790209, 18.96153846153846, Math.toRadians(90));
    private final Pose iesirea = new Pose(36.71226672976966,10.97982064609837,Math.toRadians(90));

    private Path laShooting;
    private PathChain iale, tragere,iesireas;

    private boolean TragereInProgres = false;
    private int ShootingStare = 0;
    private int currentShootSlot = 2;
    private int ballsToShoot = 0;

    private volatile boolean[] slotOcupat = new boolean[3];
    private volatile boolean intakePornit = false;
    private volatile boolean stop = false;
    private Thread IntakeThread;

    private int getLoculete() {
        int count = 0;
        for (boolean occupied : slotOcupat) {
            if (occupied) count++;
        }
        return count;
    }

    public void buildPaths() {
        laShooting = new Path(new BezierLine(startPose, shootingPose));
        laShooting.setConstantHeadingInterpolation(startPose.getHeading());

        iale = follower.pathBuilder()
                .addPath(new BezierCurve(shootingPose, controlPoint, colectare))
                .setTangentHeadingInterpolation()
                .build();
        tragere = follower.pathBuilder()
                .addPath(new BezierCurve(colectare, controlTragere, tragere2))
                .setLinearHeadingInterpolation(colectare.getHeading(), tragere2.getHeading())
                .setHeadingConstraint(10)
                .setTimeoutConstraint(50)
                .build();
        iesireas = follower.pathBuilder()
                .addPath(new BezierLine(tragere2,iesirea))
                .setLinearHeadingInterpolation(tragere2.getHeading(),iesirea.getHeading())
                .build();
    }

    private void TragereLaPupitru() {
        switch (ShootingStare) {
            case 0:
                PIDFCoefficients pid = new PIDFCoefficients(n.SkP, n.SkI, n.SkD, n.SkF);
                n.shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
                n.shooter2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
                n.shooter.setVelocity(1850);
                n.shooter2.setVelocity(1850);
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
                ballsToShoot = getLoculete();
                if (ballsToShoot > 0 && currentShootSlot >= 0) {
                    ShootingStare = 3;
                } else {
                    ShootingStare = 10;
                }
                break;

            case 3:
                while (currentShootSlot >= 0 && !slotOcupat[currentShootSlot]) {
                    currentShootSlot--;
                }

                if (currentShootSlot >= 0 && slotOcupat[currentShootSlot]) {
                    if (currentShootSlot == 0) {
                        n.sortare.setPosition(Pozitii.aruncare1);
                    } else if (currentShootSlot == 1) {
                        n.sortare.setPosition(Pozitii.aruncare2);
                    } else if (currentShootSlot == 2) {
                        n.sortare.setPosition(Pozitii.aruncare3);
                    }
                    actionTimer.resetTimer();
                    ShootingStare = 4;
                } else {
                    ShootingStare = 10;
                }
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
                if (actionTimer.getElapsedTimeSeconds() >= 0.3) {
                    n.shooter.setVelocity(0);
                    n.shooter2.setVelocity(0);
                    ShootingStare = 12;
                }
                break;

            case 12:
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;
                currentShootSlot = 2;
                TragereInProgres = false;
                ShootingStare = 0;
                break;
        }
    }

/*    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }*/

    private void track() {
        n.turelaS.setPosition(0.6749);
        n.turelaD.setPosition(0.6749);
    }

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
                track();

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
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(iale,0.7,false);
                setPathState(5);
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.holdPoint(colectare);
                    actionTimer.resetTimer();
                    setPathState(6);
                }
                break;

            case 6:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 2.0) {
                    intakePornit = false;
                    if (getLoculete() > 0) {
                        setPathState(7);
                    } else {
                        setPathState(-1);
                    }
                }
                break;

            case 7:
                follower.followPath(tragere);
                setPathState(8);
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.holdPoint(tragere2);
                    setPathState(9);
                }
                break;

            case 9:
                track();
                if (!TragereInProgres) {
                    TragereInProgres = true;
                    ShootingStare = 0;
                }
                TragereLaPupitru();
                if (!TragereInProgres) {
                    follower.followPath(iesireas);
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
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Loculete", getLoculete());
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
        stop = false;
        intakePornit = false;
        currentShootSlot = 2;

        slotOcupat[0] = true;
        slotOcupat[1] = true;
        slotOcupat[2] = true;

        n.sortare.setPosition(Pozitii.luarea1);

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
