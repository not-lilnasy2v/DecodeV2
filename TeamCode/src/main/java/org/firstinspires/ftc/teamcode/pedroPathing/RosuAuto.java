package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Pozitii;
import org.firstinspires.ftc.teamcode.RobotPozitie;
import org.firstinspires.ftc.teamcode.pop;
import org.firstinspires.ftc.teamcode.sistemeAuto;

@Autonomous
public class RosuAuto extends OpMode {
    sistemeAuto n = new sistemeAuto();
    public Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private static final double TARGET_X = 144;
    private static final double TARGET_Y = 167;
    private final Pose startPose = new Pose(120.35855054036871, 127.40143772311603, Math.toRadians(37));
    private final Pose tragere1 = new Pose(84.16783216783216, 96.5874125874126, Math.toRadians(0));
    private final Pose aluat1 = new Pose(120.51322803071056, 87.17702577143136, Math.toRadians(0));
    private final Pose ARatat1 = new Pose(80.55254387312878, 67.32287653839902, Math.toRadians(0));
    private final Pose tras1 = new Pose(83.16783216783216, 96.5874125874126, Math.toRadians(0));
    private final Pose aduna2 = new Pose(72.35664335664335, 55.374125874125866, Math.toRadians(0));
    private final Pose aluat2 = new Pose(125.8479143234388,57.40247933884299, Math.toRadians(0));
    private final Pose tras2 = new Pose(83.16783216783216, 96.5874125874126, Math.toRadians(0));
//    private final Pose aduna3 = new Pose(62.96273656413514, 28.151596655093154, Math.toRadians(0));
//    private final Pose aluat3 = new Pose(125.1163333170326, 37.5416401780038 , Math.toRadians(0));
    private final Pose tras3 = new Pose(83.16783216783216,96.5874125874126,Math.toRadians(0));
    private final Pose returnToBase = new Pose(118.42657342657343,95.67832167832168,Math.toRadians(0));

    private final Pose deschideCombinat = new Pose(126.19230769230772, 66.92307692307692, Math.toRadians(40));
    private final Pose CPdeschideCombinat = new Pose(96.03907281529658, 63.576556310822035);
    private final Pose lansareGate = new Pose(88.69230769230768, 91.15384615384615, Math.toRadians(0));
    private final Pose CPlansareGate = new Pose(84.84615384615384, 69.7980769230769);
    private final Pose preGate = new Pose(131.34615384615384, 64.48076923076923, Math.toRadians(0));

    private final Pose gateBackoff = new Pose(
            deschideCombinat.getX() + 1.5, deschideCombinat.getY() - 1.5, deschideCombinat.getHeading());
    private boolean backingOff = false;
    private Timer backoffTimer = new Timer();
    private static final double JAM_CURRENT = 6.5;

    private PathChain scorePreload;
    private PathChain collectare1, Miss1, trasUnu, collectare2, trasDoi, returnarea, DuceGate, Trage_Gata;

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
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, tragere1))
                .setLinearHeadingInterpolation(startPose.getHeading(), tragere1.getHeading())
                .setGlobalDeceleration(5.0)
                .setBrakingStart(2.5)
                .build();

        collectare1 = follower.pathBuilder()
                .addPath(new BezierLine(tragere1, aluat1))
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
                .setGlobalDeceleration(5.0)
                .setBrakingStart(2.5)
                .build();

        collectare2 = follower.pathBuilder()
                .addPath(new BezierCurve(tras1, aduna2, aluat2))
                .setLinearHeadingInterpolation(tras1.getHeading(), aluat2.getHeading())
                .build();

        // Miss2 = follower.pathBuilder()
        //         .addPath(new BezierCurve(aluat2, ARatat2, aluat3))
        //         .setLinearHeadingInterpolation(aluat2.getHeading(), aluat3.getHeading())
        //         .build();

        trasDoi = follower.pathBuilder()
                .addPath(new BezierLine(aluat2, tras2))
                .setLinearHeadingInterpolation(aluat2.getHeading(), tras2.getHeading())
                .setTranslationalConstraint(1)
                .setTimeoutConstraint(70)
                .setGlobalDeceleration(5.0)
                .setBrakingStart(2.5)
                .build();

        // collectare3 = follower.pathBuilder()
        //         .addPath(new BezierCurve(tras2, aduna3, aluat3))
        //         .setLinearHeadingInterpolation(tras2.getHeading(), aluat3.getHeading())
        //         .build();

        // trasTrei = follower.pathBuilder()
        //         .addPath(new BezierLine(aluat3, tras3))
        //         .setLinearHeadingInterpolation(aluat3.getHeading(), tras3.getHeading())
        //         .setTranslationalConstraint(1)
        //         .setTimeoutConstraint(100)
        //         .build();
        returnarea=  follower.pathBuilder()
                .addPath(new BezierLine(lansareGate,returnToBase))
                .setLinearHeadingInterpolation(lansareGate.getHeading(),returnToBase.getHeading())
                .build();

        // TODO: pune traiectoriile corecte (control points, etc.)
        DuceGate = follower.pathBuilder()
                .addPath(new BezierCurve(tras2, CPdeschideCombinat, preGate))
                .setLinearHeadingInterpolation(tras2.getHeading(), preGate.getHeading())
                .build();

        Trage_Gata = follower.pathBuilder()
                .addPath(new BezierCurve(deschideCombinat, CPlansareGate, lansareGate))
                .setLinearHeadingInterpolation(deschideCombinat.getHeading(), lansareGate.getHeading())
                .setGlobalDeceleration(5.0)
                .setBrakingStart(2.5)
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
    private static final int[] SHOOT_ORDER = {0, 2, 1};
    private static final double[] SHOOT_POS = {Pozitii.aruncare1, Pozitii.aruncare3, Pozitii.aruncare2};

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
                actionTimer.resetTimer();
                ShootingStare = 5;
                break;
            case 5:
                double minWait = ShootSlot == 0 ? 0.15 : 0.03;
                double maxWait = ShootSlot == 0 ? 0.40 : 0.20;
                if (actionTimer.getElapsedTimeSeconds() >= minWait) {
                    boolean tagVisible = n.isTagVisible();
                    double tx = Math.abs(n.getLimelightTx());
                    boolean ready;
                    if (ShootSlot == 0) {
                        double v1 = Math.abs(n.shooter.getVelocity());
                        double v2 = Math.abs(n.shooter2.getVelocity());
                        ready = tagVisible && tx < 1.0
                                && v1 >= 1550 * 0.96 && v1 <= 1550 * 1.03
                                && v2 >= 1550 * 0.96 && v2 <= 1550 * 1.03;
                    } else {
                        ready = tagVisible && tx < 1.0;
                    }
                    if (ready || actionTimer.getElapsedTimeSeconds() >= maxWait) {
                        slotOcupat[SHOOT_ORDER[ShootSlot]] = false;
                        ShootSlot++;
                        if (ShootSlot > 2) {
                            n.bascula.setPosition(Pozitii.sede);
                            n.scula.setPower(0);
                            n.shooter.setVelocity(0);
                            n.shooter2.setVelocity(0);
                            n.sortare.setPosition(Pozitii.luarea1);
                            TragereInProgres = false;
                            ShootingStare = 0;
                        } else {
                            n.sortare.setPosition(SHOOT_POS[ShootSlot]);
                            actionTimer.resetTimer();
                        }
                    }
                }
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
                                n.kdf(80);

                            } else if (Math.abs(servoPos - Pozitii.luarea2) < 0.1 && !slotOcupat[1]) {
                                slotOcupat[1] = true;
                                if (!slotOcupat[2]) {
                                    n.sortare.setPosition(Pozitii.luarea3);
                                } else if (!slotOcupat[0]) {
                                    n.sortare.setPosition(Pozitii.luarea1);
                                }
                                n.kdf(80);

                            } else if (Math.abs(servoPos - Pozitii.luarea3) < 0.1 && !slotOcupat[2]) {
                                slotOcupat[2] = true;
                                if (!slotOcupat[0]) {
                                    n.sortare.setPosition(Pozitii.luarea1);
                                } else if (!slotOcupat[1]) {
                                    n.sortare.setPosition(Pozitii.luarea2);
                                }
                                n.kdf(80);
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
                    actionTimer.resetTimer();
                    setPathState(22);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.holdPoint(aluat1);
                    actionTimer.resetTimer();
                    setPathState(4);
                }
                break;

            case 4:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 0.8) {
                    intakePornit = false;
                    if (getLoculete() == 0) {
                        setPathState(23);
                    } else {
                        setPathState(5);
                    }
                }
                break;

            case 5:
                intakePornit = false;
                pregatireShooter();
                n.sortare.setPosition(Pozitii.aruncare1);
                follower.followPath(trasUnu);
                setPathState(6);
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.holdPoint(tras1);
                    setPathState(7);
                }
                break;

            case 7:
                if (!TragereInProgres) {
                    TragereInProgres = true;
                    ShootingStare = 0;
                }
                TragereLaPupitru();

                if (!TragereInProgres) {
                    setPathState(8);
                }
                break;

            case 8:
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(collectare2,0.85,false);
                setPathState(9);
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.holdPoint(aluat2);
                    actionTimer.resetTimer();
                    setPathState(10);
                }
                break;

            case 10:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 0.8) {
                    intakePornit = false;
                    if (getLoculete() == 0) {
                        setPathState(21);
                    } else {
                        setPathState(11);
                    }
                }
                break;

            case 11:
                intakePornit = false;
                pregatireShooter();
                n.sortare.setPosition(Pozitii.aruncare1);
                follower.followPath(trasDoi);
                setPathState(12);
                break;

            case 12:
                if (!follower.isBusy()) {
                    follower.holdPoint(tras2);
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
                    setPathState(40);
                }
                break;

            // case 14:
            //     slotOcupat[0] = false;
            //     slotOcupat[1] = false;
            //     slotOcupat[2] = false;
            //     n.sortare.setPosition(Pozitii.luarea1);
            //     intakePornit = true;
            //     follower.followPath(collectare3,0.85,false);
            //     setPathState(15);
            //     break;
            // case 15:
            //     if (!follower.isBusy()) {
            //         follower.holdPoint(aluat3);
            //         actionTimer.resetTimer();
            //         setPathState(16);
            //     }
            //     break;
            // case 16:
            //     if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 0.9) {
            //         intakePornit = false;
            //         if (getLoculete() == 0) {
            //             setPathState(21);
            //         } else {
            //             setPathState(17);
            //         }
            //     }
            //     break;
            // case 17:
            //     intakePornit = false;
            //     pregatireShooter();
            //     n.sortare.setPosition(Pozitii.aruncare1);
            //     follower.followPath(trasTrei);
            //     setPathState(18);
            //     break;
            // case 18:
            //     if (!follower.isBusy()) {
            //         follower.holdPoint(tras3);
            //         setPathState(19);
            //     }
            //     break;
            // case 19:
            //     if (!TragereInProgres) {
            //         TragereInProgres = true;
            //         ShootingStare = 0;
            //     }
            //     TragereLaPupitru();
            //     if (!TragereInProgres) {
            //         setPathState(20);
            //     }
            //     break;

            // === Gate #1 ===
            case 40:
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(DuceGate);
                setPathState(41);
                break;

            case 41:
                if (getLoculete() >= 3) {
                    intakePornit = false;
                    pregatireShooter();
                    follower.followPath(Trage_Gata);
                    setPathState(43);
                } else if (!follower.isBusy()) {
                    follower.holdPoint(deschideCombinat);
                    actionTimer.resetTimer();
                    setPathState(42);
                }
                break;

            case 42:
                if (!backingOff) {
                    double amps42 = n.intake.getCurrent(CurrentUnit.AMPS);
                    if (amps42 > JAM_CURRENT) {
                        backingOff = true;
                        follower.holdPoint(gateBackoff);
                        backoffTimer.resetTimer();
                    }
                } else if (backoffTimer.getElapsedTimeSeconds() >= 0.35) {
                    backingOff = false;
                    follower.holdPoint(deschideCombinat);
                }
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 2.5) {
                    backingOff = false;
                    intakePornit = false;
                    pregatireShooter();
                    follower.followPath(Trage_Gata);
                    setPathState(43);
                }
                break;

            case 43:
                if (!follower.isBusy()) {
                    follower.holdPoint(lansareGate);
                    TragereInProgres = true;
                    ShootingStare = 0;
                    setPathState(44);
                }
                break;

            case 44:
                TragereLaPupitru();
                if (!TragereInProgres) {
                    setPathState(50);
                }
                break;

            // === Gate #2 ===
            case 50:
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(DuceGate);
                setPathState(51);
                break;

            case 51:
                if (getLoculete() >= 3) {
                    intakePornit = false;
                    pregatireShooter();
                    follower.followPath(Trage_Gata);
                    setPathState(53);
                } else if (!follower.isBusy()) {
                    follower.holdPoint(deschideCombinat);
                    actionTimer.resetTimer();
                    setPathState(52);
                }
                break;

            case 52:
                if (!backingOff) {
                    double amps52 = n.intake.getCurrent(CurrentUnit.AMPS);
                    if (amps52 > JAM_CURRENT) {
                        backingOff = true;
                        follower.holdPoint(gateBackoff);
                        backoffTimer.resetTimer();
                    }
                } else if (backoffTimer.getElapsedTimeSeconds() >= 0.35) {
                    backingOff = false;
                    follower.holdPoint(deschideCombinat);
                }
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 2.5) {
                    backingOff = false;
                    intakePornit = false;
                    pregatireShooter();
                    follower.followPath(Trage_Gata);
                    setPathState(53);
                }
                break;

            case 53:
                if (!follower.isBusy()) {
                    follower.holdPoint(lansareGate);
                    TragereInProgres = true;
                    ShootingStare = 0;
                    setPathState(54);
                }
                break;

            case 54:
                TragereLaPupitru();
                if (!TragereInProgres) {
                    setPathState(20);
                }
                break;

            case 20:
                follower.followPath(returnarea);
                setPathState(21);
                break;

            case 21:
                if (!follower.isBusy()) {
                    follower.holdPoint(returnToBase);
                    setPathState(-1);
                }
                break;

            case 22:
                if (actionTimer.getElapsedTimeSeconds() >= 0.5) {
                    slotOcupat[0] = false;
                    slotOcupat[1] = false;
                    slotOcupat[2] = false;
                    n.sortare.setPosition(Pozitii.luarea1);
                    intakePornit = true;
                    follower.followPath(collectare1,0.9,false);

                    setPathState(3);
                }
                break;


            case 23:
                slotOcupat[0] = false;
                slotOcupat[1] = false;
                slotOcupat[2] = false;
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(Miss1);
                setPathState(24);
                break;

            case 24:
                if (!follower.isBusy()) {
                    follower.holdPoint(aluat2);
                    actionTimer.resetTimer();
                    setPathState(25);
                }
                break;

            case 25:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 0.8) {
                    intakePornit = false;
                    if (getLoculete() == 0) {
                        setPathState(21);
                    } else {
                        setPathState(11);
                    }
                }
                break;

            // case 26:
            //     slotOcupat[0] = false;
            //     slotOcupat[1] = false;
            //     slotOcupat[2] = false;
            //     n.sortare.setPosition(Pozitii.luarea1);
            //     intakePornit = true;
            //     follower.followPath(Miss2);
            //     setPathState(27);
            //     break;
            // case 27:
            //     if (!follower.isBusy()) {
            //         follower.holdPoint(aluat3);
            //         actionTimer.resetTimer();
            //         setPathState(28);
            //     }
            //     break;
            // case 28:
            //     if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= 0.9) {
            //         intakePornit = false;
            //         if (getLoculete() == 0) {
            //             setPathState(21);
            //         } else {
            //             setPathState(17);
            //         }
            //     }
            //     break;

            default:
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }



    private void startTracking() {
        TrackingThread = new Thread(() -> {
            while (!stop) {
                try { Thread.sleep(5); } catch (InterruptedException e) { break; }
                n.tracks(follower, TARGET_X, TARGET_Y);
            }
            n.turelaD.setPosition(0.5);
            n.turelaS.setPosition(0.5);
        });
        TrackingThread.start();
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

        n.limelight.pipelineSwitch(2);
        n.resetTurelaPID();
        n.applyVoltageCompensatedPIDF();
        n.shooter.setVelocity(1550);
        n.shooter2.setVelocity(1550);

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
        n.bascula.setPosition(Pozitii.sede);
    }
}