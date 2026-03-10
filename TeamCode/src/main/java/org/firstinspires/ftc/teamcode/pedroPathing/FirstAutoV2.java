package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Pozitii;
import org.firstinspires.ftc.teamcode.RobotPozitie;
import org.firstinspires.ftc.teamcode.pop;
import org.firstinspires.ftc.teamcode.sistemeAuto;

@Autonomous(name = "FirstAutoV2")
@Disabled
public class FirstAutoV2 extends OpMode {
    private static final double VELOCITY_TARGET    = 1600;
    private static final double VELOCITY_TOLERANCE = 100;    // +/-100 ticks/s
    private static final double SERVO_MIN_TIME     = 0.18;   // min timp servo travel (s)
    private static final double SHOT_TIMEOUT_FIRST = 0.40;   // timeout primul tir (s)
    private static final double SHOT_TIMEOUT_NEXT  = 0.30;   // timeout tiruri urmatoare (s)
    private static final double TURRET_AIM_TOL     = 5.0;    // eroare turela maxima (grade)

    private static final double PATH_TIMEOUT         = 5.0;  // timeout path normal (s)
    private static final double COLLECT_PATH_TIMEOUT = 6.0;  // timeout path colectare (s)
    private static final double SHOOT_CYCLE_TIMEOUT  = 3.0;  // timeout ciclu tragere complet (s)
    private static final double COLLECT_WAIT_L2      = 1.5;  // asteptare extra la L2 (s) — ca in FirstAuto
    private static final double COLLECT_WAIT_L1      = 0.9;  // asteptare extra la L1 (s) — ca in FirstAuto
    private static final double COLLECT_WAIT_GATE    = 2.5;  // asteptare la gate (s) — ca in FirstAuto
    private static final double JAM_CURRENT          = 6.5;  // amperaj intake = blocat (A)

    private static final double SKIP_L1_AFTER    = 19.0;     // sari L1+Gate#2 daca elapsed > 19s
    private static final double SKIP_GATE2_AFTER = 23.0;     // sari Gate#2 daca elapsed > 23s
    private static final double EMERGENCY_STOP   = 30;     // opreste tot la 29.5s

    private final Pose startPose        = new Pose(31.9423076923077, 137.67307692307696, Math.toRadians(180));
    private final Pose tragere1 = new Pose(60.72027972027973, 100.74825174825179, Math.toRadians(180));
    private final Pose aduna1 = new Pose(59.3076923076923, 100.5785892807554, Math.toRadians(180));
    private final Pose aluat1 = new Pose(21.55244755244755, 89.05804195804195, Math.toRadians(180));
    private final Pose tras1 = new Pose(58.72027972027973, 100.74825174825179, Math.toRadians(180));
    private final Pose aduna2 = new Pose(51.53846153846153, 58.84615384615384, Math.toRadians(180));
    private final Pose aluat2 = new Pose(13.67307692307692, 63.65384615384616, Math.toRadians(180));
    private final Pose tras2 = new Pose(60.72027972027973, 100.74825174825179, Math.toRadians(180));
    private final Pose returnToBase  = new Pose(28.790209790209786, 93.66433566433567, Math.toRadians(180));
    private final Pose deschideCombinat = new Pose(5.307692307692307, 59.07692307692307, Math.toRadians(135));
    private final Pose lansareGate = new Pose(60.69584837545127, 95.60351291307968,Math.toRadians(180));
    private final Pose preGate  = new Pose(10.692307692307686, 68.44230769230768, Math.toRadians(90));
    private final Pose CPpreGate = new Pose(66.55769230769232, 64.12499999999999);
    private final Pose midGate          = new Pose(44, 72, Math.toRadians(135));
    private final Pose gateBackoff      = new Pose(
            deschideCombinat.getX() + 2, deschideCombinat.getY() + 2, deschideCombinat.getHeading());

    private static final double[] SHOOT_POS = {Pozitii.aruncare1, Pozitii.aruncare3, Pozitii.aruncare2};
    private static final double TARGET_X = 0;
    private static final double TARGET_Y = 144;
    sistemeAuto n = new sistemeAuto();
    public Follower follower;
    private List<LynxModule> hubs;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private boolean TragereInProgres = false;
    private int ShootingStare = 0;
    private int ShootSlot = 0;
    private int idTag = 0;
    private int shotsGood = 0;
    private int shotsTimeout = 0;
    private volatile boolean intakePornit = false;
    private volatile boolean stop = false;
    private final boolean[] slotOcupat = new boolean[3];
    private final Object slotLock = new Object();
    private PathChain scorePreload, collectare2, trasDoi, DuceGate, Trage_Gata;
    private PathChain collectare1, trasUnu, DucePreGate2, returnarea;
    private Thread Intake;
    private Thread TrackingT;
    private Timer bumpTimer = new Timer();
    private boolean bumpingBack = false;
    private int getLoculete() {
        synchronized (slotLock) {
            int count = 0;
            for (boolean occupied : slotOcupat) {
                if (occupied) count++;
            }
            return count;
        }
    }

    private void resetSlots() {
        synchronized (slotLock) {
            slotOcupat[0] = false;
            slotOcupat[1] = false;
            slotOcupat[2] = false;
        }
    }

    private void fillSlots() {
        synchronized (slotLock) {
            slotOcupat[0] = true;
            slotOcupat[1] = true;
            slotOcupat[2] = true;
        }
    }
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, tragere1))
                .setLinearHeadingInterpolation(startPose.getHeading(), tragere1.getHeading())
                .setGlobalDeceleration(5.0)
                .setBrakingStart(2.5)
                .build();

        collectare2 = follower.pathBuilder()
                .addPath(new BezierCurve(tragere1, aduna2, aluat2))
                .setLinearHeadingInterpolation(tragere1.getHeading(), aluat2.getHeading())
                .setGlobalDeceleration(4.0)
                .setBrakingStart(2.0)
                .build();

        trasDoi = follower.pathBuilder()
                .addPath(new BezierLine(aluat2, tras2))
                .setLinearHeadingInterpolation(aluat2.getHeading(), tras2.getHeading())
                .setGlobalDeceleration(5.0)
                .setBrakingStart(2.5)
                .build();

        DuceGate = follower.pathBuilder()
                .addPath(new BezierLine(tras2, midGate))
                .setLinearHeadingInterpolation(tras2.getHeading(), midGate.getHeading())
                .addPath(new BezierLine(midGate, preGate))
                .setLinearHeadingInterpolation(midGate.getHeading(), preGate.getHeading())
                .addPath(new BezierLine(preGate, deschideCombinat))
                .setLinearHeadingInterpolation(preGate.getHeading(), deschideCombinat.getHeading())
                .setGlobalDeceleration(4.0)
                .setBrakingStart(2.0)
                .build();

        Trage_Gata = follower.pathBuilder()
                .addPath(new BezierLine(deschideCombinat, lansareGate))
                .setLinearHeadingInterpolation(deschideCombinat.getHeading(), lansareGate.getHeading())
                .setGlobalDeceleration(5.0)
                .setBrakingStart(2.5)
                .build();

        collectare1 = follower.pathBuilder()
                .addPath(new BezierCurve(lansareGate, aduna1, aluat1))
                .setLinearHeadingInterpolation(lansareGate.getHeading(), aluat1.getHeading())
                .build();

        trasUnu = follower.pathBuilder()
                .addPath(new BezierLine(aluat1, tras1))
                .setLinearHeadingInterpolation(aluat1.getHeading(), tras1.getHeading())
                .setGlobalDeceleration(5.0)
                .setBrakingStart(2.5)
                .build();

        DucePreGate2 = follower.pathBuilder()
                .addPath(new BezierLine(tras1, midGate))
                .setLinearHeadingInterpolation(tras1.getHeading(), midGate.getHeading())
                .addPath(new BezierLine(midGate, preGate))
                .setLinearHeadingInterpolation(midGate.getHeading(), preGate.getHeading())
                .addPath(new BezierLine(preGate, deschideCombinat))
                .setLinearHeadingInterpolation(preGate.getHeading(), deschideCombinat.getHeading())
                .setGlobalDeceleration(4.0)
                .setBrakingStart(2.0)
                .build();

        returnarea = follower.pathBuilder()
                .addPath(new BezierLine(lansareGate, returnToBase))
                .setLinearHeadingInterpolation(lansareGate.getHeading(), returnToBase.getHeading())
                .build();
    }

    private void pregatireShooter() {
        n.applyVoltageCompensatedPIDF();
        n.shooter.setVelocity(VELOCITY_TARGET);
        n.shooter2.setVelocity(VELOCITY_TARGET);
        n.unghiD.setPosition(pop.posUnghi);
        n.sortare.setPosition(Pozitii.aruncare1);
    }

    // Termina ciclul fara sa opreasca shooterele (raman spinning)
    private void finishShootingCycle() {
        n.bascula.setPosition(Pozitii.sede);
        n.scula.setPower(0);
        n.sortare.setPosition(Pozitii.luarea1);
        TragereInProgres = false;
        ShootingStare = 0;
    }

    // Opreste TOTUL — doar la parcare si end
    private void fullShutdown() {
        n.shooter.setVelocity(0);
        n.shooter2.setVelocity(0);
        n.intake.setPower(0);
        n.scula.setPower(0);
        n.bascula.setPosition(Pozitii.sede);
        n.sortare.setPosition(Pozitii.luarea1);
        intakePornit = false;
        TragereInProgres = false;
        ShootingStare = 0;
    }

    // Park din orice pozitie curenta
    private void parkFromCurrentPose() {
        if (TrackingT != null) TrackingT.interrupt();
        n.turelaD.setPosition(0.5);
        n.turelaS.setPosition(0.5);
        intakePornit = false;
        Pose cur = follower.getPose();
        PathChain park = follower.pathBuilder()
                .addPath(new BezierLine(cur, returnToBase))
                .setLinearHeadingInterpolation(cur.getHeading(), returnToBase.getHeading())
                .build();
        follower.followPath(park);
    }

    private void TragereLaPupitru() {
        switch (ShootingStare) {
            case 0:
                ShootSlot = 0;
                n.scula.setPower(-1);
                n.bascula.setPosition(Pozitii.lansareRapid);
                n.applyVoltageCompensatedPIDF();
                n.shooter.setVelocity(VELOCITY_TARGET);
                n.shooter2.setVelocity(VELOCITY_TARGET);
                n.unghiD.setPosition(pop.posUnghi);
                idTag = n.detectIdTag();
                n.sortare.setPosition(SHOOT_POS[0]);
                actionTimer.resetTimer();
                ShootingStare = 1;
                break;

            case 1:
                double elapsed = actionTimer.getElapsedTimeSeconds();
                double timeout = ShootSlot == 0 ? SHOT_TIMEOUT_FIRST : SHOT_TIMEOUT_NEXT;

                boolean servoReady = elapsed >= SERVO_MIN_TIME;
                boolean velocityOK = Math.abs(n.shooter.getVelocity() - VELOCITY_TARGET) < VELOCITY_TOLERANCE
                                  && Math.abs(n.shooter2.getVelocity() - VELOCITY_TARGET) < VELOCITY_TOLERANCE;
                boolean turretOK = n.getTurelaError() < TURRET_AIM_TOL;
                boolean qualityShot = servoReady && velocityOK && turretOK;

                if (qualityShot || elapsed >= timeout) {
                    if (qualityShot) {
                        shotsGood++;
                    } else {
                        shotsTimeout++;
                    }

                    ShootSlot++;
                    if (ShootSlot > 2) {
                        finishShootingCycle();
                    } else {
                        n.applyVoltageCompensatedPIDF();
                        n.sortare.setPosition(SHOOT_POS[ShootSlot]);
                        actionTimer.resetTimer();
                    }
                }
                break;
        }
    }

    // =====================================================================
    // INTAKE THREAD
    // =====================================================================
    private void createIntakeThread() {
        Intake = new Thread(new Runnable() {
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

                            synchronized (slotLock) {
                                if (Math.abs(servoPos - Pozitii.luarea1) < 0.1 && !slotOcupat[0]) {
                                    slotOcupat[0] = true;
                                    if (!slotOcupat[1]) {
                                        n.sortare.setPosition(Pozitii.luarea2);
                                    } else if (!slotOcupat[2]) {
                                        n.sortare.setPosition(Pozitii.luarea3);
                                    }
                                } else if (Math.abs(servoPos - Pozitii.luarea2) < 0.1 && !slotOcupat[1]) {
                                    slotOcupat[1] = true;
                                    if (!slotOcupat[2]) {
                                        n.sortare.setPosition(Pozitii.luarea3);
                                    } else if (!slotOcupat[0]) {
                                        n.sortare.setPosition(Pozitii.luarea1);
                                    }
                                } else if (Math.abs(servoPos - Pozitii.luarea3) < 0.1 && !slotOcupat[2]) {
                                    slotOcupat[2] = true;
                                    if (!slotOcupat[0]) {
                                        n.sortare.setPosition(Pozitii.luarea1);
                                    } else if (!slotOcupat[1]) {
                                        n.sortare.setPosition(Pozitii.luarea2);
                                    }
                                }
                            }
                            n.kdf(80);

                        } else if (leDistanta >= 20 && ballBeingProcessed) {
                            ballBeingProcessed = false;
                        }
                    } else if (!intakePornit) {
                        n.intake.setPower(0);
                        ballBeingProcessed = false;
                    }
                }
            }
        }, "AutoIntake");
    }

    // =====================================================================
    // TRACKING THREAD
    // =====================================================================
    private void startTracking() {
        TrackingT = new Thread(() -> {
            while (!stop) {
                try { Thread.sleep(5); } catch (InterruptedException e) { break; }
                n.tracks(follower, TARGET_X, TARGET_Y);
            }
            n.turelaD.setPosition(0.5);
            n.turelaS.setPosition(0.5);
        }, "AutoTracking");
        TrackingT.start();
    }

    // =====================================================================
    // STATE MACHINE
    // Flow: Preload(3) -> L2(3) -> Gate(3) -> Park
    // Total maxim: 9 artefacte
    // States: 0-2 Preload, 3-5 CollectL2, 6-8 ShootL2,
    //         9-11 Gate, 12-13 ShootGate,
    //         25-26 Park, 28 EmergencyPark
    // =====================================================================
    public void autonomousPathUpdate() {
        switch (pathState) {

            // ========== PRELOAD ==========
            case 0:
                follower.followPath(scorePreload);
                pregatireShooter();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= PATH_TIMEOUT) {
                    follower.holdPoint(tragere1);
                    n.resetLimelightCorrection();
                    TragereInProgres = true;
                    ShootingStare = 0;
                    setPathState(2);
                }
                break;
            case 2:
                TragereLaPupitru();
                if (!TragereInProgres || pathTimer.getElapsedTimeSeconds() >= SHOOT_CYCLE_TIMEOUT) {
                    if (TragereInProgres) finishShootingCycle();
                    setPathState(3);
                }
                break;

            // ========== COLLECT LINE 2 ==========
            case 3:
                resetSlots();
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(collectare2);
                setPathState(4);
                break;
            case 4:
                if (getLoculete() >= 3) {
                    intakePornit = false;
                    setPathState(6);
                } else if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= COLLECT_PATH_TIMEOUT) {
                    follower.holdPoint(aluat2);
                    actionTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= COLLECT_WAIT_L2) {
                    intakePornit = false;
                    setPathState(6);
                }
                break;

            // ========== SHOOT LINE 2 ==========
            case 6: {
                intakePornit = false;
                pregatireShooter();
                Pose cur6 = follower.getPose();
                PathChain toShoot2 = follower.pathBuilder()
                        .addPath(new BezierLine(cur6, tras2))
                        .setLinearHeadingInterpolation(cur6.getHeading(), tras2.getHeading())
                        .setGlobalDeceleration(5.0)
                        .setBrakingStart(2.5)
                        .build();
                follower.followPath(toShoot2);
                setPathState(7);
                break;
            }
            case 7:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= PATH_TIMEOUT) {
                    follower.holdPoint(tras2);
                    n.resetLimelightCorrection();
                    TragereInProgres = true;
                    ShootingStare = 0;
                    setPathState(8);
                }
                break;
            case 8:
                TragereLaPupitru();
                if (!TragereInProgres || pathTimer.getElapsedTimeSeconds() >= SHOOT_CYCLE_TIMEOUT) {
                    if (TragereInProgres) finishShootingCycle();
                    setPathState(9);
                }
                break;

            // ========== GATE #1 ==========
            case 9:
                resetSlots();
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(DuceGate);
                setPathState(10);
                break;
            case 10:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= COLLECT_PATH_TIMEOUT) {
                    actionTimer.resetTimer();
                    bumpTimer.resetTimer();
                    bumpingBack = true;
                    follower.holdPoint(gateBackoff);
                    setPathState(11);
                }
                break;
            case 11:
                if (bumpingBack) {
                    if (bumpTimer.getElapsedTimeSeconds() >= 0.5) {
                        bumpingBack = false;
                        follower.holdPoint(deschideCombinat);
                        bumpTimer.resetTimer();
                    }
                } else {
                    double amps11 = n.intake.getCurrent(CurrentUnit.AMPS);
                    if (bumpTimer.getElapsedTimeSeconds() >= 0.7 || amps11 > JAM_CURRENT) {
                        bumpingBack = true;
                        follower.holdPoint(gateBackoff);
                        bumpTimer.resetTimer();
                    }
                }
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= COLLECT_WAIT_GATE) {
                    bumpingBack = false;
                    intakePornit = false;
                    pregatireShooter();
                    follower.followPath(Trage_Gata);
                    setPathState(12);
                }
                break;

            // ========== SHOOT GATE #1 ==========
            case 12:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= PATH_TIMEOUT) {
                    follower.holdPoint(lansareGate);
                    n.resetLimelightCorrection();
                    TragereInProgres = true;
                    ShootingStare = 0;
                    setPathState(13);
                }
                break;
            case 13:
                TragereLaPupitru();
                if (!TragereInProgres || pathTimer.getElapsedTimeSeconds() >= SHOOT_CYCLE_TIMEOUT) {
                    if (TragereInProgres) finishShootingCycle();
                    setPathState(25);
                }
                break;

            // ========== COLLECT LINE 1 ==========
            case 14:
                if (opmodeTimer.getElapsedTimeSeconds() > SKIP_L1_AFTER) {
                    parkFromCurrentPose();
                    setPathState(28);
                    break;
                }
                resetSlots();
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(collectare1);
                setPathState(15);
                break;
            case 15:
                if (getLoculete() >= 3) {
                    intakePornit = false;
                    setPathState(17);
                } else if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= COLLECT_PATH_TIMEOUT) {
                    follower.holdPoint(aluat1);
                    actionTimer.resetTimer();
                    setPathState(16);
                }
                break;
            case 16:
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= COLLECT_WAIT_L1) {
                    intakePornit = false;
                    setPathState(17);
                }
                break;

            // ========== SHOOT LINE 1 ==========
            case 17:
                intakePornit = false;
                pregatireShooter();
                follower.followPath(trasUnu);
                setPathState(18);
                break;
            case 18:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= PATH_TIMEOUT) {
                    follower.holdPoint(tras1);
                    n.resetLimelightCorrection();
                    TragereInProgres = true;
                    ShootingStare = 0;
                    setPathState(19);
                }
                break;
            case 19:
                TragereLaPupitru();
                if (!TragereInProgres || pathTimer.getElapsedTimeSeconds() >= SHOOT_CYCLE_TIMEOUT) {
                    if (TragereInProgres) finishShootingCycle();
                    setPathState(20);
                }
                break;

            // ========== GATE #2 ==========
            case 20:
                if (opmodeTimer.getElapsedTimeSeconds() > SKIP_GATE2_AFTER) {
                    parkFromCurrentPose();
                    setPathState(28);
                    break;
                }
                resetSlots();
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(DucePreGate2);
                setPathState(21);
                break;
            case 21:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= COLLECT_PATH_TIMEOUT) {
                    actionTimer.resetTimer();
                    bumpTimer.resetTimer();
                    bumpingBack = true;
                    follower.holdPoint(gateBackoff);
                    setPathState(22);
                }
                break;
            case 22:
                if (bumpingBack) {
                    if (bumpTimer.getElapsedTimeSeconds() >= 0.5) {
                        bumpingBack = false;
                        follower.holdPoint(deschideCombinat);
                        bumpTimer.resetTimer();
                    }
                } else {
                    double amps22 = n.intake.getCurrent(CurrentUnit.AMPS);
                    if (bumpTimer.getElapsedTimeSeconds() >= 0.7 || amps22 > JAM_CURRENT) {
                        bumpingBack = true;
                        follower.holdPoint(gateBackoff);
                        bumpTimer.resetTimer();
                    }
                }
                if (getLoculete() >= 3 || actionTimer.getElapsedTimeSeconds() >= COLLECT_WAIT_GATE) {
                    bumpingBack = false;
                    intakePornit = false;
                    pregatireShooter();
                    follower.followPath(Trage_Gata);
                    setPathState(23);
                }
                break;

            // ========== SHOOT GATE #2 ==========
            case 23:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= PATH_TIMEOUT) {
                    follower.holdPoint(lansareGate);
                    n.resetLimelightCorrection();
                    TragereInProgres = true;
                    ShootingStare = 0;
                    setPathState(24);
                }
                break;
            case 24:
                TragereLaPupitru();
                if (!TragereInProgres || pathTimer.getElapsedTimeSeconds() >= SHOOT_CYCLE_TIMEOUT) {
                    if (TragereInProgres) finishShootingCycle();
                    setPathState(25);
                }
                break;

            // ========== PARK ==========
            case 25:
                n.shooter.setVelocity(0);
                n.shooter2.setVelocity(0);
                intakePornit = false;
                if (TrackingT != null) TrackingT.interrupt();
                n.turelaD.setPosition(0.5);
                n.turelaS.setPosition(0.5);
                follower.followPath(returnarea, false);
                setPathState(26);
                break;
            case 26:
                n.turelaD.setPosition(0.5);
                n.turelaS.setPosition(0.5);
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= PATH_TIMEOUT) {
                    setPathState(-1);
                }
                break;

            // ========== EMERGENCY PARK (from time skip) ==========
            case 28:
                n.shooter.setVelocity(0);
                n.shooter2.setVelocity(0);
                intakePornit = false;
                if (TrackingT != null) TrackingT.interrupt();
                n.turelaD.setPosition(0.5);
                n.turelaS.setPosition(0.5);
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= PATH_TIMEOUT) {
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

    // =====================================================================
    // OPMODE LIFECYCLE
    // =====================================================================
    @Override
    public void loop() {
        if (opmodeTimer.getElapsedTimeSeconds() >= EMERGENCY_STOP) {
            fullShutdown();
            stop = true;
            requestOpModeStop();
            return;
        }

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        follower.update();
        autonomousPathUpdate();

        Pose p = follower.getPose();
        telemetry.addData("S", "%d|%d", pathState, ShootingStare);
        telemetry.addData("T", "%.1f", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("B", "%d/3", getLoculete());
        telemetry.addData("P", "%.1f, %.1f, %.0f", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.addData("Q", "%d/%d", shotsGood, shotsGood + shotsTimeout);
        telemetry.update();
    }

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        n.initsisteme(hardwareMap);
        n.limelight.pipelineSwitch(0);
        n.bascula.setPosition(Pozitii.sede);
        n.sortare.setPosition(Pozitii.luarea1);

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
        telemetry.addData("Voltage", "%.2fV", n.voltageSensor.getVoltage());
        GoBildaPinpointDriver pp = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        telemetry.addData("Pinpoint", pp.getDeviceStatus().toString());
        telemetry.addData("LL", n.limelight.isConnected() ? "OK" : "DISCONNECTED");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        TragereInProgres = false;
        ShootingStare = 0;
        stop = false;
        intakePornit = false;
        shotsGood = 0;
        shotsTimeout = 0;

        fillSlots();

        n.limelight.pipelineSwitch(0);
        n.resetTurelaPID();

        // Shooterele pornesc ACUM si raman ON pana la parcare
        n.applyVoltageCompensatedPIDF();
        n.shooter.setVelocity(VELOCITY_TARGET);
        n.shooter2.setVelocity(VELOCITY_TARGET);

        createIntakeThread();
        Intake.start();
        startTracking();
    }

    @Override
    public void stop() {
        stop = true;

        if (TrackingT != null) TrackingT.interrupt();
        if (Intake != null) Intake.interrupt();

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
