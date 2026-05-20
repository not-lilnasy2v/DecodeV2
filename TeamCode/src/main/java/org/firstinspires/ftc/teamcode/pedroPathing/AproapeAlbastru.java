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
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Pozitii;
import org.firstinspires.ftc.teamcode.RobotPozitie;
import org.firstinspires.ftc.teamcode.pop;
import org.firstinspires.ftc.teamcode.sistemeAuto;

@Autonomous(name = "Departe Albastru")
public class AproapeAlbastru extends OpMode {
    sistemeAuto n = new sistemeAuto();
    public Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private static final double TARGET_X = 0;
    private static final double TARGET_Y = 144;


    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose shootingPose = new Pose(59.39160839160837, 24.083916083916087, Math.toRadians(90));

    private final Pose colectare = new Pose(14.34965034965035, 35.34965034965035, Math.toRadians(180));
    private final Pose controlColectare = new Pose(64.3618068628121, 37.29370629370628);
    private final Pose tragere2 = new Pose(60.54545454545453, 24.020979020979027, Math.toRadians(90));
    private final Pose controlTragere = new Pose(41.59790209790209, 18.96153846153846);

    private final Pose luarePose = new Pose(14.357499479925096, 9.710434302539593, Math.toRadians(180));
    private final Pose controlLuare = new Pose(14.247139588100687, 9.585100254436647);
    private final Pose tragePose = new Pose(59.55944055944056, 24.36363636363636, Math.toRadians(90));

    private final Pose colectare2 = new Pose(14.453986162090423, 59.31457253162709, Math.toRadians(180));
    private final Pose controlColectare2 = new Pose(64.77244562621172, 61.433784081499006);

    private final Pose tras2Pose = new Pose(59.639741941439155, 23.587535561238276, Math.toRadians(90));
    private final Pose colectare3 = new Pose(14.42869344435154, 82.5957336702663, Math.toRadians(180));
    private final Pose controlColectare3 = new Pose(71.10916048465737, 86.87138289950515);
    private final Pose tras3Pose = new Pose(60.16148694204216, 23.860566871317477, Math.toRadians(90));

    private final Pose iesirea = new Pose(34.71226672976966, 10.97982064609837, Math.toRadians(90));

    private Path laShooting;
    private PathChain iale, tragere, luarePath, tragePath, laColectare2, tras2Path, laColectare3, tras3Path, iesireas;

    private volatile boolean TragereInProgres = false;

    private int ShootingStare = 0;
    private int currentShootSlot = 2;
    private int ballTras = 0;
    private int flushSlot = 2;
    private int flushRound = 0;
    private long velocityCheckStart = 0;

    private final boolean[] slotOcupat = new boolean[3];
    private final int[] slotColor = new int[3];
    private volatile boolean intakePornit = false;
    private volatile boolean stop = false;
    private Thread IntakeThread;
    private Thread TrackingThread;
    private final Object slot = new Object();

    private int idTag = 0;
    private final int[] cPattern = new int[3];
    private boolean Pattern = false;

    private volatile boolean scanareTerminata = false;
    private Thread scanThread;


    private int getLoculete() {
        synchronized (slot) {
            int count = 0;
            for (boolean occupied : slotOcupat) {
                if (occupied) count++;
            }
            return count;
        }
    }

    private void resetSlots() {
        synchronized (slot) {
            for (int i = 0; i < 3; i++) {
                slotOcupat[i] = false;
                slotColor[i] = -1;
            }
        }
    }

    private boolean hasTimeFor(double seconds) {
        return opmodeTimer.getElapsedTimeSeconds() + seconds < 29.0;
    }

    private double getLuarePos(int slot) {
        if (slot == 0) return Pozitii.luarea1;
        if (slot == 1) return Pozitii.luarea2;
        return Pozitii.luarea3;
    }

    private double getAruncarePos(int slot) {
        if (slot == 0) return Pozitii.aruncare1;
        if (slot == 1) return Pozitii.aruncare2;
        return Pozitii.aruncare3;
    }


    public void buildPaths() {
        laShooting = new Path(new BezierLine(startPose, shootingPose));
        laShooting.setConstantHeadingInterpolation(startPose.getHeading());

        iale = follower.pathBuilder()
                .addPath(new BezierCurve(shootingPose, controlColectare, colectare))
                .setTangentHeadingInterpolation()
                .build();

        tragere = follower.pathBuilder()
                .addPath(new BezierCurve(colectare, controlTragere, tragere2))
                .setLinearHeadingInterpolation(colectare.getHeading(), tragere2.getHeading())
                .setHeadingConstraint(2)
                .setBrakingStrength(1.0)
                .setTimeoutConstraint(50)
                .build();

        luarePath = follower.pathBuilder()
                .addPath(new BezierCurve(tragere2, controlLuare, luarePose))
                .setLinearHeadingInterpolation(tragere2.getHeading(), luarePose.getHeading())
                .setHeadingConstraint(2)
                .setTimeoutConstraint(50)
                .build();

        tragePath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        luarePose,
                        new Pose(21.85329875441761, 12.140362282520464),
                        new Pose(25.941916540504437, 13.465777544328212),
                        new Pose(30.257679759151642, 14.8648269873475)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(157))
                .addPath(new BezierCurve(
                        new Pose(30.257679759151642, 14.8648269873475),
                        new Pose(34.34629754523847, 16.190242249155247),
                        new Pose(38.66206076388567, 17.589291692174537),
                        new Pose(42.7506785499725, 18.914706953982286)))
                .setLinearHeadingInterpolation(Math.toRadians(157), Math.toRadians(135))
                .addPath(new BezierCurve(
                        new Pose(42.7506785499725, 18.914706953982286),
                        new Pose(46.83929633605933, 20.240122215790034),
                        new Pose(51.155059554706526, 21.63917165880932),
                        new Pose(55.243677340793354, 22.964586920617073)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(112))
                .addPath(new BezierLine(
                        new Pose(55.243677340793354, 22.964586920617073),
                        tragePose))
                .setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(90))
                .setHeadingConstraint(2)
                .setBrakingStrength(1.0)
                .setTimeoutConstraint(50)
                .build();

        laColectare2 = follower.pathBuilder()
                .addPath(new BezierCurve(tragePose, controlColectare2, colectare2))
                .setTangentHeadingInterpolation()
                .build();

        tras2Path = follower.pathBuilder()
                .addPath(new BezierLine(colectare2, tras2Pose))
                .setLinearHeadingInterpolation(colectare2.getHeading(), tras2Pose.getHeading())
                .setHeadingConstraint(2)
                .setBrakingStrength(1.0)
                .setTimeoutConstraint(50)
                .build();

        laColectare3 = follower.pathBuilder()
                .addPath(new BezierCurve(tras2Pose, controlColectare3, colectare3))
                .setTangentHeadingInterpolation()
                .build();

        tras3Path = follower.pathBuilder()
                .addPath(new BezierLine(colectare3, tras3Pose))
                .setLinearHeadingInterpolation(colectare3.getHeading(), tras3Pose.getHeading())
                .setHeadingConstraint(2)
                .setBrakingStrength(1.0)
                .setTimeoutConstraint(50)
                .build();

        iesireas = follower.pathBuilder()
                .addPath(new BezierLine(tras3Pose, iesirea))
                .setLinearHeadingInterpolation(tras3Pose.getHeading(), iesirea.getHeading())
                .build();
    }


    private boolean toateBileleDetectate() {
        synchronized (slot) {
            for (int i = 0; i < 3; i++) {
                if (slotOcupat[i] && slotColor[i] == -1) return false;
            }
            return true;
        }
    }

    private void startScaneazaPreloadAsync() {
        scanareTerminata = false;
        scanThread = new Thread(() -> {
            int maxAttempts = 5;
            int attempt = 0;

            while (!toateBileleDetectate() && attempt < maxAttempts && !stop) {
                attempt++;
                double lastPos = n.sortare.getPosition();

                for (int i = 0; i < 3; i++) {
                    if (stop) return;
                    boolean needsScan;
                    synchronized (slot) {
                        needsScan = slotOcupat[i] && slotColor[i] == -1;
                    }
                    if (needsScan) {
                        double targetPos = getLuarePos(i);
                        n.sortare.setPosition(targetPos);

                        double dist = Math.abs(targetPos - lastPos);
                        int moveWait = (int)(dist * 550) + 150;
                        n.kdf(moveWait);

                        for (int retry = 0; retry < 3; retry++) {
                            synchronized (slot) {
                                if (slotColor[i] != -1) break;
                            }
                            n.resetareDetection();
                            int detected = n.detecteazaBiloaca();
                            synchronized (slot) {
                                slotColor[i] = detected;
                            }
                            if (detected == -1) n.kdf(80);
                        }
                        lastPos = targetPos;
                    }
                }
            }

            if (Pattern) {
                int firstSlot = gasesteBilaCuCuloare(cPattern[0]);
                n.sortare.setPosition(firstSlot != -1 ? getAruncarePos(firstSlot) : Pozitii.aruncare3);
            } else {
                n.sortare.setPosition(Pozitii.aruncare3);
            }

            scanareTerminata = true;
        });
        scanThread.start();
    }

    private void detecteazaPattern() {
        idTag = n.detectIdTag();
        if (idTag == 23) {
            cPattern[0] = 1; cPattern[1] = 1; cPattern[2] = 0;
            Pattern = true;
        } else if (idTag == 22) {
            cPattern[0] = 1; cPattern[1] = 0; cPattern[2] = 1;
            Pattern = true;
        } else if (idTag == 21) {
            cPattern[0] = 0; cPattern[1] = 1; cPattern[2] = 1;
            Pattern = true;
        } else {
            cPattern[0] = -1; cPattern[1] = -1; cPattern[2] = -1;
            Pattern = false;
        }
    }

    private int gasesteBilaCuCuloare(int culoareNecesara) {
        synchronized (slot) {
            if (culoareNecesara == -1) {
                for (int i = 0; i < 3; i++) {
                    if (slotOcupat[i]) return i;
                }
                return -1;
            }
            for (int i = 0; i < 3; i++) {
                if (slotOcupat[i] && slotColor[i] == culoareNecesara) return i;
            }
            for (int i = 0; i < 3; i++) {
                if (slotOcupat[i]) return i;
            }
            return -1;
        }
    }


    private static final double SHOOTER_VEL = 1850;
    private static final double IDLE_RATIO = 0.67;

    private void TragereLaPupitru() {
        switch (ShootingStare) {
            case 0:
                n.applyVoltageCompensatedPIDF();
                n.shooter.setVelocity(SHOOTER_VEL);
                n.shooter2.setVelocity(SHOOTER_VEL);
                n.unghiD.setPosition(pop.posUnghi);
                n.scula.setPower(-1);
                n.bascula.setPosition(Pozitii.sede);
                actionTimer.resetTimer();
                ShootingStare = 1;
                break;

            case 1:
                if (actionTimer.getElapsedTimeSeconds() >= 0.35) {
                    actionTimer.resetTimer();
                    ShootingStare = 2;
                }
                break;

            case 2:
                if (actionTimer.getElapsedTimeSeconds() >= 1.0) {
                    ShootingStare = 3;
                }
                break;

            case 3:
                ballTras = getLoculete();
                if (ballTras > 0) {
                    if (Pattern) {
                        int patternIndex = Math.max(0, Math.min(2, 3 - ballTras));
                        currentShootSlot = gasesteBilaCuCuloare(cPattern[patternIndex]);
                    } else {
                        currentShootSlot = 2;
                        synchronized (slot) {
                            while (currentShootSlot >= 0 && !slotOcupat[currentShootSlot]) {
                                currentShootSlot--;
                            }
                        }
                    }
                    if (currentShootSlot >= 0) {
                        ShootingStare = 4;
                    } else {
                        flushSlot = 2;
                        flushRound = 0;
                        n.bascula.setPosition(Pozitii.lansare);
                        n.intake.setPower(1);
                        ShootingStare = 11;
                    }
                } else {
                    flushSlot = 2;
                    flushRound = 0;
                    n.bascula.setPosition(Pozitii.lansare);
                    n.intake.setPower(1);
                    ShootingStare = 11;
                }
                break;

            case 4:
                n.sortare.setPosition(getAruncarePos(currentShootSlot));
                actionTimer.resetTimer();
                ShootingStare = 5;
                break;

            case 5:
                if (actionTimer.getElapsedTimeSeconds() >= 0.20) {
                    velocityCheckStart = System.currentTimeMillis();
                    ShootingStare = 6;
                }
                break;

            case 6:
                double v1 = Math.abs(n.shooter.getVelocity());
                double v2 = Math.abs(n.shooter2.getVelocity());
                double tol = SHOOTER_VEL * 0.03;
                if ((Math.abs(v1 - SHOOTER_VEL) < tol && Math.abs(v2 - SHOOTER_VEL) < tol)
                        || (System.currentTimeMillis() - velocityCheckStart) > 500) {
                    ShootingStare = 7;
                }
                break;

            case 7:
                n.bascula.setPosition(Pozitii.lansare);
                actionTimer.resetTimer();
                ShootingStare = 8;
                break;

            case 8:
                if (actionTimer.getElapsedTimeSeconds() >= 0.20) {
                    ShootingStare = 9;
                }
                break;

            case 9:
                n.bascula.setPosition(Pozitii.sede);
                actionTimer.resetTimer();
                ShootingStare = 10;
                break;

            case 10:
                if (actionTimer.getElapsedTimeSeconds() >= 0.10) {
                    synchronized (slot) {
                        slotOcupat[currentShootSlot] = false;
                        slotColor[currentShootSlot] = -1;
                    }
                    ballTras--;
                    ShootingStare = 3;
                }
                break;

            case 11:
                double flushTarget;
                if (flushSlot == 0) flushTarget = Pozitii.aruncare1;
                else if (flushSlot == 1) flushTarget = Pozitii.aruncare2;
                else flushTarget = Pozitii.aruncare3;
                n.sortare.setPosition(flushTarget);
                actionTimer.resetTimer();
                ShootingStare = 12;
                break;

            case 12:
                if (actionTimer.getElapsedTimeSeconds() >= 0.25) {
                    ShootingStare = 13;
                }
                break;

            case 13:
                n.bascula.setPosition(Pozitii.lansare);
                actionTimer.resetTimer();
                ShootingStare = 14;
                break;

            case 14:
                if (actionTimer.getElapsedTimeSeconds() >= 0.15) {
                    ShootingStare = 15;
                }
                break;

            case 15:
                n.bascula.setPosition(Pozitii.sede);
                actionTimer.resetTimer();
                ShootingStare = 16;
                break;

            case 16:
                if (actionTimer.getElapsedTimeSeconds() >= 0.10) {
                    flushSlot--;
                    if (flushSlot >= 0) {
                        ShootingStare = 11;
                    } else {
                        actionTimer.resetTimer();
                        ShootingStare = 17;
                    }
                }
                break;

            case 17:
                if (actionTimer.getElapsedTimeSeconds() >= 0.3) {
                    double dist = n.cachedDistanta;
                    if (dist < 20 && flushRound < 2) {
                        flushRound++;
                        flushSlot = 2;
                        ShootingStare = 11;
                    } else {
                        ShootingStare = 18;
                    }
                }
                break;

            case 18:
                n.bascula.setPosition(Pozitii.sede);
                n.scula.setPower(0);
                n.intake.setPower(0);
                n.sortare.setPosition(Pozitii.luarea1);
                actionTimer.resetTimer();
                ShootingStare = 19;
                break;

            case 19:
                if (actionTimer.getElapsedTimeSeconds() >= 0.3) {
                    n.shooter.setVelocity(0);
                    n.shooter2.setVelocity(0);
                    ShootingStare = 20;
                }
                break;

            case 20:
                resetSlots();
                currentShootSlot = 2;
                TragereInProgres = false;
                ShootingStare = 0;
                break;
        }
    }


    private void startTracking() {
        TrackingThread = new Thread(() -> {
            while (!stop) {
                n.Nkdf(2000000);
                n.tracks(follower, TARGET_X, TARGET_Y);
            }
            n.turelaD.setPosition(0.5);
            n.turelaS.setPosition(0.5);
        });
        TrackingThread.start();
    }

    private void Intake() {
        IntakeThread = new Thread(new Runnable() {
            private boolean ballBeingProcessed = false;
            private long lastDistReadTime = 0;

            @Override
            public void run() {
                while (!stop) {
                    n.Nkdf(3000000);
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

                            synchronized (slot) {
                                if (Math.abs(servoPos - Pozitii.luarea1) < 0.1 && !slotOcupat[0]) {
                                    slotOcupat[0] = true;
                                    if (!slotOcupat[1]) n.sortare.setPosition(Pozitii.luarea2);
                                    else if (!slotOcupat[2]) n.sortare.setPosition(Pozitii.luarea3);
                                } else if (Math.abs(servoPos - Pozitii.luarea2) < 0.1 && !slotOcupat[1]) {
                                    slotOcupat[1] = true;
                                    if (!slotOcupat[2]) n.sortare.setPosition(Pozitii.luarea3);
                                    else if (!slotOcupat[0]) n.sortare.setPosition(Pozitii.luarea1);
                                } else if (Math.abs(servoPos - Pozitii.luarea3) < 0.1 && !slotOcupat[2]) {
                                    slotOcupat[2] = true;
                                    if (!slotOcupat[0]) n.sortare.setPosition(Pozitii.luarea1);
                                    else if (!slotOcupat[1]) n.sortare.setPosition(Pozitii.luarea2);
                                }
                            }
                            n.kdf(150);

                        } else if (leDistanta >= 20 && ballBeingProcessed) {
                            ballBeingProcessed = false;
                        }
                    } else if (!intakePornit) {
                        n.intake.setPower(0);
                        ballBeingProcessed = false;
                    }

                    if (!TragereInProgres) {
                        if (loculete == 3) {
                            n.shooter.setVelocity(SHOOTER_VEL);
                            n.shooter2.setVelocity(SHOOTER_VEL);
                        } else if (loculete > 0) {
                            n.shooter.setVelocity(SHOOTER_VEL * IDLE_RATIO);
                            n.shooter2.setVelocity(SHOOTER_VEL * IDLE_RATIO);
                        } else {
                            n.shooter.setVelocity(0);
                            n.shooter2.setVelocity(0);
                        }
                    }
                }
            }
        });
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                detecteazaPattern();
                follower.followPath(laShooting);
                startScaneazaPreloadAsync();
                setPathState(1);
                break;

            case 1:
                if ((!follower.isBusy() && scanareTerminata) || pathTimer.getElapsedTimeSeconds() > 4.0) {
                    follower.holdPoint(shootingPose);
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
                    setPathState(100);
                }
                break;

            case 100:
                resetSlots();
                n.sortare.setPosition(Pozitii.luarea1);
                actionTimer.resetTimer();
                setPathState(101);
                break;

            case 101:
                if (actionTimer.getElapsedTimeSeconds() >= 0.1) {
                    intakePornit = true;
                    follower.followPath(iale, 0.7, false);
                    setPathState(102);
                }
                break;

            case 102:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 5.0) {
                    follower.holdPoint(colectare);
                    actionTimer.resetTimer();
                    setPathState(103);
                }
                break;

            case 103:
                if (getLoculete() >= 3) {
                    actionTimer.resetTimer();
                    setPathState(104);
                } else if (actionTimer.getElapsedTimeSeconds() >= 3.0) {
                    intakePornit = false;
                    if (getLoculete() > 0) {
                        setPathState(105);
                    } else {
                        goToExit();
                    }
                }
                break;

            case 104:
                if (actionTimer.getElapsedTimeSeconds() >= 0.3) {
                    intakePornit = false;
                    setPathState(105);
                }
                break;

            case 105:
                follower.followPath(tragere, 0.7, false);
                startScaneazaPreloadAsync();
                setPathState(106);
                break;

            case 106:
                if ((!follower.isBusy() && scanareTerminata) || pathTimer.getElapsedTimeSeconds() > 5.0) {
                    follower.holdPoint(tragere2);
                    setPathState(107);
                }
                break;

            case 107:
                if (!TragereInProgres) {
                    TragereInProgres = true;
                    ShootingStare = 0;
                }
                TragereLaPupitru();
                if (!TragereInProgres) {
                    if (hasTimeFor(10.0)) {
                        setPathState(200);
                    } else {
                        goToExit();
                    }
                }
                break;

            case 200:
                resetSlots();
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(luarePath);
                setPathState(201);
                break;

            case 201:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 5.0) {
                    follower.holdPoint(luarePose);
                    actionTimer.resetTimer();
                    setPathState(202);
                }
                break;

            case 202:
                if (getLoculete() >= 3) {
                    actionTimer.resetTimer();
                    setPathState(203);
                } else if (actionTimer.getElapsedTimeSeconds() >= 3.0) {
                    intakePornit = false;
                    if (getLoculete() > 0) {
                        setPathState(204);
                    } else {
                        goToExit();
                    }
                }
                break;

            case 203:
                if (actionTimer.getElapsedTimeSeconds() >= 0.3) {
                    intakePornit = false;
                    setPathState(204);
                }
                break;

            case 204:
                follower.followPath(tragePath);
                startScaneazaPreloadAsync();
                setPathState(205);
                break;

            case 205:
                if ((!follower.isBusy() && scanareTerminata) || pathTimer.getElapsedTimeSeconds() > 6.0) {
                    follower.holdPoint(tragePose);
                    setPathState(206);
                }
                break;

            case 206:
                if (!TragereInProgres) {
                    TragereInProgres = true;
                    ShootingStare = 0;
                }
                TragereLaPupitru();
                if (!TragereInProgres) {
                    if (hasTimeFor(10.0)) {
                        setPathState(300);
                    } else {
                        goToExit();
                    }
                }
                break;

            case 300:
                resetSlots();
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(laColectare2);
                setPathState(301);
                break;

            case 301:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 5.0) {
                    follower.holdPoint(colectare2);
                    actionTimer.resetTimer();
                    setPathState(302);
                }
                break;

            case 302:
                if (getLoculete() >= 3) {
                    actionTimer.resetTimer();
                    setPathState(303);
                } else if (actionTimer.getElapsedTimeSeconds() >= 3.0) {
                    intakePornit = false;
                    if (getLoculete() > 0) {
                        setPathState(304);
                    } else {
                        goToExit();
                    }
                }
                break;

            case 303:
                if (actionTimer.getElapsedTimeSeconds() >= 0.3) {
                    intakePornit = false;
                    setPathState(304);
                }
                break;

            case 304:
                follower.followPath(tras2Path);
                startScaneazaPreloadAsync();
                setPathState(305);
                break;

            case 305:
                if ((!follower.isBusy() && scanareTerminata) || pathTimer.getElapsedTimeSeconds() > 5.0) {
                    follower.holdPoint(tras2Pose);
                    setPathState(306);
                }
                break;

            case 306:
                if (!TragereInProgres) {
                    TragereInProgres = true;
                    ShootingStare = 0;
                }
                TragereLaPupitru();
                if (!TragereInProgres) {
                    if (hasTimeFor(10.0)) {
                        setPathState(400);
                    } else {
                        goToExit();
                    }
                }
                break;

            case 400:
                resetSlots();
                n.sortare.setPosition(Pozitii.luarea1);
                intakePornit = true;
                follower.followPath(laColectare3);
                setPathState(401);
                break;

            case 401:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 5.0) {
                    follower.holdPoint(colectare3);
                    actionTimer.resetTimer();
                    setPathState(402);
                }
                break;

            case 402:
                if (getLoculete() >= 3) {
                    actionTimer.resetTimer();
                    setPathState(403);
                } else if (actionTimer.getElapsedTimeSeconds() >= 3.0) {
                    intakePornit = false;
                    if (getLoculete() > 0) {
                        setPathState(404);
                    } else {
                        goToExit();
                    }
                }
                break;

            case 403:
                if (actionTimer.getElapsedTimeSeconds() >= 0.3) {
                    intakePornit = false;
                    setPathState(404);
                }
                break;

            case 404:
                follower.followPath(tras3Path);
                startScaneazaPreloadAsync();
                setPathState(405);
                break;

            case 405:
                if ((!follower.isBusy() && scanareTerminata) || pathTimer.getElapsedTimeSeconds() > 5.0) {
                    follower.holdPoint(tras3Pose);
                    setPathState(406);
                }
                break;

            case 406:
                if (!TragereInProgres) {
                    TragereInProgres = true;
                    ShootingStare = 0;
                }
                TragereLaPupitru();
                if (!TragereInProgres) {
                    goToExit();
                }
                break;

            case 999:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4.0) {
                    setPathState(-1);
                }
                break;

            default:
                break;
        }
    }

    private void goToExit() {
        intakePornit = false;
        n.shooter.setVelocity(0);
        n.shooter2.setVelocity(0);
        Pose cur = follower.getPose();
        if (cur != null) {
            PathChain exitPath = follower.pathBuilder()
                    .addPath(new BezierLine(cur, iesirea))
                    .setLinearHeadingInterpolation(cur.getHeading(), iesirea.getHeading())
                    .build();
            follower.followPath(exitPath);
        } else {
            follower.followPath(iesireas);
        }
        setPathState(999);
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
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
        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Shoot State", ShootingStare);
        telemetry.addData("Bile", getLoculete());
        telemetry.addData("Timp", "%.1fs", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addLine("");
        for (int i = 0; i < 3; i++) {
            String status;
            synchronized (slot) {
                if (!slotOcupat[i]) status = "gol";
                else if (slotColor[i] == 0) status = "VERDE";
                else if (slotColor[i] == 1) status = "MOV";
                else status = "?";
            }
            telemetry.addData("Slot " + (i + 1), status);
        }
        telemetry.addLine("");
        telemetry.addData("Pattern ID", idTag);
        telemetry.addData("Pattern", Pattern);
        telemetry.addData("LL tx", "%.2f", n.getLimelightTx());
        telemetry.addData("Tag", n.isTagVisible() ? "DA" : "NU");
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

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.recalibrateIMU();
        ElapsedTime calibTimer = new ElapsedTime();
        while (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                && calibTimer.milliseconds() < 2000) {
            n.Nkdf(10000000);
        }

        buildPaths();
        follower.setStartingPose(startPose);
        follower.update();

        for (int i = 0; i < 3; i++) slotColor[i] = -1;

        n.limelight.pipelineSwitch(1);
    }

    @Override
    public void init_loop() {
        idTag = n.detectIdTag();
        telemetry.addData("Limelight ID", idTag);
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        TragereInProgres = false;
        stop = false;
        intakePornit = false;
        currentShootSlot = 2;
        scanareTerminata = false;

        synchronized (slot) {
            slotOcupat[0] = true;
            slotOcupat[1] = true;
            slotOcupat[2] = true;
            slotColor[0] = -1;
            slotColor[1] = -1;
            slotColor[2] = -1;
        }

        n.sortare.setPosition(Pozitii.luarea1);
        n.limelight.pipelineSwitch(0);
        n.resetTurelaPID();

        Intake();
        IntakeThread.start();
        startTracking();
    }

    @Override
    public void stop() {
        stop = true;
        intakePornit = false;

        if (scanThread != null) scanThread.interrupt();
        if (TrackingThread != null) TrackingThread.interrupt();
        if (IntakeThread != null) IntakeThread.interrupt();

        try { if (scanThread != null) scanThread.join(200); } catch (InterruptedException ignored) {}
        try { if (TrackingThread != null) TrackingThread.join(200); } catch (InterruptedException ignored) {}
        try { if (IntakeThread != null) IntakeThread.join(200); } catch (InterruptedException ignored) {}

        Pose currentPose = follower.getPose();
        if (currentPose != null) {
            RobotPozitie.X = currentPose.getX();
            RobotPozitie.Y = currentPose.getY();
            RobotPozitie.heading = currentPose.getHeading();
        }
        RobotPozitie.idTag = idTag;

        n.shooter.setVelocity(0);
        n.shooter2.setVelocity(0);
        n.intake.setPower(0);
        n.scula.setPower(0);
        n.turelaD.setPosition(0.5);
        n.turelaS.setPosition(0.5);
        n.bascula.setPosition(0.5807);
    }
}
