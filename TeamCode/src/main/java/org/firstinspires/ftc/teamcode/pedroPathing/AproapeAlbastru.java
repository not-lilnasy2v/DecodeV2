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
    public Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private static final double TARGET_X = 0;
    private static final double TARGET_Y = 127;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose shootingPose = new Pose(59.39160839160837, 24.083916083916087, Math.toRadians(90));

    private final Pose colectare = new Pose(14.34965034965035, 35.34965034965035, Math.toRadians(180));
    private final Pose controlPoint = new Pose(64.36713286713285, 37.29370629370628);
    private final Pose tragere2 = new Pose(60.54545454545453, 24.020979020979027, Math.toRadians(90));
    private final Pose controlTragere = new Pose(41.59790209790209, 18.96153846153846, Math.toRadians(90));
    private final Pose luare3 = new Pose(12.09090909090908,35.77622377622379,Math.toRadians(250));
    private final Pose aluat3 = new Pose(9.223776223776227,13.006993006993023, Math.toRadians(250));
    private final Pose tras3 = new Pose(59.55944055944056,24.36363636363636, Math.toRadians(90));
    private final Pose iesirea = new Pose(34.71226672976966, 10.97982064609837, Math.toRadians(90));

    private Path laShooting;
    private PathChain iale, tragere, iesireas,treiluat,treialuat,treiTras;

    private boolean TragereInProgres = false;
    private int ShootingStare = 0;
    private int currentShootSlot = 2;
    private int ballTras = 0;
    private int flushSlot = 2;
    private int flushRound = 0;
    private long velocityCheckStart = 0;

    private volatile boolean[] slotOcupat = new boolean[3];
    private volatile int[] slotColor = new int[3];  // 0=verde, 1=mov, -1=necunoscut
    private volatile boolean intakePornit = false;
    private volatile boolean stop = false;
    private Thread IntakeThread;
    private volatile int IntakeSlot = 0;
    private final Object slot = new Object();

    private int idTag = 0;
    private int[] cPattern = new int[3];
    private boolean Pattern = false;

    // Pentru scanare paralela
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

    private int getPrimulSlotLiber() {
        synchronized (slot) {
            for (int i = 0; i < 3; i++) {
                if (!slotOcupat[i]) return i;
            }
            return -1;
        }
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
                .addPath(new BezierCurve(shootingPose, controlPoint, colectare))
                .setTangentHeadingInterpolation()
                .build();
        tragere = follower.pathBuilder()
                .addPath(new BezierCurve(colectare, controlTragere, tragere2))
                .setLinearHeadingInterpolation(colectare.getHeading(), tragere2.getHeading())
                .setHeadingConstraint(2)
                .setBrakingStrength(1.0)
                .setTimeoutConstraint(50)
                .build();
        treiluat= follower.pathBuilder()
                .addPath(new BezierLine(tragere2,luare3))
                .setLinearHeadingInterpolation(tragere2.getHeading(),luare3.getHeading())
                .setHeadingConstraint(2)
                .setTimeoutConstraint(50)
                .build();
        treialuat = follower.pathBuilder()
                .addPath(new BezierLine(luare3,aluat3))
                .setLinearHeadingInterpolation(luare3.getHeading(),aluat3.getHeading())
                .setHeadingConstraint(2)
                .setTimeoutConstraint(50)
                .build();
        treiTras = follower.pathBuilder()
                .addPath(new BezierLine(aluat3,tras3))
                .setLinearHeadingInterpolation(aluat3.getHeading(),tras3.getHeading())
                .setHeadingConstraint(2)
                .setBrakingStrength(1.0)
                .setTimeoutConstraint(50)
                .build();
        iesireas = follower.pathBuilder()
                .addPath(new BezierLine(tras3, iesirea))
                .setLinearHeadingInterpolation(tras3.getHeading(), iesirea.getHeading())
                .build();
    }

    private boolean toateBileleDetectate() {
        for (int i = 0; i < 3; i++) {
            if (slotOcupat[i] && slotColor[i] == -1) {
                return false;
            }
        }
        return true;
    }

    // Porneste scanarea in paralel cu mersul
    private void startScaneazaPreloadAsync() {
        scanareTerminata = false;
        scanThread = new Thread(() -> {
            int Incercari = 5;
            int attempt = 0;

            while (!toateBileleDetectate() && attempt < Incercari) {
                attempt++;
                double lastPos = n.sortare.getPosition();

                for (int i = 0; i < 3; i++) {
                    if (slotOcupat[i] && slotColor[i] == -1) {
                        double targetPos = getLuarePos(i);

                        n.sortare.setPosition(targetPos);

                        double dist = Math.abs(targetPos - lastPos);
                        int moveWait = (int)(dist * 550) + 150;
                        n.kdf(moveWait);

                        for (int retry = 0; retry < 3 && slotColor[i] == -1; retry++) {
                            n.resetareDetection();
                            slotColor[i] = n.detecteazaBiloaca();
                            if (slotColor[i] == -1) {
                                n.kdf(80);
                            }
                        }

                        lastPos = targetPos;
                    }
                }
            }

            // Pune sortare la pozitia de aruncare
            if (Pattern) {
                int firstColor = cPattern[0];
                int firstSlot = gasesteBilaCuCuloare(firstColor);
                if (firstSlot != -1) {
                    n.sortare.setPosition(getAruncarePos(firstSlot));
                } else {
                    n.sortare.setPosition(Pozitii.aruncare3);
                }
            } else {
                n.sortare.setPosition(Pozitii.aruncare3);
            }

            scanareTerminata = true;
        });
        scanThread.start();
    }

    private void verificaSloturiNecunoscute() {
        int Incercari = 5;
        int attempt = 0;

        while (!toateBileleDetectate() && attempt < Incercari) {
            attempt++;
            double lastPos = n.sortare.getPosition();

            for (int i = 0; i < 3; i++) {
                if (slotOcupat[i] && slotColor[i] == -1) {
                    double pozitie = getLuarePos(i);

                    n.sortare.setPosition(pozitie);

                    double dist = Math.abs(pozitie - lastPos);
                    int asteapta = (int)(dist * 550) + 150;
                    n.kdf(asteapta);

                    for (int retry = 0; retry < 3 && slotColor[i] == -1; retry++) {
                        n.resetareDetection();
                        slotColor[i] = n.detecteazaBiloaca();
                        if (slotColor[i] == -1) {
                            n.kdf(80);
                        }
                    }

                    lastPos = pozitie;
                }
            }
        }

        n.sortare.setPosition(Pozitii.luarea1);
    }

    private void scaneazaPreload() {
        int Incercari = 5;
        int attempt = 0;

        while (!toateBileleDetectate() && attempt < Incercari) {
            attempt++;
            double lastPos = n.sortare.getPosition();

            for (int i = 0; i < 3; i++) {
                if (slotOcupat[i] && slotColor[i] == -1) {
                    double targetPos = getLuarePos(i);

                    n.sortare.setPosition(targetPos);

                    double dist = Math.abs(targetPos - lastPos);
                    int moveWait = (int)(dist * 550) + 150;
                    n.kdf(moveWait);

                    for (int retry = 0; retry < 3 && slotColor[i] == -1; retry++) {
                        n.resetareDetection();
                        slotColor[i] = n.detecteazaBiloaca();
                        if (slotColor[i] == -1) {
                            n.kdf(80);
                        }
                    }

                    lastPos = targetPos;
                }
            }
        }

        if (Pattern) {
            int firstColor = cPattern[0];
            int firstSlot = gasesteBilaCuCuloare(firstColor);
            if (firstSlot != -1) {
                n.sortare.setPosition(getAruncarePos(firstSlot));
            } else {
                n.sortare.setPosition(Pozitii.aruncare3);
            }
        } else {
            n.sortare.setPosition(Pozitii.aruncare3);
        }
        n.kdf(200);
    }

    private void detecteazaPattern() {
        idTag = n.detectIdTag();

        if (idTag == 23) {
            cPattern[0] = 1; cPattern[1] = 1; cPattern[2] = 0;  // mov, mov, verde
            Pattern = true;
        } else if (idTag == 22) {
            cPattern[0] = 1; cPattern[1] = 0; cPattern[2] = 1;  // mov, verde, mov
            Pattern = true;
        } else if (idTag == 21) {
            cPattern[0] = 0; cPattern[1] = 1; cPattern[2] = 1;  // verde, mov, mov
            Pattern = true;
        } else {
            cPattern[0] = -1; cPattern[1] = -1; cPattern[2] = -1;
            Pattern = false;
        }
    }

    private int gasesteBilaCuCuloare(int culoareNecesara) {
        if (culoareNecesara == -1) {
            for (int i = 0; i < 3; i++) {
                if (slotOcupat[i]) return i;
            }
            return -1;
        }

        for (int i = 0; i < 3; i++) {
            if (slotOcupat[i] && slotColor[i] == culoareNecesara) {
                return i;
            }
        }

        for (int i = 0; i < 3; i++) {
            if (slotOcupat[i]) return i;
        }
        return -1;
    }

    private static final double SHOOTER_VEL = 1850;

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
                track();
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
                        while (currentShootSlot >= 0 && !slotOcupat[currentShootSlot]) {
                            currentShootSlot--;
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
                track();
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
                track();
                if (actionTimer.getElapsedTimeSeconds() >= 0.25) {
                    ShootingStare = 13;
                }
                break;

            case 13:
                actionTimer.resetTimer();
                ShootingStare = 14;
                break;

            case 14:
                if (actionTimer.getElapsedTimeSeconds() >= 0.13) {
                    ShootingStare = 15;
                }
                break;

            case 15:
                actionTimer.resetTimer();
                ShootingStare = 16;
                break;

            case 16:
                if (actionTimer.getElapsedTimeSeconds() >= 0.09) {
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
                    double dist = n.distanta.getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM);
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
                synchronized (slot) {
                    slotOcupat[0] = false;
                    slotOcupat[1] = false;
                    slotOcupat[2] = false;
                    slotColor[0] = -1;
                    slotColor[1] = -1;
                    slotColor[2] = -1;
                }
                currentShootSlot = 2;
                TragereInProgres = false;
                ShootingStare = 0;
                break;
        }
    }

    private void track() {
        n.tracks(follower, TARGET_X, TARGET_Y);
    }
    private void Intake() {
        IntakeThread = new Thread(new Runnable() {
            private boolean ballBeingProcessed = false;

            @Override
            public void run() {
                while (!stop) {
                    try { Thread.sleep(10); } catch (InterruptedException e) { break; }
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
                detecteazaPattern();
                follower.followPath(laShooting);
                // Porneste scanarea in paralel cu mersul
                startScaneazaPreloadAsync();
                setPathState(1);
                break;

            case 1:
                // Asteapta si sa ajunga SI sa termine scanarea
                if (!follower.isBusy() && scanareTerminata) {
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
                // Redus de la 1.5s - trece direct
                setPathState(4);
                break;

            case 4:
                synchronized (slot) {
                    slotOcupat[0] = false;
                    slotOcupat[1] = false;
                    slotOcupat[2] = false;
                    slotColor[0] = -1;
                    slotColor[1] = -1;
                    slotColor[2] = -1;
                    IntakeSlot = 0;
                }
                n.sortare.setPosition(Pozitii.luarea1);
                n.kdf(100);
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
                if (getLoculete() >= 3) {
                    n.kdf(400);
                    intakePornit = false;
                    setPathState(7);
                } else if (actionTimer.getElapsedTimeSeconds() >= 3.5) {
                    intakePornit = false;
                    if (getLoculete() > 0) {
                        setPathState(7);
                    } else {
                        setPathState(-1);
                    }
                }
                break;

            case 7:
                follower.followPath(tragere,0.7,false);
                // Porneste scanarea in paralel cu mersul
                startScaneazaPreloadAsync();
                setPathState(8);
                break;

            case 8:
                // Asteapta si sa ajunga SI sa termine scanarea
                if (!follower.isBusy() && scanareTerminata) {
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
                    setPathState(10);
                }
                break;

            case 10:
                synchronized (slot) {
                    slotOcupat[0] = false;
                    slotOcupat[1] = false;
                    slotOcupat[2] = false;
                    slotColor[0] = -1;
                    slotColor[1] = -1;
                    slotColor[2] = -1;
                }
                n.sortare.setPosition(Pozitii.luarea1);
                follower.followPath(treiluat);
                setPathState(11);
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.holdPoint(luare3);
                    n.intake.setPower(-1);
                    n.kdf(150);
                    n.intake.setPower(0);
                    intakePornit = true;
                    follower.followPath(treialuat);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    follower.holdPoint(aluat3);
                    actionTimer.resetTimer();
                    setPathState(13);
                }
                break;

            case 13:
                if (getLoculete() >= 3) {
                    n.kdf(400);
                    intakePornit = false;
                    setPathState(14);
                } else if (actionTimer.getElapsedTimeSeconds() >= 3.0) {
                    intakePornit = false;
                    if (getLoculete() > 0) {
                        setPathState(14);
                    } else {
                        follower.followPath(iesireas);
                        setPathState(-1);
                    }
                }
                break;

            case 14:
                follower.followPath(treiTras);
                // Porneste scanarea in paralel cu mersul
                startScaneazaPreloadAsync();
                setPathState(15);
                break;

            case 15:
                // Asteapta si sa ajunga SI sa termine scanarea
                if (!follower.isBusy() && scanareTerminata) {
                    follower.holdPoint(tras3);
                    setPathState(16);
                }
                break;

            case 16:
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
        telemetry.addData("Scanare terminata", scanareTerminata ? "DA" : "NU");
        telemetry.addData("Bile colectate", getLoculete());
        telemetry.addData("Toate detectate", toateBileleDetectate() ? "DA" : "NU");
        telemetry.addLine("");

        for (int i = 0; i < 3; i++) {
            String status;
            if (!slotOcupat[i]) {
                status = "gol";
            } else if (slotColor[i] == 0) {
                status = "VERDE";
            } else if (slotColor[i] == 1) {
                status = "MOV";
            } else {
                status = "?";
            }
            telemetry.addData("Slot " + (i + 1), status);
        }

        telemetry.addLine("");
        telemetry.addData("Pattern ID", idTag);
        telemetry.addData("Use Pattern", Pattern);
        telemetry.addData("LL tx", "%.2f°", n.getLimelightTx());
        telemetry.addData("Tag vizibil", n.isTagVisible() ? "DA" : "NU");
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

        for (int i = 0; i < 3; i++) {
            slotColor[i] = -1;
        }

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

        slotOcupat[0] = true;
        slotOcupat[1] = true;
        slotOcupat[2] = true;
        slotColor[0] = -1;
        slotColor[1] = -1;
        slotColor[2] = -1;

        n.sortare.setPosition(Pozitii.luarea1);

        n.limelight.pipelineSwitch(0);
        n.resetTurelaPID();

        Intake();
        IntakeThread.start();
    }

    @Override
    public void stop() {
        stop = true;

        if (scanThread != null) {
            scanThread.interrupt();
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
        n.bascula.setPosition(0.5807);
    }
}
