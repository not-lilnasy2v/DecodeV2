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

@Autonomous(name = "Departe Rosu")
public class AproapeRosu extends OpMode {
    sistemeAuto n = new sistemeAuto();
    public Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(88, 8, Math.toRadians(90));
    private final Pose shootingPose = new Pose(85.35034965034964, 23.07692307692308, Math.toRadians(90));
    private final Pose colectare = new Pose(134.54545454545456, 35.75524475524475, Math.toRadians(0));
    private final Pose cotrolare = new Pose(81.74272629354311, 40.5307423032044);
    private final Pose corectarelacotrolare = new Pose(103.58734200455321, 34.11270001876771);
    private final Pose tragere2 = new Pose(87.35034965034964, 23.07692307692308, Math.toRadians(90));

    private Path laShooting;
    private PathChain iale, tragere;

    private boolean TragereInProgres = false;
    private int ShootingStare = 0;
    private int currentShootSlot = 2;
    private int ballTras = 0;

    private volatile boolean[] slotOcupat = new boolean[3];
    private volatile int[] slotColor = new int[3];  // 0=verde, 1=mov, -1=ceva de nevazut
    private volatile boolean intakePornit = false;
    private volatile boolean stop = false;
    private Thread IntakeThread;
    private volatile int IntakeSlot = 0;
    private final Object slot = new Object();

    private int idTag = 0;
    private int[] cPattern = new int[3];
    private boolean Pattern = false;

    private int getLoculete() {
        synchronized (slot) {
            int count = 0;
            for (boolean occupied : slotOcupat) {
                if (occupied) count++;
            }
            return count;
        }
    }

    private double getLuarePos(int slot) {
        if (slot == 0) return Pozitii.luarea1;
        if (slot == 1) return Pozitii.luarea2;
        return Pozitii.luarea3;
    }

    public void buildPaths() {
        laShooting = new Path(new BezierLine(startPose, shootingPose));
        laShooting.setConstantHeadingInterpolation(startPose.getHeading());

        iale = follower.pathBuilder()
                .addPath(new BezierCurve(shootingPose, cotrolare, corectarelacotrolare,colectare))
                .setTangentHeadingInterpolation()
                .build();
        tragere = follower.pathBuilder()
                .addPath(new BezierLine(colectare, tragere2))
                .setLinearHeadingInterpolation(colectare.getHeading(), tragere2.getHeading())
                .setTranslationalConstraint(1)
                .setHeadingConstraint(10)
                .setTimeoutConstraint(50)
                .build();
    }

    private void verificaSloturiNecunoscute() {
        int Incercari = 5;  // 5 incercari
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

    private boolean toateBileleDetectate() {
        for (int i = 0; i < 3; i++) {
            if (slotOcupat[i] && slotColor[i] == -1) {
                return false;
            }
        }
        return true;
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

    private double getAruncarePos(int slot) {
        if (slot == 0) return Pozitii.aruncare1;
        if (slot == 1) return Pozitii.aruncare2;
        return Pozitii.aruncare3;
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
                ballTras = getLoculete();
                if (ballTras > 0) {
                    if (Pattern) {
                        int patternIndex = Math.max(0, Math.min(2, 3 - ballTras));
                        currentShootSlot = gasesteBilaCuCuloare(cPattern[patternIndex]);
                    } else {
                        // Rapid fire - de la slot 2 în jos
                        currentShootSlot = 2;
                        while (currentShootSlot >= 0 && !slotOcupat[currentShootSlot]) {
                            currentShootSlot--;
                        }
                    }

                    if (currentShootSlot >= 0) {
                        ShootingStare = 3;
                    } else {
                        ShootingStare = 9;
                    }
                } else {
                    ShootingStare = 9;
                }
                break;

            case 3:
                n.sortare.setPosition(getAruncarePos(currentShootSlot));
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
                    synchronized (slot) {
                        slotOcupat[currentShootSlot] = false;
                        slotColor[currentShootSlot] = -1;
                    }
                    ballTras--;
                    ShootingStare = 2;
                }
                break;

            case 9:
                n.sortare.setPosition(Pozitii.luarea1);
                actionTimer.resetTimer();
                ShootingStare = 10;
                break;

            case 10:
                if (actionTimer.getElapsedTimeSeconds() >= 0.3) {
                    n.shooter.setVelocity(0);
                    n.shooter2.setVelocity(0);
                    ShootingStare = 11;
                }
                break;

            case 11:
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
        n.trackLimelight();
    }

    private void untrack() {
        n.stopTurela();
    }

    private void Intake() {
        IntakeThread = new Thread(new Runnable() {
            private long lastBallTime = 0;
            private int bileCurente = 0;
            private boolean asteptaIntrare = false;

            @Override
            public void run() {
                while (!stop) {
                    if (intakePornit && bileCurente < 3) {
                        n.intake.setPower(1);

                        double leDistanta = n.distanta.getDistance(DistanceUnit.CM);
                        long currentTime = System.currentTimeMillis();

                        if (!asteptaIntrare && leDistanta < 20 && (currentTime - lastBallTime) > 400) {
                            asteptaIntrare = true;
                            lastBallTime = currentTime;

                            n.resetareDetection();
                            slotColor[bileCurente] = n.detecteazaBiloaca();

                        } else if (asteptaIntrare && leDistanta >= 20) {
                            slotOcupat[bileCurente] = true;
                            bileCurente++;

                            if (bileCurente < 3) {
                                n.sortare.setPosition(getLuarePos(bileCurente));
                                n.kdf(200);
                            }

                            asteptaIntrare = false;
                        }
                    } else if (!intakePornit) {
                        n.intake.setPower(0);
                        bileCurente = 0;
                        asteptaIntrare = false;
                    }

                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        break;
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
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.holdPoint(shootingPose);
                    scaneazaPreload();
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
                if (actionTimer.getElapsedTimeSeconds() >= 1.5) {
                    setPathState(4);
                }
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
                follower.followPath(iale);
                setPathState(5);
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.holdPoint(colectare);
                    n.kdf(50);
                    actionTimer.resetTimer();
                    setPathState(6);
                }
                break;

            case 6:
                if (getLoculete() >= 3) {
                    n.kdf(400);
                    intakePornit = false;
                    setPathState(7);
                } else if (actionTimer.getElapsedTimeSeconds() >= 3.0) {
                    intakePornit = false;
                    if (getLoculete() > 0) {
                        setPathState(7);
                    } else {
                        setPathState(-1);
                    }
                }
                break;

            case 7:
                verificaSloturiNecunoscute();

                if (Pattern) {
                    int PrimaCuloare = cPattern[0];
                    int PrimuSlot = gasesteBilaCuCuloare(PrimaCuloare);
                    if (PrimuSlot != -1) {
                        n.sortare.setPosition(getAruncarePos(PrimuSlot));
                    }
                } else {
                    n.sortare.setPosition(Pozitii.aruncare3);
                }

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

        slotOcupat[0] = true;
        slotOcupat[1] = true;
        slotOcupat[2] = true;
        slotColor[0] = -1;
        slotColor[1] = -1;
        slotColor[2] = -1;

        n.sortare.setPosition(Pozitii.luarea1);

        n.limelight.pipelineSwitch(2);
        n.resetTurelaPID();

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
        RobotPozitie.idTag = idTag;

        n.shooter.setVelocity(0);
        n.shooter2.setVelocity(0);
        n.intake.setPower(0);
    }
}
