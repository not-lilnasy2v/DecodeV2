package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Main Rosu")
public class TelR extends OpMode {
    private DcMotorEx frontRight, frontLeft, backRight, backLeft;
    private Limelight3A limelight3A;
    boolean stop;
    double sm = 1;
    double max = 0;
    double FL, BL, BR, FR;
    sistemeTeleOp m = new sistemeTeleOp();

    //PIDF
    private final double TkP = 0.004, TkI = 0.0, TkD = 0.007;
    public boolean turelaTracking = false, tracking = false, Ipornit = false, IntakePornit = false, SortingPornit = false, SortingToggle = false;
    private double integral = 0, lastError = 0, imata, tx = 0, power = 0, posU;
    int loculete = 0, idTag = 0, positiiTurela;

    private volatile boolean sugere = false;
    private volatile boolean trageShooting = false;
    private final Object blocat = new Object();

    @Override
    public void init() {
        m.initsisteme(hardwareMap);
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(2);
        limelight3A.start();
    }

    public void start() {
        Chassis.start();
        Butoane.start();
        Turela.start();
        Sortare.start();
        Shooter.start();
    }

    private final Thread Butoane = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                posU = m.unghiS.getPosition();

                // Turela toggle
                boolean dpad_right1 = gamepad1.dpad_right;
                if (turelaTracking != dpad_right1) {
                    if (gamepad1.dpad_right) {
                        tracking = !tracking;
                    }
                    turelaTracking = dpad_right1;
                }

                if (gamepad1.dpad_up) {
                    posU += 0.003;
                }
                if (gamepad1.dpad_down) {
                    posU -= 0.003;
                }
                m.unghiD.setPosition(posU);
                m.unghiS.setPosition(posU);

                // Intake toggle
                boolean gamepad1_a = gamepad1.a;
                if (IntakePornit != gamepad1_a) {
                    if (gamepad1.a) {
                        Ipornit = !Ipornit;
                    }
                    IntakePornit = gamepad1_a;
                }
                if(gamepad2.dpad_up && idTag <1){
                    idTag = 1;
                }
                if(gamepad2.dpad_left && idTag <1){
                    idTag= 2;
                }
                if(gamepad2.dpad_down && idTag <1){
                    idTag= 3;
                }

                boolean gamepad2_a = gamepad2.a;
                if (SortingToggle != gamepad2_a) {
                    if (gamepad2.a) {
                        SortingPornit = !SortingPornit;
                        gamepad2.rumble(500); //Vibrator
                    }
                    SortingToggle = gamepad2_a;
                }

                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    });

    private final Thread Turela = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                positiiTurela = m.turela.getCurrentPosition();

                if (tracking) {
                    LLResult result = limelight3A.getLatestResult();

                    if (result != null && result.isValid()) {
                        tx = result.getTx();

                        double error = tx;
                        integral += error;
                        double derivative = error - lastError;

                        power = TkP * error + TkI * integral + TkD * derivative;

                        int targetPos = positiiTurela + (int) (power * 50);

                        if (targetPos > Pozitii.TURRET_MAX_POS) {
                            targetPos = Pozitii.TURRET_MAX_POS;
                            integral = 0;
                        }
                        if (targetPos < Pozitii.TURRET_MIN_POS) {
                            targetPos = Pozitii.TURRET_MIN_POS;
                            integral = 0;
                        }

                        if (positiiTurela >= Pozitii.TURRET_MIN_POS && positiiTurela <= Pozitii.TURRET_MAX_POS) {
                            m.turela.setPower(power);
                        } else {
                            if ((positiiTurela <= Pozitii.TURRET_MIN_POS && power > 0) ||
                                    (positiiTurela >= Pozitii.TURRET_MAX_POS && power < 0)) {
                                m.turela.setPower(power);
                            } else {
                                m.turela.setPower(0);
                            }
                        }
                        lastError = error;


                    } else {
                        m.turela.setPower(0);
                    }

                    //securitate
                    if (positiiTurela > Pozitii.TURRET_MAX_POS) {
                        positiiTurela = Pozitii.TURRET_MAX_POS;
                    }
                    if (positiiTurela < Pozitii.TURRET_MIN_POS) {
                        positiiTurela = Pozitii.TURRET_MIN_POS;
                    }


                    lastError = 0;
                    integral = 0;
                }
                else {
                    m.turela.setPower(0);
                }

                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    });

    private final Thread Sortare = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                synchronized (blocat) {
                    if (Ipornit && !trageShooting && loculete < 3) {
                        sugere = true;
                        m.intake.setPower(1);

                        imata = m.distanta.getDistance(DistanceUnit.CM);

                        if (imata < 20) {
                            if (loculete == 0) {
                                gamepad1.rumble(1000);
                                m.sortare.setPosition(Pozitii.luarea2);
                                loculete++;
                                m.kdf(950);
                            } else if (loculete == 1) {
                                gamepad1.rumble(1000);

                                m.sortare.setPosition(Pozitii.luarea3);
                                loculete++;
                                m.kdf(950);
                            } else if (loculete == 2) {
                                gamepad1.rumble(1000);
                                loculete++;
                            }
                        }
                    } else {
                        sugere = false;
                        m.intake.setPower(0);
                    }
                }

                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    });

    private enum SortareShooter {
        SIDLE,
        Incarca,
        CautaSiImpuscai,
        GATA,
        RESETTING
    }

    private SortareShooter Sstare = SortareShooter.SIDLE;
    private int[] cPattern = new int[3];
    private boolean[] ocupat = new boolean[3];

    private final Thread Shooter = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {

                synchronized (blocat) {
                    if (gamepad1.x && loculete > 0 && !sugere) {
                        trageShooting = true;
                        m.shooter.setVelocity(1800);

                        if (SortingPornit) {
                            switch (Sstare) {
                                case SIDLE:
                                    m.sortare.setPosition(Pozitii.aruncare1);
                                    m.kdf(250);

                                    for (int i = 0; i < 3; i++) {
                                        ocupat[i] = (i < loculete);
                                    }

                                    Sstare = SortareShooter.Incarca;
                                    break;

                                case Incarca:
                                    if (idTag == 3) {
                                        cPattern[0] = 1;
                                        cPattern[1] = 1;
                                        cPattern[2] = 0;
                                    } else if (idTag == 2) {
                                        cPattern[0] = 1;
                                        cPattern[1] = 0;
                                        cPattern[2] = 1;
                                    } else if (idTag == 1) {
                                        cPattern[0] = 0;
                                        cPattern[1] = 1;
                                        cPattern[2] = 1;
                                    } else {
                                        Sstare = SortareShooter.GATA;
                                        break;
                                    }

                                    Sstare = SortareShooter.CautaSiImpuscai;
                                    break;

                                case CautaSiImpuscai:
                                    BilaInPattern();
                                    Sstare = SortareShooter.RESETTING;
                                    break;

                                case RESETTING:
                                    m.sortare.setPosition(Pozitii.luarea1);
                                    m.kdf(300);
                                    Sstare = SortareShooter.GATA;
                                    break;

                                case GATA:
                                    m.shooter.setVelocity(0);
                                    trageShooting = false;
                                    Sstare = SortareShooter.SIDLE;
                                    break;
                            }
                        } else {
                            rapidFireShoot();
                        }
                    } else {
                        if (Sstare != SortareShooter.SIDLE) {
                            m.shooter.setVelocity(0);
                            m.sortare.setPosition(Pozitii.luarea1);
                            trageShooting = false;
                            Sstare = SortareShooter.SIDLE;
                        }
                    }
                }

                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }

        private void rapidFireShoot() {
            if (loculete > 0 && Sstare == SortareShooter.SIDLE) {
                Sstare = SortareShooter.Incarca;
            }

            while (loculete > 0) {
                int shootPos = -1;
                if (loculete >= 3) shootPos = 2;
                else if (loculete >= 2) shootPos = 1;
                else if (loculete >= 1) shootPos = 0;

                if (shootPos >= 0) {
                    if (shootPos == 0) m.sortare.setPosition(Pozitii.aruncare1);
                    else if (shootPos == 1) m.sortare.setPosition(Pozitii.aruncare2);
                    else if (shootPos == 2) m.sortare.setPosition(Pozitii.aruncare3);

                    m.kdf(1500);

                    m.Saruncare.setPosition(Pozitii.lansare);
                    m.kdf(150);
                    m.Saruncare.setPosition(Pozitii.coborare);
                    m.kdf(150);

                    loculete--;
                }
            }

            m.sortare.setPosition(Pozitii.luarea1);
            m.kdf(300);
            m.shooter.setVelocity(0);
            trageShooting = false;
            Sstare = SortareShooter.SIDLE;
        }

        private void BilaInPattern() {
            for (int step = 0; step < 3 && loculete > 0; step++) {
                int need = cPattern[step];
                boolean ballShot = false;

                if (!ballShot && ocupat[0]) {
                    m.sortare.setPosition(Pozitii.aruncare1);
                    m.kdf(950);

                    boolean mov = m.color.green() <= Pozitii.mov_verde;

                    if ((need == 1 && mov) || (need == 0 && !mov)) {
                        TrageBila();
                        ocupat[0] = false;
                        loculete--;
                        ballShot = true;
                    }
                }

                if (!ballShot && ocupat[1]) {
                    m.sortare.setPosition(Pozitii.aruncare2);
                    m.kdf(950);

                    boolean mov = m.color.green() <= Pozitii.mov_verde;

                    if ((need == 1 && mov) || (need == 0 && !mov)) {
                        TrageBila();
                        ocupat[1] = false;
                        loculete--;
                        ballShot = true;
                    }
                }

                if (!ballShot && ocupat[2]) {
                    m.sortare.setPosition(Pozitii.aruncare3);
                    m.kdf(950);

                    boolean mov = m.color.green() <= Pozitii.mov_verde;

                    if ((need == 1 && mov) || (need == 0 && !mov)) {
                        TrageBila();
                        ocupat[2] = false;
                        loculete--;
                        ballShot = true;
                    }
                }

                if (!ballShot) {
                    for (int i = 0; i < 3; i++) {
                        if (ocupat[i]) {
                            if (i == 0) m.sortare.setPosition(Pozitii.aruncare1);
                            else if (i == 1) m.sortare.setPosition(Pozitii.aruncare2);
                            else m.sortare.setPosition(Pozitii.aruncare3);

                            m.kdf(1300);
                            TrageBila();
                            ocupat[i] = false;
                            loculete--;
                            break;
                        }
                    }
                }
            }
        }

        private void TrageBila() {
            m.Saruncare.setPosition(Pozitii.lansare);
            m.kdf(150);
            m.Saruncare.setPosition(Pozitii.coborare);
            m.kdf(150);
        }
    });

    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;

                FL = y + x + rx;
                BL = y - x + rx;
                FR = y - x - rx;
                BR = y + x - rx;

                max = abs(FL);
                if (abs(FR) > max) max = abs(FR);
                if (abs(BL) > max) max = abs(BL);
                if (abs(BR) > max) max = abs(BR);

                if (max > 1) {
                    FL /= max;
                    FR /= max;
                    BL /= max;
                    BR /= max;
                }

                if (gamepad1.right_trigger > 0) {
                    sm = 2;
                } else if (gamepad1.left_trigger > 0) {
                    sm = 5;
                } else {
                    sm = 1;
                }

                POWER(FR / sm, BL / sm, BR / sm, FL / sm);

                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    });

    public void stop() {
        stop = true;
    }

    @Override
    public void loop() {

        telemetry.addData("TX", tx);
        telemetry.addData(" Power", power);
        telemetry.addData("Pos", positiiTurela);
        telemetry.update();
    }

    public void POWER(double fr1, double bl1, double br1, double fl1) {
        frontRight.setPower(fr1);
        backLeft.setPower(bl1);
        frontLeft.setPower(fl1);
        backRight.setPower(br1);
    }
}