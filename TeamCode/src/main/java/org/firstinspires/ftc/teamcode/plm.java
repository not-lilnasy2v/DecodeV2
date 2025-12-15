//package org.firstinspires.ftc.teamcode;
//
//import static java.lang.Math.abs;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import static java.lang.Math.abs;
//
//
//
//
//
//
//@TeleOp(name = "gionno dip", group = " ")
//@Disabled
//public class plm extends OpMode {
//    public DcMotorEx motorBR,motorBL,motorFL,motorFR;
//    double sm = 1;
//    double y, x, rx;
//    double max = 0, poz_st, poz_dr;
//    double pmotorBL, pmotorBR, pmotorFL, pmotorFR;
//    ublic boolean stop, timp1 = false, timp3 = false, timp2 = false, timp4 = false, timp5 = false, timp6 =false, timp7 = false, timp8 = false, timp9 = false, timp10 = false, timp11=false, timp12=false, timp13=false, timp14=false, timp15=false, timp16=false, timp17=false, timp18=false;
//    boolean pid = false;
//    double pidResultS;
//    PidControllerAdevarat pidSlider = new PidControllerAdevarat(0,0,0);
//    sistemeTeleOp s = new sistemeTeleOp();
//    ElapsedTime timer1 = new ElapsedTime();
//    ElapsedTime timer2 = new ElapsedTime();
//    ElapsedTime timer3 = new ElapsedTime();
//    ElapsedTime timer4 = new ElapsedTime();
//    ElapsedTime timer5 = new ElapsedTime();
//    ElapsedTime timer6 = new ElapsedTime();
//    ElapsedTime timer7 = new ElapsedTime();
//    ElapsedTime timer8 = new ElapsedTime();
//    ElapsedTime timer9 = new ElapsedTime();
//    ElapsedTime timer10 = new ElapsedTime();
//    ElapsedTime timer11 = new ElapsedTime();
//    ElapsedTime timer12 = new ElapsedTime();
//    ElapsedTime timer13 = new ElapsedTime();
//    ElapsedTime timer14 = new ElapsedTime();
//    ElapsedTime timer15 = new ElapsedTime();
//    ElapsedTime timer16 = new ElapsedTime();
//    ElapsedTime timer17 = new ElapsedTime();
//    ElapsedTime timer18 = new ElapsedTime();
//
//    @Override
//    public void init() {
//        s.initSistem(hardwareMap);
//        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL"); // Motor Back-Left
//        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR"); // Motor Back-Left
//        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL"); // Motor Back-Left
//        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR"); // Motor Back-Left
//
//        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
//        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
//
//        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//
//        // s.bratOut2.setPosition(p.out_luare);
//        s.bratDreapta.setPosition(Pozitii.bratDreapta_bara);
//        s.bratStanga.setPosition(Pozitii.bratStanga_bara);
//        s.planulB.setPosition(Pozitii.planulB_inchis);
//        s.bratIN.setPosition(p.bratIN_static);
//        s.turela.setPosition(p.turela_drept);
//        // s.bratIn2.setPosition(p.bratIn_extindere);
//        //  s.bratIn1.setPosition(p.bratIn_extindereTele);
//        //s.brat_st.setPosition(p.turela_mijloc_st);
//        //s.brat_dr.setPosition(p.turela_mijloc_dr);
//        s.incheietura.setPosition(p.incheietura_human);
//        s.ext2.setPosition(p.ext_inchis);
//        s.ext1.setPosition(p.ext_inchis);
//        s.ghearaOut.setPosition(Pozitii.gheara_inchis);
//        s.rotire.setPosition(p.poz_rotire_ciung);
//        s.incheieturaout.setPosition(Pozitii.incheietura_bara);
//
//    }
//
//    public void start(){
//        // aici se pornesc thread-urile
//        Chassis.start();
//        System.start();
//    }
//    //
////
//    private final Thread Chassis = new Thread(new Runnable() {
//        @Override
//        public void run() {
//            while (!stop) {
//                y = -gamepad1.left_stick_y;
//                x = gamepad1.left_stick_x;
//                rx = gamepad1.right_stick_x;
//
//                pmotorFL = (y + x + rx);
//                pmotorBL = (y - x + rx);
//                pmotorBR = (y + x - rx);
//                pmotorFR = (y - x - rx);
//
//                max = abs(pmotorFL);
//                if (abs(pmotorFR) > max) {
//                    max = abs(pmotorFR);
//                }
//                if (abs(pmotorBL) > max) {
//                    max = abs(pmotorBL);
//                }
//                if (abs(pmotorBR) > max) {
//                    max = abs(pmotorBR);
//                }
//                if (max > 1) {
//                    pmotorFL /= max;
//                    pmotorFR /= max;
//                    pmotorBL /= max;
//                    pmotorBR /= max;
//                }
//                //SLOW-MOTION
//                if (gamepad1.left_trigger > 0) {
//                    sm = 2;
//                    //arm.setPower(poz/sm);
//                } else {
//                    //SLOWER-MOTION
//                    if (gamepad1.right_trigger > 0) {
//                        sm = 5;
//                    } else {
//                        sm = 1;
//                    }
//                }
//                POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
//
//            }
//        }
//    });
//
//    private final Thread System = new Thread(new Runnable() {
//        @Override
//        public void run() {
//            pidSlider.enable();
//            while (!stop) {
//                pidSlider.setPID(Config.pslider, Config.islider, Config.dslider);
//
//                if (gamepad2.left_stick_y != 0.0) {
//                    s.sliderD.setPower(gamepad2.left_stick_y);
//                    s.sliderS.setPower(gamepad2.left_stick_y);
//                    pid = true;
//                } else {
//                    if (pid) {
//                        pid = false;
//                        pidSlider.setSetpoint(s.sliderD.getCurrentPosition());
//                    }
//                    if (s.touch_slider.isPressed()) {
//                        pidSlider.setSetpoint(0);
//                        s.sliderD.setPower(0);
//                        s.sliderS.setPower(0);
//                    } else {
//                        pidResultS = pidSlider.performPID(s.sliderD.getCurrentPosition());
//                        //  s.slider1.setPower(pidResultS);
//                        //  s.slider2.setPower(pidResultS);
//                        s.sliderD.setPower(pidResultS);
//                        s.sliderS.setPower(pidResultS);
//                    }
//                }
//
//
//                s.urcareStanga.setPower(gamepad2.right_stick_y);
//                s.urcareDreapta.setPower(gamepad2.right_stick_y);
//
//
//                if (gamepad1.b) {
//                    s.ghearaOut.setPosition(Pozitii.gheara_inchis);
//                    //s.ext1.setPosition(p.ext_urcare);
//                    // s.ext2.setPosition(p.ext_urcare);
//                    s.planulB.setPosition(Pozitii.planulB_inchis);
//                    timer16.reset();
//                    timp16 = true;
//                }
//                if (timer16.milliseconds() >= 60 && timp16) {
//                    s.bratDreapta.setPosition(Pozitii.bratDreapta_luare_gard);
//                    s.bratStanga.setPosition(Pozitii.bratStanga_luare_gard);
//                    s.incheieturaout.setPosition(Pozitii.incheietura_luare_gard);
//                    s.retragere_slider();
//                    //timer17.reset();
//                    timp16 = false;
//                    //timp17 = true;
//                    timer18.reset();
//                    timp18 = true;
//
//                }
//                if (timer18.milliseconds() >=400 && timp18) {
//                    s.planulB.setPosition(Pozitii.planulB_luare_gard);
//                    s.ghearaOut.setPosition(Pozitii.gheara_deschis);
//                    timp18 = false;
//                }
//                if (gamepad2.b) {
//                    s.ghearaOut.setPosition(Pozitii.gheara_inchis);
//                    s.planulB.setPosition(Pozitii.planulB_inchis);
//                    timer8.reset();
//                    timp8 = true;
//                }
//                if (timer8.milliseconds() >= 60 && timp8) {
//                    s.bratDreapta.setPosition(Pozitii.bratDreapta_luare_gard);
//                    s.bratStanga.setPosition(Pozitii.bratStanga_luare_gard);
//                    s.incheieturaout.setPosition(Pozitii.incheietura_luare_gard);
//                    s.retragere_slider();
//                    timer3.reset();
//                    timp8 = false;
//                    timp3 = true;
//                    timer11.reset();
//                    timp11 = true;
//
//                }
//                if (timer11.milliseconds() >=400 && timp11) {
//                    s.planulB.setPosition(Pozitii.planulB_luare_gard);
//                    s.ghearaOut.setPosition(Pozitii.gheara_deschis);
//                    timp11 = false;
//                }
//                if (gamepad2.touchpad) {
//                    s.ghearaOut.setPosition(Pozitii.gheara_deschis);
//                }
//                if (gamepad1.touchpad) {
//                    s.ghearaOut.setPosition(Pozitii.gheara_deschis);
//                }
//                if (gamepad1.x) {
//                    s.ghearaOut.setPosition(Pozitii.gheara_deschis);
//                    s.ext1.setPosition(p.ext_urcare);
//                    s.ext2.setPosition(p.ext_urcare);
//                }
//
//                if (gamepad1.a) {
//                    s.rotire.setPosition(p.poz_rotire_normal);
//                    //s.ghearaIn.setPosition(0.482);
//                    s.bratIN.setPosition(p.bratIN_punere_human);
//                    s.incheietura.setPosition(p.incheietura_human);
//                    s.turela.setPosition(p.turela_human);}
//
//
//
//                if (gamepad1.left_bumper) {
//                    s.rotire.setPosition(p.poz_rotire_normal);
//                    s.ghearaIn.setPosition(0.8);
//                    s.bratIN.setPosition(p.bratIN_intermediar);
//
//                }
//                if (gamepad1.right_bumper) {
//                    s.rotire.setPosition(p.poz_rotire_ciung);
//                    s.ghearaIn.setPosition(0.8);
//                    s.bratIN.setPosition(p.bratIN_intermediar);
//
//                }
//                if (gamepad1.right_trigger > 0) {
//                    s.extindere_ext();
//                    s.bratIN.setPosition(p.bratIN_static);
//                    s.turela.setPosition(p.turela_drept);
//
//                    s.incheietura.setPosition(p.incheiatura_luare);
//                    s.rotire.setPosition(p.poz_rotire_ciung);
//
//                }
//                if (gamepad2.y) {slider_extension(-1110, 5000, 15);
//                }
//                if (gamepad2.dpad_up) {
//                    s.ghearaOut.setPosition(Pozitii.ghera_strangulat);
//                    timer5.reset();
//                    timp5 = true;}
//
//                if (timer5.milliseconds() >= 100 && timp5){
//                    s.ghearaIn.setPosition(p.gheara_in_deschis);
//                    timp5 = false;
//                    timer2.reset();
//                    timp2 = true;}
//                if (timer2.milliseconds() >= 100 && timp2){
//                    s.bratDreapta.setPosition(Pozitii.bratDreapta_cos);
//                    s.bratStanga.setPosition(Pozitii.bratStanga_cos);
//                    s.planulB.setPosition(Pozitii.planulB_inchis);
//                    s.incheieturaout.setPosition(Pozitii.incheietura_cos);
//                    slider_extension(-1800, 5000, 15);
//                    timp2 = false;
//                }
//                if (gamepad2.dpad_down) {
//                    s.ghearaOut.setPosition(Pozitii.ghera_strangulat);
//                    timer1.reset();
//                    timp1 = true;}
//
//                if (timer1.milliseconds() >= 100 && timp1){
//                    s.ghearaIn.setPosition(p.gheara_in_deschis);
//                    timp1 = false;
//                    timer9.reset();
//                    timp9 = true;}
//
//                if (timer9.milliseconds() >= 100 && timp9){
//                    s.bratDreapta.setPosition(Pozitii.bratDreapta_cos);
//                    s.bratStanga.setPosition(Pozitii.bratStanga_cos);
//                    s.planulB.setPosition(Pozitii.planulB_inchis);
//                    s.incheieturaout.setPosition(Pozitii.incheietura_cos);
//                    slider_extension(-700, 5000, 15);
//                    timp9 = false;
//
//                }
//                if (gamepad2.dpad_left){
//                    s.bratStanga.setPosition(Pozitii.bratStanga_bara);
//                    s.bratDreapta.setPosition(Pozitii.bratDreapta_bara);
//                    s.incheieturaout.setPosition(Pozitii.incheietura_bara);
//                    s.turela.setPosition(p.turela_human);
//                    s.bratIN.setPosition(p.bratIN_punere_human);
//                    s.incheietura.setPosition(p.incheietura_human);
//                    s.planulB.setPosition(Pozitii.planulB_urcare);
//                    s.ext1.setPosition(p.ext_urcare);
//                    s.ext2.setPosition(p.ext_urcare);}
//
//
//                if (gamepad2.dpad_right) {
//                    s.ghearaIn.setPosition(p.gheara_in_deschis);
//                    s.ghearaOut.setPosition(Pozitii.gheara_inchis);
//                    s.planulB.setPosition(Pozitii.planulB_inchis);
//                    timer4.reset();
//                    timp4 = true;
//                }
//                if (timer4.milliseconds() >= 50 && timp4) {
//                    slider_extension(-400, 1000, 15);
//
//                    s.bratDreapta.setPosition(Pozitii.bratDreapta_bara);
//                    s.bratStanga.setPosition(Pozitii.bratStanga_bara);
//                    s.planulB.setPosition(Pozitii.planulB_deschis);
//                    timp4 = false;
//                    timer10.reset();
//                    timp10 = true;
//                }
//                if (timer10.milliseconds() >= 20 && timp10) {
//                    s.incheieturaout.setPosition(Pozitii.incheietura_bara);
//                    s.bratIN.setPosition(p.bratIN_static);
//                    s.turela.setPosition(p.turela_drept);
//                    s.rotire.setPosition(p.poz_rotire_ciung);
//                    timp10 = false;
//                }
//                if (gamepad1.y) {
//                    s.ghearaIn.setPosition(p.gheara_in_deschis);
//                    s.ghearaOut.setPosition(Pozitii.gheara_inchis);
//                    s.planulB.setPosition(Pozitii.planulB_inchis);
//                    timer15.reset();
//                    timp15 = true;
//                }
//                if (timer15.milliseconds() >= 70 && timp15) {
//                    //slider_extension(-400, 1000, 15);
//                    slider_extension(-1110, 5000, 15);
//
//                    s.bratDreapta.setPosition(Pozitii.bratDreapta_bara);
//                    s.bratStanga.setPosition(Pozitii.bratStanga_bara);
//                    s.planulB.setPosition(Pozitii.planulB_deschis);
//                    timp15 = false;
//                    timer14.reset();
//                    timp14 = true;
//                }
//                if (timer14.milliseconds() >= 20 && timp14) {
//                    s.incheieturaout.setPosition(Pozitii.incheietura_bara);
//                    s.bratIN.setPosition(p.bratIN_static);
//                    s.turela.setPosition(p.turela_drept);
//                    s.rotire.setPosition(p.poz_rotire_ciung);
//                    timp14 = false;
//                }
//
//
//                //if (gamepad1.y) {
//                //  s.incheietura.setPosition(p.incheiatura_luare);
//                //timer15.reset();
//                //timp15 = true;
//                //}
//                //  if (timer15.milliseconds() >= 50 && timp15) {
//                //    s.ghearaIn.setPosition(p.gheara_in_inchis);
//                //  timp15 = false;
//                //timer14.reset();
//                //timp14 = true;
//                //}
//                //if (timer14.milliseconds() >= 150 && timp14) {
//                //  s.rotire.setPosition(p.poz_rotire_ciung);
//
//                //s.incheietura.setPosition(p.incheieturaIN_tranfer);
//                //s.ext1.setPosition(0.217);
//                //s.ext2.setPosition(0.217);
//                //timp14 = false;
//                //}
//                if(gamepad1.dpad_down){s.extindere_legal();
//
//                    s.incheietura.setPosition(p.incheiatura_luare);
//                    s.rotire.setPosition(p.poz_rotire_ciung);
//
//                }
//
//                if (gamepad1.dpad_up) {
//
//                    s.bratIN.setPosition(p.bratIN_luare);
//                    s.incheietura.setPosition(p.incheiatura_luare);
//                    timer6.reset();
//                    timp6 = true;
//                }
//                if (timer6.milliseconds() >= 50 && timp6) {
//                    s.ghearaIn.setPosition(p.gheara_in_inchis);
//                    timp6 = false;
//                    timer7.reset();
//                    timp7 = true;
//                }
//                if (timer7.milliseconds() >= 200 && timp7) {
//                    s.rotire.setPosition(p.poz_rotire_ciung);
//                    s.bratIN.setPosition(p.bratIN_static);
//                    s.incheietura.setPosition(p.incheietura_human);
//                    s.ext1.setPosition(p.ext_inchis);
//                    s.ext2.setPosition(p.ext_inchis);
//
//                    timp7 = false;
//                }
//                if (gamepad1.left_trigger > 0) {
//                    s.ghearaIn.setPosition(0.77);
//                    s.turela.setPosition(p.turela_drept);
//                    timer12.reset();
//                    timp12 = true;}
//                if (timer12.milliseconds() >=200 && timp12) {
//                    s.ext1.setPosition(p.ext_inchis);
//                    s.ext2.setPosition(p.ext_inchis);
//                    s.bratIN.setPosition(p.bratIN_static);
//                    s.rotire.setPosition(p.poz_rotire_ciung);
//
//                    timp12=false;
//                }
//                if (gamepad2.a){
//                    s.bratDreapta.setPosition(Pozitii.bratDreapta_cos);
//                    s.bratStanga.setPosition(Pozitii.bratStanga_cos);
//                    s.planulB.setPosition(Pozitii.planulB_inchis);
//                    s.incheieturaout.setPosition(Pozitii.incheietura_cos);
//
//                }
//            }
//
//            //if (gamepad2.dpad_left) {
//            // s.ghearaOut.setPosition(Pozitii.gheara_inchis);
//            // s.bratDreapta.setPosition(Pozitii.bratDreapta_bara);
//            // s.bratStanga.setPosition(Pozitii.bratStanga_bara);
//            // s.planulB.setPosition(Pozitii.planulB_urcare);
//            // s.ext1.setPosition(p.ext_urcare);
//            // s.ext2.setPosition(p.ext_urcare);
//            //}
//
//
//        }
//
//    });
//    public void slider_extension(int poz1,int vel,double tolerance){
//        if (poz1 > s.sliderD.getCurrentPosition()){
//            while (s.sliderD.getCurrentPosition() < poz1){
//                s.sliderD.setVelocity(vel);
//                s.sliderS.setVelocity(vel);
//            }
//
//        }
//        else {
//            while (s.sliderD.getCurrentPosition()>poz1 + tolerance ){
//                s.sliderD.setVelocity(-vel);
//                s.sliderS.setVelocity(-vel);
//            }
//        }
//        s.sliderD.setVelocity(0);
//        s.sliderS.setVelocity(0);
//        s.sliderD.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        s.sliderS.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        pid = true;
//    }
//
//    public void stop(){stop = true;}
//    @Override
//    public void loop() {
//
//
////       telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
////    telemetry.addData("encoder dreapta", motorFR.getCurrentPosition() );
////    telemetry.addData("encoder stanga", motorFL.getCurrentPosition() );
////    telemetry.addData("encoder fata", motorBL.getCurrentPosition() );
//
//        //telemetry.addData("perpendic",  motorFL.getCurrentPosition());
//        // telemetry.addData("poz sider 2", s.slider2.getCurrentPosition());
//
//        // telemetry.addData("eroare", pidSlider.getError());
////        telemetry.addData("bratOut ", s.bratOut1.getPosition());
////        telemetry.addData("bratIn", s.bratIn1.getPosition());
////        telemetry.addData("incheietura", s.incheietura.getPosition());
////        telemetry.addData("albastru", s.colorIn.blue());
//////        telemetry.addData("extindere", ext);
////        telemetry.addData("p", pidSlider.getP());
////        telemetry.addData("i", pidSlider.getI());
////        telemetry.addData("d", pidSlider.getD());
////        telemetry.addData("limitare1 ", limitare1);
////        telemetry.addData("timp4", gamepad2.left_stick_y);
//
//        // telemetry.addData("trigger", trigger);
//        //telemetry.addData("servo2", s.bratOut1.getPosition());
////
////
//        //telemetry.update();
//    }
//    public void POWER(double df1, double sf1, double ds1, double ss1){
//        motorFR.setPower(df1);
//        motorBL.setPower(ss1);
//        motorFL.setPower(sf1);
//        motorBR.setPower(ds1);
//    }
//}