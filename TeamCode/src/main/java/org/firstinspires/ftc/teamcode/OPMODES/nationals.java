//package org.firstinspires.ftc.teamcode;
//
//import static org.firstinspires.ftc.teamcode.TeleOp.NewTest2.State.low;
//import static org.firstinspires.ftc.teamcode.TeleOp.NewTest2.State.lowOpen;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.TouchSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
///**
// * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
// * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
// * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
// * class is instantiated on the Robot Controller and executed.
// *
// * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
// * It includes all the skeletal structure that all linear OpModes contain.
// *
// * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
// * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
// */
//@TeleOp
//public class NewTest2 extends LinearOpMode {
//    BNO055IMU imu;
//    Orientation angles;
//    Acceleration gravity;
//    DcMotor fl;
//    DcMotor fr;
//    DcMotor rl;
//    DcMotor rr;
//    DcMotor hSlide;
//
//    DcMotor vSlide;
//    Servo grabber;
//    Servo dropper;
//    Servo lRotate;
//    Servo rRotate;
//
//    DigitalChannel hClose;
//    DigitalChannel hOpen;
//    DigitalChannel vClose;
//    DigitalChannel vOpen;
//    boolean grab = false;
//    boolean drop = false;
//    boolean up = false;
//    public enum State {
//        init,
//        h1,
//        h2,
//        h3,
//        loaded,
//        reset,
//        forward,
//        dump,
//        low,
//        lowOpen,
//        grab,
//        hold,
//        dropWait,
//        backward
//    }
//    Gdrive.State sState = Gdrive.State.init;
//    @Override
//    public void runOpMode() {
//        // IMU mapping
//
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//        // Chassis drive mapping
//
//        lRotate = hardwareMap.get(Servo.class, "lRotate");
//        rRotate = hardwareMap.get(Servo.class, "rRotate");
//        hOpen = hardwareMap.get(DigitalChannel.class, "hOpen");
//        hClose = hardwareMap.get(DigitalChannel.class, "hClose");
//        vOpen = hardwareMap.get(DigitalChannel.class, "vOpen");
//        vClose = hardwareMap.get(DigitalChannel.class, "vClose");
//        hSlide = hardwareMap.get(DcMotor.class, "hSlide");
//        vSlide = hardwareMap.get(DcMotorEx.class, "vSlide");
//        grabber = hardwareMap.get(Servo.class, "grabber");
//        dropper = hardwareMap.get(Servo.class, "dropper");
//        hSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        hSlide.setDirection(DcMotor.Direction.REVERSE);
//        vSlide.setDirection(DcMotor.Direction.REVERSE);
//        telemetry.addData("Status", "Initialized");
//        fl = hardwareMap.get(DcMotor.class,"fl" );
//        fr = hardwareMap.get(DcMotor.class,"fr" );
//        rl = hardwareMap.get(DcMotor.class,"rl" );
//        rr = hardwareMap.get(DcMotor.class,"rr" );
//        fl.setDirection(DcMotor.Direction.FORWARD);
//        rl.setDirection(DcMotor.Direction.FORWARD);
//        fr.setDirection(DcMotor.Direction.REVERSE);
//        rr.setDirection(DcMotor.Direction.REVERSE);
//        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
//        telemetry.update();
//
//
//        // Wait for the game to start (driver presses PLAY)
//        // After pressing init button
//        while ((!isStopRequested()) && (!isStarted())) {
//            grabber.setPosition(0.7);
//            lRotate.setPosition(0.5);
//            rRotate.setPosition(0.5);
//            dropper.setPosition(0.45);
//
//            if(vClose.getState()) {
//                vSlide.setPower(-0.65);
//
//            }
//            else {
//                vSlide.setPower(-0.01);
//
//            }
//            if(hClose.getState()) {
//                hSlide.setPower(-0.8);
//                telemetry.addLine("closing");
//            }
//            else {
//                hSlide.setPower(-0.01);
//            }
//            telemetry.addData("hSlide", hSlide.getCurrentPosition());
//            telemetry.addData("hopening", hOpen.getState());
//            telemetry.addData("hclosing", hClose.getState());
//            telemetry.addData("vopening", vOpen.getState());
//            telemetry.addData("vclosing", vClose.getState());
//            telemetry.update();
//        }
//        hSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        hSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        vSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        vSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        telemetry.update();
//        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//            // main part of code: chassis driving
//            //when play button pressed
//
//            switch (sState) {
//                case init:
//
//                    telemetry.addLine("initialized");
//                    telemetry.update();
//                    vSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    vSlide.setPower(-0.01);
//                    if(vClose.getState()) {
//                        vSlide.setPower(-0.65);
//
//                    }
//                    if(gamepad2.right_bumper){
//                        sState = org.firstinspires.ftc.teamcode.TeleOp.Gdrive.State.forward;
//                    }
//                    if (gamepad2.b){
//                        sState = org.firstinspires.ftc.teamcode.TeleOp.Gdrive.State.low;
//                    }
//                    break;
//                case forward:
//                    hSlide.setTargetPosition(1050);
//                    hSlide.setPower(1.0);
//                    hSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                    lRotate.setPosition(0.90);
//                    rRotate.setPosition(0.10);
//                    telemetry.addData("position", hSlide.getTargetPosition());
//                    telemetry.update();
//                    if(vClose.getState()) {
//                        vSlide.setPower(-0.65);
//
//                    }
//                    if (hSlide.getCurrentPosition()>=940) {
//                        sState = org.firstinspires.ftc.teamcode.TeleOp.Gdrive.State.grab;
//                        hSlide.setPower(0.01);
//                        hSlide.setZeroPowerBehavior(hSlide.getZeroPowerBehavior());
//                    }
//                    break;
//                case grab:
//                    if (gamepad2.left_bumper){
//                        sState = org.firstinspires.ftc.teamcode.TeleOp.Gdrive.State.backward;
//                    }
//                    telemetry.addLine("exited");
//                    telemetry.update();
//                    break;
//                case backward:
//                    hSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//                    grabber.setPosition(0);
//                    sleep(500);
//                    hSlide.setPower(-0.8);
//                    lRotate.setPosition(0.5);
//                    rRotate.setPosition(0.5);
//                    if (!hClose.getState()){
//                        sState = org.firstinspires.ftc.teamcode.TeleOp.Gdrive.State.loaded;
//                        hSlide.setPower(-0.01);
//                        hSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//                        hSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//                    }
//                    break;
//                case loaded:
//                    hSlide.setPower(-0.01);
//                    lRotate.setPosition(0.1);
//                    rRotate.setPosition(0.9);
//                    sleep(100);
//                    grabber.setPosition(0.7);
//                    hSlide.setPower(0.01);
//                    sleep(300);
//                    lRotate.setPosition(0.5);
//                    rRotate.setPosition(0.5);
////                    dropper.setPosition(0.5);
//                    sState = org.firstinspires.ftc.teamcode.TeleOp.Gdrive.State.dropWait;
//                    break;
//                case dropWait:
//                    if(gamepad2.right_bumper){
//                        dropper.setPosition(0.45);
//                        sState = org.firstinspires.ftc.teamcode.TeleOp.Gdrive.State.forward;
//                    }
//                    if(gamepad2.dpad_up) {
//                        sState = org.firstinspires.ftc.teamcode.TeleOp.Gdrive.State.h3;
//                        telemetry.addLine("case h3");
//                        telemetry.update();
//                    }
//                    if(gamepad2.dpad_left){
//                        sState = org.firstinspires.ftc.teamcode.TeleOp.Gdrive.State.h2;
//                        telemetry.addLine("case h2");
//                        telemetry.update();
//                    }
//                    if(gamepad2.dpad_right){
//                        sState = org.firstinspires.ftc.teamcode.TeleOp.Gdrive.State.h1;
//                        telemetry.addLine("case h1");
//                        telemetry.update();
//                    }
//                    if(gamepad2.b){
//                        sState = org.firstinspires.ftc.teamcode.TeleOp.Gdrive.State.low;
//                        telemetry.addLine("case low");
//                        telemetry.update();
//                    }
//                    break;
//                case h3:
//                    vSlide.setPower(1);
//
//                    telemetry.addData("vOpen", !vOpen.getState());
//                    if(!vOpen.getState()) {
//                        sState = org.firstinspires.ftc.teamcode.TeleOp.Gdrive.State.hold;
//                    }
//                    break;
//                case h2:
//                    vSlide.setPower(1);
//                    vSlide.setTargetPosition(1000);
//                    vSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                    telemetry.addData("vOpen", !vOpen.getState());
//                    if(vSlide.getCurrentPosition()>=1000) {
//                        sState = org.firstinspires.ftc.teamcode.TeleOp.Gdrive.State.hold;
//                    }
//                case h1:
//                    telemetry.addData("low junction activated", !vOpen.getState());
//                    sleep(500);
//                    sState = org.firstinspires.ftc.teamcode.TeleOp.Gdrive.State.dump;
//                    break;
//                case low:
//                    telemetry.addLine("low");
//                    telemetry.update();
//                    lRotate.setPosition(0.90);
//                    rRotate.setPosition(0.10);
//                    sleep(300);
//                    sState = Gdrive.State.lowOpen;
//
//                    break;
//                case lowOpen:
//                    if(gamepad2.a) {
//                        grabber.setPosition(0.0);
//                        sleep(400);
//                        lRotate.setPosition(0.5);
//                        rRotate.setPosition(0.5);
//                        sState = Gdrive.State.lowDrop; }
//                    break;
//                case lowDrop:
//                    if(gamepad2.a) {
//                        lRotate.setPosition(0.82);
//                        rRotate.setPosition(0.18);
//                        sState = Gdrive.State.lowReset;
//                    }
//                    break;
//
//                case lowReset:
//
//                    if (gamepad2.b){
//                        grabber.setPosition(0.7);
//                        lRotate.setPosition(0.5);
//                        rRotate.setPosition(0.5);
//                        sState = Gdrive.State.init;
//                    }
//
//
//                    break;
//
//                case hold:
//                    telemetry.addLine("exited");
//                    telemetry.update();
//                    vSlide.setPower(0.03);
//
//                    if(gamepad2.dpad_down){
//                        sState = org.firstinspires.ftc.teamcode.TeleOp.Gdrive.State.dump;
//                    }
//                    break;
//                case dump:
//                    vSlide.setPower(0.01);
//                    dropper.setPosition(1.0);
//                    sleep(500);
//                    sState = org.firstinspires.ftc.teamcode.TeleOp.Gdrive.State.reset;
//                    break;
//                case reset:
//                    telemetry.addLine("closing");
//                    telemetry.update();
//                    grabber.setPosition(0.7);
//                    lRotate.setPosition(0.5);
//                    rRotate.setPosition(0.5);
//                    dropper.setPosition(0.45);
//
//                    sState = org.firstinspires.ftc.teamcode.TeleOp.Gdrive.State.init;
//                    break;
//
//
//            }
//            // main part of code: chassis driving
//            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            double r = Math.hypot(gamepad2.left_stick_y,-gamepad2.left_stick_x);
//            double robotAngle = Math.atan2(-gamepad2.left_stick_y, gamepad2.left_stick_x) - Math.PI / 4  - ((angles.firstAngle/180)*Math.PI);
//            double rightX = (gamepad2.left_trigger-gamepad2.right_trigger)/2;
//            final double v1 = r * Math.cos(robotAngle) + rightX;
//            final double v2 = r * Math.sin(robotAngle) - rightX;
//            final double v3 = r * Math.sin(robotAngle) + rightX;
//            final double v4 = r * Math.cos(robotAngle) - rightX;
//
//            if (gamepad2.left_stick_y != 0 || gamepad2.left_stick_x != 0 || gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0 ) {
//
//                fl.setPower(v1);
//                fr.setPower(v2);
//                rl.setPower(v3);
//                rr.setPower(v4);
//            }
//            else{
//                brake();
//
//            }
//            telemetry.update();
//        }
//    }
//
//
//
//
//
//    public void brake() {
//
//        fl.setPower(0.0);
//        fr.setPower(0.0);
//        rl.setPower(0.0);
//        rr.setPower(0.0);
//        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        sleep(100);
//
//
//    }
//}