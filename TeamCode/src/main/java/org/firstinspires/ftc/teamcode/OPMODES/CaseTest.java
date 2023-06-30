package org.firstinspires.ftc.teamcode.OPMODES;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp
public class CaseTest extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;
    DcMotor fl;
    DcMotor fr;
    DcMotor rl;
    DigitalChannel hClose;
    DigitalChannel vClose;
    DcMotor rr;
    DcMotorEx vSlide;
    DcMotor hSlide;
    DcMotor turnTable;
    Servo aligner;
    Servo gimble;
    Servo grabber;
    DcMotor gRotate;
    AnalogInput gPot;
    Servo dRotate;
    Servo dropper;

    //    PID CODE BITS
    private PIDController vs_controller;
    public static double vsp =0.05, vsi = 0, vsd = 0.0008;

    public static double kP = 0.045;
    public static double kI = 0;
    public static double kD = 0.00035;
    // When battery low, kD = 0.00055;
    // When battery high, kD = 0.00035;
    public static double kF = 0;

    public static int targetPosition = 1000;
    public static int midtargetPosition = 700;

    int tolerance = 5;

    private PIDFController pidfController;

    double motorPower;

    boolean hCloseValue = false;

    int hCloseCounter = 0;

    public enum State {
        init,
        grabberDown,
        h_slide_forward,
        grab,
        grabberUp,
        loaded,
        high,
        retract,
        mid,
        low,
        holdForDrop,
        groundHold,
        groundDrop,
        h_slide_backward,
        neutral;
    }

    CaseTest.State sState = State.init;

    @Override
    public void runOpMode() throws InterruptedException {

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rl = hardwareMap.get(DcMotor.class, "rl");
        rr = hardwareMap.get(DcMotor.class, "rr");
        grabber = hardwareMap.get(Servo.class, "grabber");
        gimble = hardwareMap.get(Servo.class, "gimble");
        gRotate = hardwareMap.get(DcMotor.class, "gRotate");
        gPot = hardwareMap.get(AnalogInput.class, "gPot");
        vClose = hardwareMap.get(DigitalChannel.class, "vClose");
        hClose = hardwareMap.get(DigitalChannel.class, "hClose");
        vSlide = hardwareMap.get(DcMotorEx.class, "vSlide");
        hSlide = hardwareMap.get(DcMotor.class, "hSlide");
        aligner = hardwareMap.get(Servo.class, "aligner");
        turnTable = hardwareMap.get(DcMotor.class, "tTable");
        dRotate = hardwareMap.get(Servo.class, "dRotate");
        dropper = hardwareMap.get(Servo.class, "dropper");

        //SETTING UP DC MOTORS (NON-MOVEMENT)
        gRotate.setDirection(DcMotorSimple.Direction.REVERSE);
        vSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        hSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        hSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        vSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turnTable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //SETTING UP MOVEMENT MOTORS

        fl.setDirection(DcMotor.Direction.REVERSE);
        rl.setDirection(DcMotor.Direction.REVERSE);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //imu init
        setImu();
//        DECLARING PID CONTROLLER
        vs_controller = new PIDController(vsp, vsi , vsd);

//        CREATING VARIABLES
        int count_y = 0;
        int vSPos;

        pidfController = new PIDFController(kP, kI, kD, kF);

        while ((!isStopRequested()) && (!isStarted())) {
            dRotate.setPosition(1.0);
            dropper.setPosition(0.55);
            while(gPot.getVoltage()<1.15) {
                gRotate.setPower(0.8);
            }
            gRotate.setPower(0.01);
            gRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            gimble.setPosition(0.8);
            aligner.setPosition(0.5);
            hSlideRetract();
            hCloseValue = hClose.getState();
            telemetry.addData("Status", "Initialized");
            telemetry.addData("hClose", hCloseValue);
            telemetry.update();
        }
        while (opModeIsActive()){

            switch (sState){
                //Init state runs when play button is pressed
                case init:
                    telemetry.addLine("Case: Init");
                    telemetry.addData("VSlide Position", vSlide.getCurrentPosition());
                    telemetry.addData("pot Voltage", gPot.getVoltage());
                    telemetry.addData("tTable: ", turnTable.getCurrentPosition());
                    telemetry.addData("hClose", hCloseValue);
                    turnTable.setTargetPosition(0);
                    turnTable.setPower(0.4);
                    turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hCloseValue = hClose.getState();
                    hSlideRetract();
                    vSlide.setPower(0.01);
                    telemetry.update();
                    if(gamepad2.right_bumper){
                        sState = State.grabberDown;
                    }
                    if(gamepad2.x || gamepad1.x){
                        setImu();
                    }

                    break;

                //Makes grabber arm go down
                case grabberDown:
                    dropper.setPosition(0.55);
                    gimble.setPosition(0.65);
                    while (gPot.getVoltage()>0.7){
                        gRotate.setPower(-0.9);
                    }
                    gRotate.setPower(0.05);
                    grabber.setPosition(0.7);
                    sState = State.h_slide_forward;
                    break;

                    // hslide moves forward on hold
                case h_slide_forward:
                    while(gamepad2.right_bumper){hSlide.setPower(0.8);}
                    hSlide.setPower(0);
                    hSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    if(gamepad2.left_bumper){sState = State.grab;}
                    break;

                    //hslide moves backward on hold
                case h_slide_backward:
                    hSlide.setPower(-0.8);
                    if(gamepad2.x || gamepad1.x){sState = State.init;}
                    sState = State.neutral;
                    break;

                    //grab cone from ground
                case grab:
                    grabber.setPosition(0.4);
                    sleep(250);
                    sState = State.grabberUp;
                    break;

                    // gRotate rotates back up
                case grabberUp:
                    while(gPot.getVoltage()<1.2) {
                        gRotate.setPower(0.9);
                    }
                    gRotate.setPower(0.01);
                    gRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    gimble.setPosition(0.65);
                    if(gamepad2.x || gamepad1.x){sState = State.init;}
                    else if(gamepad1.left_bumper){ sState = State.low; }
                    else if(gamepad1.right_bumper){sState = State.groundHold;}
                    else if(gamepad1.y){sState = State.loaded;}
                    sState = State.h_slide_backward;
                    break;

                    // 90 degree neutral position to drop cone
                case neutral:
                    while(gPot.getVoltage()<1.2) {
                        gRotate.setPower(0.9);
                    }
                    gRotate.setPower(0.01);
                    gRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    gimble.setPosition(0.65);
                    if(gamepad2.x){sState = State.init;}
                    else if(gamepad1.dpad_left){ sState = State.low;}
                    else if(gamepad1.a){sState = State.groundHold;}
                    else if(gamepad1.y){sState = State.loaded;}
                    break;

                    // transfer cone from grabber to dropper
                case loaded:
                    dRotate.setPosition(1);
                    gimble.setPosition(0.9);
                    sleep(200);
                    grabber.setPosition(0.7);
                    sleep(300);
                    dropper.setPosition(0.45);
                    sleep(100);
                    gimble.setPosition(0.65);
                    sState = State.holdForDrop;
                    break;

                    // cone in dopper for high or mid junction
                case holdForDrop:
                    if(gamepad2.x){sState = State.init;}
                    else if(gamepad1.dpad_up){sState = State.high;}
                    else if(gamepad1.dpad_right){sState = State.mid;}
                    break;

                    // high junction
                case high:
                    aligner.setPosition(0.3);
                    while ((Math.abs(vSlide.getCurrentPosition() - targetPosition) > tolerance) && !gamepad1.dpad_down) {
                        // Calculate motor power
                        motorPower = pidfController.calculate(vSlide.getCurrentPosition(), targetPosition);
                        // Ensure power is between -1.0 and 1.0
                        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));
                        vSlide.setPower(motorPower);
                    }
                    vSlide.setPower(0);
                    vSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    dRotate.setPosition(0);

                    if(gamepad1.dpad_down){
                        sState = State.retract;
                    }

                    break;

                    // mid junction
                case mid:
                    aligner.setPosition(0.3);
                    while ((Math.abs(vSlide.getCurrentPosition() - midtargetPosition) > tolerance) && !gamepad1.dpad_down) {
                        // Calculate motor power
                        motorPower = pidfController.calculate(vSlide.getCurrentPosition(), midtargetPosition);
                        // Ensure power is between -1.0 and 1.0
                        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));
                        vSlide.setPower(motorPower);
                    }
                    vSlide.setPower(0);
                    vSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    dRotate.setPosition(0);
                    if(gamepad1.dpad_down){
                        sState = State.retract;
                    }

                    break;

                    // retract vslide
                case retract:
                    dropper.setPosition(0.55);
                    sleep(300);
                    dRotate.setPosition(1);
                    aligner.setPosition(0.5);
                    while (vClose.getState()) {
                        telemetry.addLine("back");
                        vSlide.setPower(-0.5);
                    }
                    vSlide.setPower(0);
                    sState = State.init;
                    break;


                    // low junction
                case low:
                    gimble.setPosition(0.3);
                    sleep(250);
                    grabber.setPosition(0.7);
                    sleep(500);
                    gimble.setPosition(0.65);
                    sState = State.init;
                    break;

                    // hover over ground junction
                case groundHold:
                    gimble.setPosition(0.65);
                    while (gPot.getVoltage()>0.8){
                        gRotate.setPower(-0.6);
                    }
                    gRotate.setPower(0.05);
                    if(gamepad1.b){
                        sState = State.groundDrop;
                    }
                    break;

                    // drop cone on ground junction
                case groundDrop:
                    grabber.setPosition(0.7);
                    sleep(500);
                    while(gPot.getVoltage()<1.2) {
                        gRotate.setPower(0.8);
                    }
                    gRotate.setPower(0.01);
                    gRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    sState = State.init;
            }

            // chassis movement code
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double r = Math.hypot(gamepad2.left_stick_y,-gamepad2.left_stick_x);
            double robotAngle = Math.atan2(-gamepad2.left_stick_y, gamepad2.left_stick_x) - Math.PI / 4  - ((angles.firstAngle/180)*Math.PI);
            double rightX = (gamepad2.left_trigger-gamepad2.right_trigger)/2;
            final double v1 = r * Math.cos(robotAngle) - rightX;
            final double v2 = r * Math.sin(robotAngle) + rightX;
            final double v3 = r * Math.sin(robotAngle) - rightX;
            final double v4 = r * Math.cos(robotAngle) + rightX;

            if (gamepad2.left_stick_y != 0 || gamepad2.left_stick_x != 0 || gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0 ) {

                if (gamepad1.atRest()) {

                    fl.setPower(1.5 * v1);
                    fr.setPower(1.5 * v2);
                    rl.setPower(1.5 * v3);
                    rr.setPower(1.5 * v4);
                }
            }
            else {
                fl.setPower(0.0);
                fr.setPower(0.0);
                rl.setPower(0.0);
                rr.setPower(0.0);
                fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }

    // function to retract hSlide
    public void hSlideRetract(){
        if (!hCloseValue){
            hCloseCounter = 1;
        }

        if (hCloseCounter == 1){
            hSlide.setPower(-0.4);
        }
        else {
            hSlide.setPower(-0.01);
            hSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    // imu
    public void setImu(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

}

// added gamepad1.atRest()
// imu reset button
// gPot voltage < 1.15 changed in neutral and grabberUp
// hslide back deleted while loop
// increased power of chassis to 1.5*v1...

