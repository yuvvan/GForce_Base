package org.firstinspires.ftc.teamcode.OPMODES;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.COMMANDS.gRotateDownCommand;
import org.firstinspires.ftc.teamcode.COMMANDS.gRotateUpCommand;
import org.firstinspires.ftc.teamcode.COMMANDS.gimbleHorizontalCommand;
import org.firstinspires.ftc.teamcode.COMMANDS.gimbleVerticalUpCommand;
import org.firstinspires.ftc.teamcode.COMMANDS.grabberCloseCommand;
import org.firstinspires.ftc.teamcode.COMMANDS.grabberOpenCommand;
import org.firstinspires.ftc.teamcode.COMMANDS.hSlideCloseCommand;
import org.firstinspires.ftc.teamcode.COMMANDS.hSlideOpenCommand;
import org.firstinspires.ftc.teamcode.COMMAND_GROUPS.intakeDown;
import org.firstinspires.ftc.teamcode.COMMAND_GROUPS.intakeUp;
import org.firstinspires.ftc.teamcode.COMMAND_GROUPS.load;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.intake_sub;


@TeleOp(name = "Command TeleOp")
public class CommandBaseTele extends CommandOpMode {

    private GamepadEx drivePad, mechPad;
    private intake_sub intake;
    BNO055IMU imu;
    Orientation angles;
    private DcMotorEx fl;
    private DcMotorEx fr;
    private DcMotorEx rl;
    private DcMotorEx rr;

    private gimbleHorizontalCommand gimbleHorizontalCommand;
    private gimbleVerticalUpCommand gimbleVerticalUpCommand;
    private grabberCloseCommand grabberCloseCommand;
    private grabberOpenCommand grabberOpenCommand;
    private gRotateDownCommand gRotateDownCommand;
    private gRotateUpCommand gRotateUpCommand;
    private hSlideCloseCommand hSlideCloseCommand;
    private hSlideOpenCommand hSlideOpenCommand;

    private intakeDown intakeDown;
    private intakeUp intakeUp;
    private load load;

    private Button
    intakeDownBtn,
    intakeUpBtn,
    loadBtn,
    highBtn,
    midBtn,
    lowBtn,
    groundHoldBtn,
    groundDropBtn;



    @Override
    public void initialize() {
        drivePad = new GamepadEx(gamepad2);
        mechPad = new GamepadEx(gamepad1);

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        rl = hardwareMap.get(DcMotorEx.class, "rl");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        intake = new intake_sub(
                hardwareMap,
                "hSlide",
                "grabber",
                "gimble",
                "gRotate",
                "gPot" );
        // Command initialization
        gimbleHorizontalCommand = new gimbleHorizontalCommand(intake);
        gimbleVerticalUpCommand = new gimbleVerticalUpCommand(intake);
        grabberCloseCommand = new grabberCloseCommand(intake);
        grabberOpenCommand = new grabberOpenCommand(intake);
        gRotateDownCommand = new gRotateDownCommand(intake);
        gRotateUpCommand = new gRotateUpCommand(intake);
        hSlideOpenCommand = new hSlideOpenCommand(intake);
        hSlideCloseCommand = new hSlideCloseCommand(intake);

        //Command Group initialization
        intakeDown = new intakeDown(intake);
        intakeUp = new intakeUp(intake);

        intakeDownBtn = (new GamepadButton(drivePad, GamepadKeys.Button.RIGHT_BUMPER))
                .whenPressed(intakeDown);
        intakeUpBtn = (new GamepadButton(drivePad, GamepadKeys.Button.LEFT_BUMPER))
                .whenPressed(intakeUp);

    }

    public void run(){
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double r = Math.hypot(drivePad.getLeftY(),-drivePad.getLeftX());
        double robotAngle = Math.atan2(-drivePad.getLeftY(), drivePad.getLeftX()) - Math.PI / 4  - ((angles.firstAngle/180)*Math.PI);
        double rightX = (drivePad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)-drivePad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER))/2;
        final double v1 = r * Math.cos(robotAngle) - rightX;
        final double v2 = r * Math.sin(robotAngle) + rightX;
        final double v3 = r * Math.sin(robotAngle) - rightX;
        final double v4 = r * Math.cos(robotAngle) + rightX;

        if (drivePad.getLeftY() != 0 || drivePad.getLeftX() != 0 || drivePad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0 || drivePad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0 ) {

                fl.setPower(1.5 * v1);
                fr.setPower(1.5 * v2);
                rl.setPower(1.5 * v3);
                rr.setPower(1.5 * v4);

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
