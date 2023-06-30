package org.firstinspires.ftc.teamcode.SUBSYSTEMS;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class drivetrain_sub extends SubsystemBase {

    private DcMotorEx fl;
    private DcMotorEx fr;
    private DcMotorEx rl;
    private DcMotorEx rr;

    public drivetrain_sub(
            HardwareMap hMap,
            final String flName,
            final String frName,
            final String rrName,
            final String rlName
            ){
        fl = hMap.get(DcMotorEx.class, flName);
        fr = hMap.get(DcMotorEx.class, frName);
        rl = hMap.get(DcMotorEx.class, rlName);
        rr = hMap.get(DcMotorEx.class, rrName);
    }
    public void drive(){

    }

}
