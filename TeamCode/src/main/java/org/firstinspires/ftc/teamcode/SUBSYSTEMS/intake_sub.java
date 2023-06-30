package org.firstinspires.ftc.teamcode.SUBSYSTEMS;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GLOBALS.subConstants;

public class intake_sub extends SubsystemBase {

    subConstants sc = new subConstants();

    private DcMotorEx hSlide;
    private Servo grabber;
    private Servo gimble;
    private DcMotor gRotate;
    private AnalogInput gPot;

    public intake_sub(
            final HardwareMap hMap,
            final String HSLIDE,
            final String GRABBER,
            final String GIMBLE,
            final String GROTATE,
            final String GPOT
    ){
        hSlide = hMap.get(DcMotorEx.class, HSLIDE);
        grabber = hMap.get(Servo.class, GRABBER);
        gimble = hMap.get(Servo.class, GIMBLE);
        gRotate = hMap.get(DcMotorEx.class, GROTATE);
        gPot = hMap.get(AnalogInput.class, GPOT);

    }

    public void hSlideOpen(){
        hSlide.setPower(0.8);
    }
    public void hSlideClose(){
        hSlide.setPower(-0.8);
    }

    public void grabberOpen(){
        grabber.setPosition(sc.grabberOpen);
    }
    public void grabberClose(){
        grabber.setPosition(sc.grabberClose);
    }

    public void gimbleVerticalUp(){
        gimble.setPosition(sc.gimbleVerticalUp);
    }
    public void gimbleHorizontal(){
        gimble.setPosition(sc.gimbleHorizontal);
    }
    public void gRotateDown(){
        while (gPot.getVoltage()> sc.gRotateDown){
            gRotate.setPower(-0.9);
        }
        gRotate.setPower(0.05);
    }
    public void gRotateUp(){
        while(gPot.getVoltage()<sc.gRotateUp) {
            gRotate.setPower(0.9);
        }
        gRotate.setPower(0.01);
        gRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
