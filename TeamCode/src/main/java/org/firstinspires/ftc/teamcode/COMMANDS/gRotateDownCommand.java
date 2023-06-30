package org.firstinspires.ftc.teamcode.COMMANDS;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SUBSYSTEMS.intake_sub;

public class gRotateDownCommand extends CommandBase {

    private org.firstinspires.ftc.teamcode.SUBSYSTEMS.intake_sub intake_sub;

    public gRotateDownCommand(org.firstinspires.ftc.teamcode.SUBSYSTEMS.intake_sub subsystem){
        intake_sub = subsystem;
        addRequirements(intake_sub);
    }
    public void initialize(){
        intake_sub.gRotateDown();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
