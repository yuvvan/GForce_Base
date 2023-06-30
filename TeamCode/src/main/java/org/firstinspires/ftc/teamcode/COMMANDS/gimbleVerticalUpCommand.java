package org.firstinspires.ftc.teamcode.COMMANDS;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SUBSYSTEMS.intake_sub;

public class gimbleVerticalUpCommand extends CommandBase {

    private intake_sub intake_sub;

    public gimbleVerticalUpCommand(intake_sub subsystem){
        intake_sub = subsystem;
        addRequirements(intake_sub);
    }
    public void initialize(){
        intake_sub.gimbleVerticalUp();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
