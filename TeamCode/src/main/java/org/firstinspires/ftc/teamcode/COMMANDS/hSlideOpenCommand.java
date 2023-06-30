package org.firstinspires.ftc.teamcode.COMMANDS;

import com.arcrobotics.ftclib.command.CommandBase;

public class hSlideOpenCommand extends CommandBase {

    private org.firstinspires.ftc.teamcode.SUBSYSTEMS.intake_sub intake_sub;

    public hSlideOpenCommand(org.firstinspires.ftc.teamcode.SUBSYSTEMS.intake_sub subsystem){
        intake_sub = subsystem;
        addRequirements(intake_sub);
    }
    public void initialize(){
        intake_sub.hSlideOpen();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
