package org.firstinspires.ftc.teamcode.COMMANDS;

import com.arcrobotics.ftclib.command.CommandBase;

public class hSlideCloseCommand extends CommandBase {

    private org.firstinspires.ftc.teamcode.SUBSYSTEMS.intake_sub intake_sub;

    public hSlideCloseCommand(org.firstinspires.ftc.teamcode.SUBSYSTEMS.intake_sub subsystem){
        intake_sub = subsystem;
        addRequirements(intake_sub);
    }
    public void initialize(){
        intake_sub.hSlideClose();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
