package org.firstinspires.ftc.teamcode.COMMAND_GROUPS;


import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.COMMANDS.gRotateDownCommand;
import org.firstinspires.ftc.teamcode.COMMANDS.gimbleHorizontalCommand;
import org.firstinspires.ftc.teamcode.COMMANDS.grabberOpenCommand;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.intake_sub;
import org.firstinspires.ftc.teamcode.GLOBALS.subConstants;

public class intakeDown extends SequentialCommandGroup {

    subConstants sc = new subConstants();

    public intakeDown(intake_sub intake_sub){
        addCommands(
                new ParallelCommandGroup(
                        new gRotateDownCommand(intake_sub),
                        new gimbleHorizontalCommand(intake_sub)),
                new grabberOpenCommand(intake_sub)
        );
        addRequirements(intake_sub);
    }

}
