package org.firstinspires.ftc.teamcode.COMMAND_GROUPS;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.COMMANDS.gRotateDownCommand;
import org.firstinspires.ftc.teamcode.COMMANDS.gRotateUpCommand;
import org.firstinspires.ftc.teamcode.COMMANDS.gimbleHorizontalCommand;
import org.firstinspires.ftc.teamcode.COMMANDS.gimbleVerticalUpCommand;
import org.firstinspires.ftc.teamcode.COMMANDS.grabberCloseCommand;
import org.firstinspires.ftc.teamcode.COMMANDS.grabberOpenCommand;
import org.firstinspires.ftc.teamcode.GLOBALS.subConstants;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.intake_sub;

public class intakeUp extends SequentialCommandGroup {

    subConstants sc = new subConstants();

    public intakeUp(intake_sub intake_sub){
        addCommands(
                new grabberCloseCommand(intake_sub),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        new gRotateUpCommand(intake_sub),
                        new gimbleVerticalUpCommand(intake_sub))
        );
        addRequirements(intake_sub);
    }

}
