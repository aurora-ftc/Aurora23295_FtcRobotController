package org.firstinspires.ftc.teamcode.teleOp.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.teleOp.subSystems.BallSelector;

public class IntakeBallCommand extends SequentialCommandGroup {
    public IntakeBallCommand(BallSelector selector) {
        addCommands(
                new InstantCommand(selector::getColour),
                new InstantCommand(selector::loadBall),
                new WaitUntilCommand(selector::isAtTarget)
        );
    }
}
