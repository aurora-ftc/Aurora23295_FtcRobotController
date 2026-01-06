package org.firstinspires.ftc.teamcode.teleOp.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.teleOp.subSystems.BallSelector;
import org.firstinspires.ftc.teamcode.teleOp.util.Colors;

public class ReturnBallCommand extends SequentialCommandGroup {
    public ReturnBallCommand(BallSelector selector, Colors color) {
        addCommands(
                new InstantCommand(selector::getColour),
                new InstantCommand(() -> selector.returnBall(color)),
                new WaitUntilCommand(selector::isAtTarget),
                new InstantCommand(selector::push)
        );
    }
}
