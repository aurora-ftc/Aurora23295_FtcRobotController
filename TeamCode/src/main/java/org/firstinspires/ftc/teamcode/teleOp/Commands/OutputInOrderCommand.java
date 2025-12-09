package org.firstinspires.ftc.teamcode.teleOp.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.teleOp.subSystems.BallSelector;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.Colors;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.Mosaic;

public class OutputInOrderCommand extends SequentialCommandGroup {
    public OutputInOrderCommand(BallSelector selector, Mosaic mosaic) {
        for (char c : mosaic.toString().toCharArray()) {
            Colors color = (c == 'p') ? Colors.Purple:Colors.Green;
            addCommands(
                    new InstantCommand(selector::getColour),
                    new InstantCommand(() -> selector.returnBall(color)),
                    new WaitUntilCommand(selector::isAtTarget),
                    new InstantCommand(selector::push)
            );
        }
    }
}
