package org.firstinspires.ftc.teamcode.jCode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.jCode.SubSystems.Sorter;

public class testCommand extends CommandBase {
    Sorter sorter;

    public testCommand(Sorter s) {
        this.sorter = s;
    }

    @Override
    public void initialize() {
        sorter.setState();
    }

    @Override
    public void execute() {
        sorter.rotateDown();
    }

    @Override
    public boolean isFinished() {
        return sorter.atTarget();
    }
}
