package org.firstinspires.ftc.teamcode.common.commandbase.command.duckcommand;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DuckSubsystem;

public class DuckFastBlueCommand extends SequentialCommandGroup {

    public DuckFastBlueCommand(DuckSubsystem duck) {
        super(
                new DuckBlueCommand(duck),
                new WaitCommand(750),
                new InstantCommand(() -> duck.setPower(-1)),
                new WaitCommand(500),
                new DuckOffCommand(duck)
        );
    }
}
