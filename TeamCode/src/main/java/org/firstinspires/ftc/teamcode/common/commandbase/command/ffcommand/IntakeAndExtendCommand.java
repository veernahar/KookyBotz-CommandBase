package org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.DumpCloseCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DumpSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class IntakeAndExtendCommand extends SequentialCommandGroup {
    public IntakeAndExtendCommand(DumpSubsystem dump, LiftSubsystem lift, ArmSubsystem arm, IntakeSubsystem intake) {
        super(
                new WaitCommand(150),
                new DumpCloseCommand(dump),
                new IntakeStopCommand(intake),
                new WaitCommand(250),
                new ArmOuttakeCommand(arm),
                new WaitCommand(250),
                new LiftExtendCommand(lift)
        );
    }
}
