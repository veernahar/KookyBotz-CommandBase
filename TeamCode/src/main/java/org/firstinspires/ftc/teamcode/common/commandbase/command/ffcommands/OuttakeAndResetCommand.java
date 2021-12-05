package org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.DumpIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.DumpOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DumpSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class OuttakeAndResetCommand extends SequentialCommandGroup {
    public OuttakeAndResetCommand(LiftSubsystem lift, ArmSubsystem arm, IntakeSubsystem intake, DumpSubsystem dump) {
        super(
                new DumpOuttakeCommand(dump),
                new WaitCommand(1000),
                new DumpIntakeCommand(dump),
                new LiftRetractCommand(lift),
                new WaitCommand(1000),
                new ArmIntakeCommand(arm),
                new WaitCommand(1000),
                new IntakeStartCommand(intake)
        );
    }
}
