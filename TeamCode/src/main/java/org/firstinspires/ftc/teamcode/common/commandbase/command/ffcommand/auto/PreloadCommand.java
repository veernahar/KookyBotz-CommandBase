package org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.DumpCloseCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.DumpIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.DumpOpenCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.DumpOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftSetPosCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DumpSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class PreloadCommand extends SequentialCommandGroup {
    public PreloadCommand(DumpSubsystem dump, LiftSubsystem lift, ArmSubsystem arm, IntakeSubsystem intake) {
        super(
                new ArmOuttakeCommand(arm),
                new WaitCommand(250),
                new LiftSetPosCommand(lift, 1000),
                new WaitCommand(500),
                new DumpOpenCommand(dump),
                new DumpOuttakeCommand(dump),
                new WaitCommand(500),
                new DumpIntakeCommand(dump),
                new LiftRetractCommand(lift),
                new WaitCommand(250),
                new ArmIntakeCommand(arm),
                new WaitCommand(1000),
                new IntakeStartCommand(intake)
        );
    }
}
