package org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.DumpIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.DumpOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.DumpRestCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DumpSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class StopIntakeAndExtendCommand extends SequentialCommandGroup {
    public StopIntakeAndExtendCommand(LiftSubsystem lift, ArmSubsystem arm, IntakeSubsystem intake, DumpSubsystem dump) {
        super(
                new WaitCommand(250),
                new IntakeStopCommand(intake),
                new DumpRestCommand(dump),
                new WaitCommand(250),
                new ArmOuttakeCommand(arm),
                new WaitCommand(1000),
                new LiftExtendCommand(lift)
        );
    }
}
