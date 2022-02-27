package org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.DumpIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.DumpOpenCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.DumpOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DumpSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class OuttakeAndResetAutoCommand extends SequentialCommandGroup {
    public OuttakeAndResetAutoCommand(DumpSubsystem dump, LiftSubsystem lift, ArmSubsystem arm, IntakeSubsystem intake) {
        super(
                new DumpOpenCommand(dump),
                new DumpOuttakeCommand(dump),
                new WaitCommand(500),
                new DumpIntakeCommand(dump),
                new LiftRetractCommand(lift),
                new WaitCommand(250),
                new ArmIntakeCommand(arm)
        );
    }
}
