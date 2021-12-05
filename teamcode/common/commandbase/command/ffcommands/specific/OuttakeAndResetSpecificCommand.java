package org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommands.specific;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.OuttakeIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.OuttakeOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.OuttakeSubsystem;

public class OuttakeAndResetSpecificCommand extends SequentialCommandGroup {

    public OuttakeAndResetSpecificCommand(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem,
                                          LiftSubsystem liftSubsystem, OuttakeSubsystem outtakeSubsystem) {

        addCommands(
                new OuttakeOuttakeCommand(outtakeSubsystem),
                new WaitCommand(1000),
                new OuttakeIntakeCommand(outtakeSubsystem),
                new LiftRetractCommand(liftSubsystem),
                new WaitCommand(750),
                new ArmIntakeCommand(armSubsystem),
                new WaitCommand(500),
                new IntakeStartCommand(intakeSubsystem)
        );

        addRequirements(intakeSubsystem, armSubsystem, liftSubsystem, outtakeSubsystem);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
