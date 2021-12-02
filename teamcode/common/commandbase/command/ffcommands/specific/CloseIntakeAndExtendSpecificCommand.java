package org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommands.specific;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeCloseCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeReverseCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.OuttakeSubsystem;

public class CloseIntakeAndExtendSpecificCommand extends SequentialCommandGroup {

    public CloseIntakeAndExtendSpecificCommand(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem,
                                               LiftSubsystem liftSubsystem, OuttakeSubsystem outtakeSubsystem) {

        addCommands(
                new WaitCommand(250),
                new IntakeCloseCommand(intakeSubsystem),
                new WaitCommand(500),
                new IntakeReverseCommand(intakeSubsystem),
                new ArmOuttakeCommand(armSubsystem),
                new WaitCommand(500),
                new LiftExtendCommand(liftSubsystem)
        );

        addRequirements(intakeSubsystem, armSubsystem, liftSubsystem, outtakeSubsystem);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
