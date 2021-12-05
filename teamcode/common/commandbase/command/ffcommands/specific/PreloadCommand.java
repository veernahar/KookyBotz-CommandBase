package org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommands.specific;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeReverseCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.OuttakeIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.OuttakeOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class PreloadCommand extends SequentialCommandGroup {
    public PreloadCommand(Robot robot) {
        super(new WaitCommand(500),
                new IntakeStopCommand(robot.intake),
                new ArmOuttakeCommand(robot.arm),
                new IntakeReverseCommand(robot.intake),
                new WaitCommand(450),
                new LiftExtendCommand(robot.lift),
                new WaitCommand(1500),
                new OuttakeOuttakeCommand(robot.outtake),
                new WaitCommand(1000),
                new OuttakeIntakeCommand(robot.outtake),
                new LiftRetractCommand(robot.lift),
                new WaitCommand(550),
                new ArmIntakeCommand(robot.arm),
                new IntakeStartCommand(robot.intake)
        );
    }
}
