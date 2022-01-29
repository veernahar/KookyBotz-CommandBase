package org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmSharedCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.DumpCloseCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.turretcommand.TurretOuttakeBlueCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.turretcommand.TurretOuttakeRedCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DumpSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;
import org.firstinspires.ftc.teamcode.common.ff.ALLIANCE;

public class IntakeAndExtendSharedCommand extends SequentialCommandGroup {
    public IntakeAndExtendSharedCommand(ALLIANCE alliance, LiftSubsystem lift, ArmSubsystem arm, DumpSubsystem dump, TurretSubsystem turret, IntakeSubsystem intake) {
        if (alliance == ALLIANCE.RED) {
            addCommands(
                    new WaitCommand(150),
                    new IntakeStopCommand(intake),
                    new DumpCloseCommand(dump),
                    new WaitCommand(250),
                    new ArmOuttakeCommand(arm),
                    new WaitCommand(500),
                    new TurretOuttakeRedCommand(turret),
                    new WaitCommand(500),
                    new ArmSharedCommand(arm)
            );
        } else {
            addCommands(
                    new WaitCommand(150),
                    new IntakeStopCommand(intake),
                    new DumpCloseCommand(dump),
                    new WaitCommand(250),
                    new ArmOuttakeCommand(arm),
                    new WaitCommand(500),
                    new TurretOuttakeBlueCommand(turret),
                    new WaitCommand(500),
                    new ArmSharedCommand(arm)
            );
        }
    }
}
