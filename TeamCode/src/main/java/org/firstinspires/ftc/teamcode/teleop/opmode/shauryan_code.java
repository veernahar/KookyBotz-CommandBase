package org.firstinspires.ftc.teamcode.teleop.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.DumpIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.DumpOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.turretcommand.TurretIntakeCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@TeleOp
public class shauryan_code extends CommandOpMode {
    private Robot robot;


    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);

//        schedule(
//                new SequentialCommandGroup(
//                        new OuttakeArmCommand(robot.arm),
//                        new ParallelCommandGroup(
//                                new LiftExtendCommand(robot.lift),
//                                new OuttakeOuttakeCommand(robot.outtake)
//                        )
//
//                )
//        );

        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        gamepadEx.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> schedule(
                new SequentialCommandGroup(
                        new TurretIntakeCommand(robot.turret),
                        new WaitCommand(1000),
                        new ArmIntakeCommand(robot.arm),
                        new DumpIntakeCommand(robot.dump),
                        new WaitCommand(2000),
                        new ArmOuttakeCommand(robot.arm),
                        new WaitCommand(1000),
                        new DumpOuttakeCommand(robot.dump),
                        new WaitCommand(1000),
                        new LiftExtendCommand(robot.lift),
                        new WaitCommand(2000),
                        new LiftRetractCommand(robot.lift)
                )
        ));


    }

    @Override
    public void run() {
        super.run();
    }

}
