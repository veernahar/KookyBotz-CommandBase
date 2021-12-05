package org.firstinspires.ftc.teamcode.teleop.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommands.specific.CloseIntakeAndExtendSpecificCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommands.specific.OuttakeAndResetSpecificCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.StartIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.outtakecommand.OuttakeOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.ff.ALLIANCE;
import org.firstinspires.ftc.teamcode.common.ff.MODE;
import org.firstinspires.ftc.teamcode.common.ff.STATE;
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
