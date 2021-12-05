package org.firstinspires.ftc.teamcode.teleop.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommands.OuttakeAndResetCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommands.StopIntakeAndExtendCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.ff.ALLIANCE;
import org.firstinspires.ftc.teamcode.common.ff.MODE;
import org.firstinspires.ftc.teamcode.common.ff.STATE;

@TeleOp
public class teleop extends CommandOpMode {
    private Robot robot;
    private GamepadEx gamepadEx1, gamepadEx2;
    private STATE state = STATE.INTAKE;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot.lift.intake();
        robot.arm.intake();
        robot.dump.intake();
        robot.turret.intake();
        robot.intake.start();

        gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                () -> {
                    schedule(
                            state == STATE.REST ?
                                    new OuttakeAndResetCommand(robot.lift, robot.arm, robot.intake, robot.dump)
                                    : new InstantCommand()
                    );
                    state = state.next();
                }
        );
    }

    @Override
    public void run() {
        super.run();

        // drive lol
        robot.drive.driveRobotCentric(
                gamepadEx1.getLeftX(),
                gamepadEx1.getLeftY(),
                gamepadEx1.getRightX()
        );

        if (robot.intake.hasFreight() && state == STATE.INTAKE) {
            schedule(new StopIntakeAndExtendCommand(robot.lift, robot.arm, robot.intake, robot.dump));
            state = state.next();
        }
    }
}
