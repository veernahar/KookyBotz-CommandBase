package org.firstinspires.ftc.teamcode.teleop.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.DumpIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.DumpOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.DumpRest2Command;
import org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand.DumpRestCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftRetractCommand;
import org.firstinspires.ftc.teamcode.common.ff.STATE;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

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
                    if (state == STATE.REST) {
                        schedule(
                                new SequentialCommandGroup(
                                        new DumpOuttakeCommand(robot.dump),
                                        new WaitCommand(1000),
                                        new DumpIntakeCommand(robot.dump),
                                        new LiftRetractCommand(robot.lift),
                                        new WaitCommand(1000),
                                        new ArmIntakeCommand(robot.arm),
                                        new WaitCommand(1000),
                                        new IntakeStartCommand(robot.intake),
                                        new InstantCommand(() -> state = state.next())
                                )
                        );
                    }
                }
        );
    }

    @Override
    public void run() {
        super.run();

        // drive lol
        robot.drive.driveRobotCentric(
                scale(gamepadEx1.getLeftX(), 0.6) / ((gamepad1.right_trigger > 0.5) ? 1 : 1.5),
                scale(gamepadEx1.getLeftY(), 0.6) / ((gamepad1.right_trigger > 0.5) ? 1 : 1.5),
                scale(gamepadEx1.getRightX(), 0.6) / ((gamepad1.right_trigger > 0.5) ? 1 : 1.5)
        );

        if (robot.intake.hasFreight() && state == STATE.INTAKE) {
            System.out.println("has stuff");
            schedule(
                    new SequentialCommandGroup(
                            new WaitCommand(250),
                            new IntakeStopCommand(robot.intake),
                            new DumpRestCommand(robot.dump),
                            new WaitCommand(250),
                            new ParallelCommandGroup(
                                    new ArmOuttakeCommand(robot.arm),
                                    new WaitCommand(300).andThen(new DumpRest2Command(robot.dump))
                            ),
                            new WaitCommand(500),
                            new LiftExtendCommand(robot.lift)
                    )
            );
            state = state.next();
        }

        telemetry.addData("slide", robot.lift.getCurrentDrawA());
        telemetry.addData("intake", robot.intake.getCurrentDrawA());
        telemetry.update();
    }

    public double scale(double x, double k) {
        return (1 - k) * x + k * x * x * x;
    }
}
