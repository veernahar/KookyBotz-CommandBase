package org.firstinspires.ftc.teamcode.teleop.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.IntakeAndExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.IntakeAndExtendSharedCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.OuttakeAndResetCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.OuttakeAndResetSharedCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.ResetCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.common.ff.ALLIANCE;
import org.firstinspires.ftc.teamcode.common.ff.MODE;
import org.firstinspires.ftc.teamcode.common.ff.STATE;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.rr.AutonomousDrivetrain;

@Disabled
public class teleop extends CommandOpMode {
    private Robot robot;
    private AutonomousDrivetrain drive;
    private GamepadEx GamepadEx1, GamepadEx2;
    public STATE state = STATE.INTAKE;
    private MODE mode = MODE.SPECIFIC;
    public ALLIANCE alliance;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        drive = new AutonomousDrivetrain(hardwareMap, 0);
        GamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx2 = new GamepadEx(gamepad2);

        robot.lift.intake();
        robot.arm.intake();
        robot.dump.intake();
        robot.dump.open();
        robot.turret.intake();
        robot.intake.start();

        GamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                () -> {
                    if (state == STATE.REST) {
                        schedule(
                                mode == MODE.SHARED ?
                                        new SequentialCommandGroup(
                                                new OuttakeAndResetSharedCommand(robot.dump, robot.lift, robot.arm, robot.intake, robot.turret),
                                                new InstantCommand(() -> state = state.next())
                                        )
                                        :
                                        new SequentialCommandGroup(
                                                new OuttakeAndResetCommand(robot.dump, robot.lift, robot.arm, robot.intake),
                                                new InstantCommand(() -> state = state.next())
                                        )
                        );
                    }
                }
        );

        GamepadEx2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> schedule(new ResetCommand(robot.dump, robot.lift, robot.arm, robot.intake, robot.turret, this)));


        GamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(alliance == ALLIANCE.RED ? robot.ducc::red : robot.ducc::blue).whenReleased(robot.ducc::off);

        GamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(this::toggle);
        ;

        GamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(robot.lift::reset);
    }

    @Override
    public void run() {
        super.run();

        // drive lol

        drive.setWeightedDrivePower(
                new Pose2d(
                        scale(GamepadEx1.getLeftY(), 0.6) * (gamepad1.right_trigger > 0.5 ? 1 : 0.75),
                        -scale(GamepadEx1.getLeftX(), 0.6) * (gamepad1.right_trigger > 0.5 ? 1 : 0.75),
                        -scale(GamepadEx1.getRightX(), 0.6) * (gamepad1.right_trigger > 0.5 ? 1 : 0.75)
                )
        );

        if (robot.intake.hasFreight() && state == STATE.INTAKE) {
            System.out.println("has stuff");
            schedule(
                    mode == MODE.SHARED ?
                            new IntakeAndExtendSharedCommand(alliance, robot.lift, robot.arm, robot.dump, robot.turret, robot.intake)
                            :
                            new IntakeAndExtendCommand(robot.dump, robot.lift, robot.arm, robot.intake)
            );
            state = state.next();
        }

        if (mode == MODE.SHARED && state == STATE.REST) {
            if (gamepad2.left_trigger > 0.5) {
                robot.turret.left();
            } else if (gamepad2.right_trigger > 0.5) {
                robot.turret.right();
            }
        }

        drive.update();

        telemetry.addData("slide", robot.lift.getCurrentDrawA());
        telemetry.addData("intake", robot.intake.getCurrentDrawA());
        telemetry.addLine(mode.toString());
        telemetry.addLine(alliance.toString());
        telemetry.addLine(state.toString());
        telemetry.update();
    }

    public double scale(double x, double k) {
        return (1 - k) * x + k * x * x * x;
    }

    public void toggle() {
        mode = mode == MODE.SHARED ? MODE.SPECIFIC : MODE.SHARED;
    }

}
