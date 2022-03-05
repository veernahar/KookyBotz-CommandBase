package org.firstinspires.ftc.teamcode.teleop.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.commandbase.command.duckcommand.DuckFastBlueCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.duckcommand.DuckFastRedCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.IntakeAndExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.IntakeAndExtendSharedCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.IntakeAndExtendSharedOppositeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.OuttakeAndResetCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.OuttakeAndResetSharedCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.ResetCommand;
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
    private MODE mode = MODE.SHARED;
    public ALLIANCE alliance;

    private ElapsedTime timer;
    private boolean flag;

    private boolean working = true;

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
        robot.initLiftUp();

        GamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(alliance == ALLIANCE.RED ? robot.ducc::red : robot.ducc::blue).whenReleased(robot.ducc::off);

        GamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(this::toggle);

        GamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(robot.intake::toggle);

        GamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(robot::up).whenReleased(robot::down);


        GamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                () -> {
                    if (state == STATE.REST) {
                        schedule(
                                mode == MODE.SHARED || mode == MODE.SHARED_OPPOSITE ?
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

        GamepadEx2.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            if(state == STATE.INTAKE){
                schedule(extend());
                state = state.next();
                schedule(new WaitCommand(500).andThen(new InstantCommand(() -> GamepadEx1.gamepad.rumble(500))));
            }
        });

        GamepadEx2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ResetCommand(robot.dump, robot.lift, robot.arm, robot.intake, robot.turret, this));

        GamepadEx2.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> working = !working);

        GamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(alliance == ALLIANCE.RED ? new DuckFastRedCommand(robot.ducc):new DuckFastBlueCommand(robot.ducc));
    }

    @Override
    public void run() {
        super.run();

        if (timer == null) {
            timer = new ElapsedTime();
        }

        if (timer.seconds() > (120 - 30) && !flag) {
            GamepadEx1.gamepad.rumble(1000);
            GamepadEx2.gamepad.rumble(1000);
            flag = true;
        }

        if(timer.seconds() > 120){
            GamepadEx1.gamepad.rumble(100);
            GamepadEx2.gamepad.rumble(100);
        }

        // drive lol

        if (Math.abs(GamepadEx2.getLeftX()) > 0.001 || Math.abs(GamepadEx2.getLeftY()) > 0.001 || Math.abs(GamepadEx2.getRightX()) > 0.001) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            dead(scale(GamepadEx2.getLeftY(), 0.6), 0),
                            dead(-scale(GamepadEx2.getLeftX(), 0.6), 0),
                            -scale(GamepadEx2.getRightX(), 0.6)*0.85
                    )
            );
        } else {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            scale(GamepadEx1.getLeftY(), 0.6) * (gamepad1.right_trigger > 0.5 ? 1 : 0.66) * (gamepad1.right_bumper ? 0.5 : 1),
                            -scale(GamepadEx1.getLeftX(), 0.6) * (gamepad1.right_trigger > 0.5 ? 1 : 0.66) * (gamepad1.right_bumper ? 0.5 : 1),
                            -scale(GamepadEx1.getRightX(), 0.6) * (gamepad1.right_trigger > 0.5 ? 1 : 0.66)
                    )
            );
        }


        if (robot.intake.hasFreight() && state == STATE.INTAKE && working) {
            System.out.println("has stuff");
            schedule(extend());
            state = state.next();
            //schedule(new WaitCommand(500).andThen(new InstantCommand(() -> GamepadEx1.gamepad.rumble(500))));
        }

        if ((mode == MODE.SHARED || mode == MODE.SHARED_OPPOSITE) && state == STATE.REST) {
            if (gamepad2.left_trigger > 0.5) {
                robot.turret.left();
            } else if (gamepad2.right_trigger > 0.5) {
                robot.turret.right();
            }
        }

        if (mode == MODE.SPECIFIC && state == STATE.REST) {
            if (gamepad2.left_trigger > 0.5) {
                robot.lift.intake();
            } else if (gamepad2.right_trigger > 0.5) {
                robot.lift.outtake();
            }
        }

        drive.update();

        telemetry.addData("slide", robot.lift.getCurrentDrawA());
        telemetry.addData("intake", robot.intake.getCurrentDrawA());
        telemetry.addLine(mode.toString());
        telemetry.addLine(alliance.toString());
        telemetry.addLine(state.toString());
        telemetry.addLine(String.valueOf(robot.distanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addLine(drive.getPoseEstimate().toString());
        telemetry.update();
    }

    public double scale(double x, double k) {
        return (1 - k) * x + k * x * x * x;
    }

    public double dead(double x, double k) {
        return Math.abs(x) > k ? x : 0;
    }

    public void toggle() {
        switch (mode) {
            case SPECIFIC:
                mode = MODE.SHARED;
                break;
            case SHARED:
                mode = MODE.SHARED_OPPOSITE;
                break;
            default:
                mode = MODE.SPECIFIC;
                break;
        }
    }

    public Command extend() {
        switch (mode) {
            case SPECIFIC:
                return new IntakeAndExtendCommand(robot.dump, robot.lift, robot.arm, robot.intake);
            case SHARED:
                return new IntakeAndExtendSharedCommand(alliance, robot.lift, robot.arm, robot.dump, robot.turret, robot.intake);
            default:
                return new IntakeAndExtendSharedOppositeCommand(alliance, robot.lift, robot.arm, robot.dump, robot.turret, robot.intake);
        }
    }

}