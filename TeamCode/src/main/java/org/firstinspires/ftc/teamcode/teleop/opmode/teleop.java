package org.firstinspires.ftc.teamcode.teleop.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.ArmOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.IntakeAndExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.IntakeAndExtendSharedCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.OuttakeAndResetCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.OuttakeAndResetSharedCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.common.ff.ALLIANCE;
import org.firstinspires.ftc.teamcode.common.ff.MODE;
import org.firstinspires.ftc.teamcode.common.ff.STATE;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class teleop extends CommandOpMode {
    private Robot robot;
    private GamepadEx gamepadEx1, gamepadEx2;
    private STATE state = STATE.INTAKE;
    private MODE mode = MODE.SHARED;
    public ALLIANCE alliance;
    private MecanumDrive drive;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        MotorEx rf = new MotorEx(hardwareMap, "rf");
        MotorEx rb = new MotorEx(hardwareMap, "rb");
        MotorEx lf = new MotorEx(hardwareMap, "lf");
        MotorEx lb = new MotorEx(hardwareMap, "lb");

        rf.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rf.motorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.motorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.motorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.motorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rf.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);

        drive = new MecanumDrive(lf, rf, lb, rb);

        robot.lift.intake();
        robot.arm.intake();
        robot.dump.intake();
        robot.turret.intake();
        robot.intake.start();

        gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
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


        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                () -> schedule(
                        new IntakeStopCommand(robot.intake),
                        new ArmOuttakeCommand(robot.arm)
                )
        ).whenReleased(
                () -> schedule(
                        new SequentialCommandGroup(
                                new ArmIntakeCommand(robot.arm),
                                new WaitCommand(1000),
                                new IntakeStartCommand(robot.intake)
                        )
                )
        );

        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(robot.ducc::rightOn).whenReleased(robot.ducc::rightOff);
        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(robot.ducc::leftOn).whenReleased(robot.ducc::leftOff);

        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(this::shared);
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(this::specific);

        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(robot.lift::reset);
    }

    @Override
    public void run() {
        super.run();

        // drive lol
        drive.driveRobotCentric(
                scale(gamepadEx1.getLeftX(), 0.6) / ((gamepad1.right_trigger > 0.5) ? 1 : 1.5) / (mode == MODE.SHARED ? 1.5 : 1),
                scale(gamepadEx1.getLeftY(), 0.6) / ((gamepad1.right_trigger > 0.5) ? 1 : 1.5) / (mode == MODE.SHARED ? 1.5 : 1),
                scale(gamepadEx1.getRightX(), 0.6) / ((gamepad1.right_trigger > 0.5) ? 1 : 1.5)
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

    public void shared() {
        mode = MODE.SHARED;
    }

    public void specific() {
        mode = MODE.SPECIFIC;
    }
}
