//package org.firstinspires.ftc.teamcode.teleop.opmode;
//
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.PurePursuitCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.common.hardware.Robot;
//import org.firstinspires.ftc.teamcode.common.ff.ALLIANCE;
//import org.firstinspires.ftc.teamcode.common.ff.MODE;
//import org.firstinspires.ftc.teamcode.common.ff.STATE;
//
//@Disabled
//@TeleOp
//public class teleop extends CommandOpMode {
//    private Robot robot;
//    private GamepadEx gamepadEx1, gamepadEx2;
//    private STATE state = STATE.INTAKE;
//    private MODE mode = MODE.SPECIFIC;
//    private ALLIANCE alliance = ALLIANCE.RED;
//
//
//    @Override
//    public void initialize() {
//        robot = new Robot(hardwareMap);
//        gamepadEx1 = new GamepadEx(gamepad1);
//        gamepadEx2 = new GamepadEx(gamepad2);
//
//        robot.lift.intake();
//        robot.arm.intake();
//        robot.outtake.intake();
//        robot.turret.intake();
//        robot.intake.start();
//
//        if (mode == MODE.SPECIFIC) {
//            gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                    new InstantCommand(() -> schedule(state == STATE.REST ? outtakeAndResetSpecific : new InstantCommand()))
//            );
//        } else {
//            gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                    new InstantCommand(() -> schedule(state == STATE.REST ? outtakeAndResetShared : new InstantCommand()))
//            );
//        }
//
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> mode = MODE.SPECIFIC);
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> mode = MODE.SHARED);
//
//        if (alliance == ALLIANCE.RED) {
//            gamepadEx2.getGamepadButton(GamepadKeys.Button.B).whenPressed(robot.ducc::leftOn);
//            gamepadEx2.getGamepadButton(GamepadKeys.Button.B).whenReleased(robot.ducc::leftOff);
//        } else {
//            gamepadEx2.getGamepadButton(GamepadKeys.Button.B).whenPressed(robot.ducc::rightOn);
//            gamepadEx2.getGamepadButton(GamepadKeys.Button.B).whenReleased(robot.ducc::rightOff);
//        }
//    }
//
//    @Override
//    public void run() {
//        super.run();
//
//        // drive lol
//        robot.drive.driveRobotCentric(
//                gamepadEx1.getLeftX() / gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5 ? 1 : 2,
//                gamepadEx1.getLeftY() / gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5 ? 1 : 2,
//                gamepadEx1.getRightX() / gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5 ? 1 : 2
//        );
//
//        if (robot.intake.hasFreight() && state == STATE.INTAKE) {
//            if (mode == MODE.SPECIFIC) {
//                schedule(intakeAndExtendSpecific);
//            } else {
//                schedule(intakeAndExtendShared);
//            }
//
//            state = state.next();
//        }
//    }
//
//    public final SequentialCommandGroup intakeAndExtendSpecific = new SequentialCommandGroup(
//            //state is rest
//            new WaitCommand(250),
//            new InstantCommand(robot.intake::close, robot.intake),
//            new WaitCommand(500),
//            new InstantCommand(robot.intake::reverse, robot.intake),
//            new InstantCommand(robot.arm::outtake, robot.arm),
//            new WaitCommand(500),
//            new InstantCommand(robot.lift::outtake, robot.lift)
//    );
//
//    public final SequentialCommandGroup intakeAndExtendShared = new SequentialCommandGroup(
//            //hi
//    );
//
//    public final SequentialCommandGroup outtakeAndResetSpecific = new SequentialCommandGroup(
//            new InstantCommand(() -> state = state.next()),
//            // state is outtaking
//            new InstantCommand(robot.outtake::outtake, robot.outtake),
//            new WaitCommand(1000),
//            new InstantCommand(robot.outtake::intake, robot.outtake),
//            new InstantCommand(robot.lift::intake, robot.lift),
//            new WaitCommand(750),
//            new InstantCommand(robot.arm::intake, robot.arm),
//            new WaitCommand(500),
//            new InstantCommand(robot.intake::start, robot.intake),
//            new InstantCommand(() -> state = state.next())
//            // state is intaking again
//    );
//
//    public final SequentialCommandGroup outtakeAndResetShared = alliance == ALLIANCE.RED ? new SequentialCommandGroup(
//            // red
//    ) : new SequentialCommandGroup(
//            // blu
//    );
//}
