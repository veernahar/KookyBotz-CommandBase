package org.firstinspires.ftc.teamcode.teleop.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technototes.library.command.Command;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.command.SequentialCommandGroup;
import com.technototes.library.command.WaitCommand;
import com.technototes.library.structure.CommandOpMode;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.ff.ALLIANCE;
import org.firstinspires.ftc.teamcode.common.ff.MODE;
import org.firstinspires.ftc.teamcode.common.ff.STATE;

@Disabled
@TeleOp
public class TeleOpMode extends CommandOpMode {
    private Robot robot;
    private STATE state = STATE.INTAKE;
    private MODE mode = MODE.SPECIFIC;
    private ALLIANCE alliance = ALLIANCE.RED;


    @Override
    public void uponInit() {
        robot = new Robot();

        robot.lift.intake();
        robot.arm.intake();
        robot.outtake.intake();
        robot.turret.intake();
        robot.intake.start();

        codriverGamepad.a.whenPressed(mode == MODE.SPECIFIC ? outtakeAndResetSpecific : outtakeAndResetShared);


        codriverGamepad.dpadUp.whenPressed(() -> mode = MODE.SPECIFIC);
        codriverGamepad.dpadDown.whenPressed(() -> mode = MODE.SHARED);

        codriverGamepad.b.whenPressedReleased(alliance == ALLIANCE.RED ? robot.ducc::leftOn : robot.ducc::rightOn,
                alliance == ALLIANCE.RED ? robot.ducc::leftOff : robot.ducc::rightOff);

        CommandScheduler.getInstance().schedule((mode == MODE.SPECIFIC ? intakeAndExtendSpecific : intakeAndExtendShared)
                .andThen(()->state = state.next()),
                ()->robot.intake.hasFreight()&&state == STATE.INTAKE);
    }

    @Override
    public void runLoop() {
        // drive lol
        //TODO make this work with technolib this is modified, idk if itl work but it should
        robot.drive.setWeightedDrivePower(new Pose2d(
                driverGamepad.leftStick.getXAxis() / driverGamepad.rightTrigger.getAsDouble() > 0.5 ? 1 : 2,
                driverGamepad.leftStick.getYAxis() / driverGamepad.rightTrigger.getAsDouble() > 0.5 ? 1 : 2,
                driverGamepad.rightStick.getXAxis() / driverGamepad.rightTrigger.getAsDouble() > 0.5 ? 1 : 2
        ));

    }

    public final SequentialCommandGroup intakeAndExtendSpecific = new SequentialCommandGroup(
            //state is rest
            new WaitCommand(0.25),
            Command.create(robot.intake::close, robot.intake),
            new WaitCommand(0.5),
            Command.create(robot.intake::reverse, robot.intake),
            Command.create(robot.arm::outtake, robot.arm),
            new WaitCommand(0.5),
            Command.create(robot.lift::outtake, robot.lift)
    );

    public final SequentialCommandGroup intakeAndExtendShared = new SequentialCommandGroup(
            //hi
    );

    public final SequentialCommandGroup outtakeAndResetSpecific = new SequentialCommandGroup(
            () -> state = state.next(),
            // state is outtaking
            Command.create(robot.outtake::outtake, robot.outtake),
            new WaitCommand(1),
            Command.create(robot.outtake::intake, robot.outtake),
            Command.create(robot.lift::intake, robot.lift),
            new WaitCommand(0.75),
            Command.create(robot.arm::intake, robot.arm),
            new WaitCommand(0.5),
            Command.create(robot.intake::start, robot.intake),
            Command.create(() -> state = state.next())
            // state is intaking again
    );

    public final SequentialCommandGroup outtakeAndResetShared = alliance == ALLIANCE.RED ? new SequentialCommandGroup(
            // red
    ) : new SequentialCommandGroup(
            // blu
    );
}
