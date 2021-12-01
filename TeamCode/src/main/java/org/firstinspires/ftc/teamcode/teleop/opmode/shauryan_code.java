package org.firstinspires.ftc.teamcode.teleop.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommands.specific.CloseIntakeAndExtendSpecificCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommands.specific.OuttakeAndResetSpecificCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftExtendCommand;
import org.firstinspires.ftc.teamcode.common.ff.ALLIANCE;
import org.firstinspires.ftc.teamcode.common.ff.MODE;
import org.firstinspires.ftc.teamcode.common.ff.STATE;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@Disabled
@TeleOp
public class shauryan_code extends CommandOpMode {
    private Robot robot;


    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);

        schedule(new OuttakeArmCommand(robot.arm).andThen(new LiftExtendCommand(robot.lift)));
    }

    @Override
    public void run() {
        super.run();
    }

}
