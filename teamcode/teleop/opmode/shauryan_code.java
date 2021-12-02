package org.firstinspires.ftc.teamcode.teleop.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommands.specific.PreloadCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@TeleOp
public class shauryan_code extends CommandOpMode {
    private Robot robot;

    private PreloadCommand preload;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        preload = new PreloadCommand(robot);
        schedule(new IntakeStartCommand(robot.intake));
    }

    @Override
    public void run() {
        super.run();
        if (robot.intake.hasFreight() && preload != null) {
            schedule(preload);
            preload = null;
        }
    }

}
