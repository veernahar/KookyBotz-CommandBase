package org.firstinspires.ftc.teamcode.teleop.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.ff.ALLIANCE;

@TeleOp
public class blue extends teleop {
    @Override
    public void initialize() {
        alliance = ALLIANCE.BLUE;
        super.initialize();
    }
}
