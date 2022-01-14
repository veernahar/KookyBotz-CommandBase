package org.firstinspires.ftc.teamcode.auto.opmode;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.commandbase.command.drivecommand.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.IntakeAndExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.IntakeAndExtendLowCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.IntakeAndExtendMidCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.OuttakeAndResetCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.OuttakeAndResetMidLowCommand;
import org.firstinspires.ftc.teamcode.common.ff.vision.BarcodePipeline;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.rr.AutonomousDrivetrain;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class trajcommand_test extends OpMode {
    private AutonomousDrivetrain autonomousDrivetrain;

    private TrajectorySequence traj;


    Pose2d CYCLE_START = new Pose2d(-34, -62, toRadians(-90));
    Pose2d CYCLE_DEPOSIT = new Pose2d(-30, -56, toRadians(-115));
    Pose2d DUCK = new Pose2d(-60, -54, toRadians(180));
    Pose2d PARK = new Pose2d(-60, -35, toRadians(180));

    @Override
    public void init() {
        autonomousDrivetrain = new AutonomousDrivetrain(hardwareMap, toRadians(-90));
        autonomousDrivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        autonomousDrivetrain.getLocalizer().setPoseEstimate(CYCLE_START);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        traj = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_START)
                .setReversed(true)
                .lineToSplineHeading(CYCLE_DEPOSIT)
//                .waitSeconds(4)
//                .lineToLinearHeading(DUCK)
//                .strafeLeft(3)
                .build();

    }

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(new FollowTrajectoryCommand(autonomousDrivetrain, traj));

    }

    @Override
    public void loop() {
        autonomousDrivetrain.update();
        CommandScheduler.getInstance().run();
    }
}
