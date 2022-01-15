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
import org.firstinspires.ftc.teamcode.common.commandbase.command.drivecommand.MoveForwardUntilIntakeCommand;
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
public class red_cycle_auto extends OpMode {
    private Robot robot;
    private AutonomousDrivetrain autonomousDrivetrain;
    private BarcodePipeline pipeline;
    private FtcDashboard dashboard;

    private TrajectorySequence preload, pickup, drop;

    Pose2d CYCLE_START = new Pose2d(12, -62, toRadians(-90));
    Pose2d CYCLE_DEPOSIT = new Pose2d(10, -56, toRadians(-55));
    Pose2d[] GAP = new Pose2d[]{
            new Pose2d(12, -63.75, toRadians(0)),
            new Pose2d(12, -63.75, toRadians(0)),
            new Pose2d(12, -63.75, toRadians(0)),
            new Pose2d(12, -63.75, toRadians(0)),
            new Pose2d(12, -63.75, toRadians(0))
    };
    Pose2d[] CYCLE_COLLECT = new Pose2d[]{
            new Pose2d(34, -63.75, toRadians(0)),
            new Pose2d(34, -63.75, toRadians(0)),
            new Pose2d(34, -63.75, toRadians(0)),
            new Pose2d(34, -63.75, toRadians(0)),
            new Pose2d(34, -63.75, toRadians(0)),
    };

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        autonomousDrivetrain = new AutonomousDrivetrain(hardwareMap, Math.toRadians(-90));
        autonomousDrivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dashboard = FtcDashboard.getInstance();

        robot.lift.intake();
        robot.arm.intake();
        robot.dump.intake();
        robot.turret.intake();

        robot.webcam.setPipeline(pipeline = new BarcodePipeline());

        robot.webcam.setMillisecondsPermissionTimeout(2500);
        robot.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robot.webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        dashboard.startCameraStream(robot.webcam, 30);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        autonomousDrivetrain.getLocalizer().setPoseEstimate(CYCLE_START);

        preload = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_START)
                .setReversed(false)
                .lineToLinearHeading(CYCLE_DEPOSIT)
                .waitSeconds(1.5)
                .build();

        pickup = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_DEPOSIT)
                .lineToSplineHeading(GAP[0])
                .lineTo(CYCLE_COLLECT[0].vec())
                .build();

        drop = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_DEPOSIT)
                .lineToSplineHeading(GAP[1])
                .lineTo(CYCLE_COLLECT[1].vec())

                .lineTo(GAP[2].vec())
                .lineToSplineHeading(CYCLE_DEPOSIT)
                .build();
    }

    @Override
    public void start() {
        BarcodePipeline.BarcodePosition position = pipeline.getAnalysis();
        telemetry.addLine(position.toString());
        telemetry.update();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowTrajectoryCommand(autonomousDrivetrain, preload).alongWith(preload(position)),

                        //cycle 1
                        new FollowTrajectoryCommand(autonomousDrivetrain, pickup),
                        new MoveForwardUntilIntakeCommand(autonomousDrivetrain, robot.intake, 0.5),
                        new IntakeAndExtendCommand(robot.dump, robot.lift, robot.arm, robot.intake)
                                .alongWith(new FollowTrajectoryCommand(autonomousDrivetrain, generateDrop())),
                        new OuttakeAndResetCommand(robot.dump, robot.lift, robot.arm, robot.intake),
                        new WaitCommand(500),

                        //cycle 2
                        new FollowTrajectoryCommand(autonomousDrivetrain, pickup),
                        new MoveForwardUntilIntakeCommand(autonomousDrivetrain, robot.intake, 0.5),
                        new IntakeAndExtendCommand(robot.dump, robot.lift, robot.arm, robot.intake)
                                .alongWith(new FollowTrajectoryCommand(autonomousDrivetrain, generateDrop())),
                        new OuttakeAndResetCommand(robot.dump, robot.lift, robot.arm, robot.intake),

                        //park
                        new FollowTrajectoryCommand(autonomousDrivetrain, pickup)
                )


        );

    }

    public Command preload(BarcodePipeline.BarcodePosition position) {
        switch (position) {
            case LEFT:
                return new SequentialCommandGroup(
                        new IntakeAndExtendLowCommand(robot.dump, robot.lift, robot.arm, robot.intake),
                        new WaitCommand(750),
                        new OuttakeAndResetMidLowCommand(robot.dump, robot.lift, robot.arm, robot.intake)
                );
            case CENTER:
                return new SequentialCommandGroup(
                        new IntakeAndExtendMidCommand(robot.dump, robot.lift, robot.arm, robot.intake),
                        new WaitCommand(750),
                        new OuttakeAndResetMidLowCommand(robot.dump, robot.lift, robot.arm, robot.intake)
                );
            default:
                return new SequentialCommandGroup(
                        new IntakeAndExtendCommand(robot.dump, robot.lift, robot.arm, robot.intake),
                        new WaitCommand(750),
                        new OuttakeAndResetCommand(robot.dump, robot.lift, robot.arm, robot.intake)
                );

        }
    }

    public TrajectorySequence generateDrop() {
        return autonomousDrivetrain.trajectorySequenceBuilder(autonomousDrivetrain.getPoseEstimate())
                .lineToSplineHeading(GAP[2])
                .lineToSplineHeading(CYCLE_DEPOSIT)
                .build();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        autonomousDrivetrain.update();
    }
}
