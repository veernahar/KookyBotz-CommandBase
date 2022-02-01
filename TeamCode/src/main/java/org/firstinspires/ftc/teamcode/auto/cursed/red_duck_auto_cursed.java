package org.firstinspires.ftc.teamcode.auto.cursed;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.commandbase.command.drivecommand.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.duckcommand.DuckOffCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.duckcommand.DuckRedCommand;
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
public class red_duck_auto_cursed extends OpMode {
    private Robot robot;
    private AutonomousDrivetrain autonomousDrivetrain;
    private BarcodePipeline pipeline;

    private TrajectorySequence preload, duck, pick, drop, park;
    Pose2d CYCLE_START = new Pose2d(-34, -62, toRadians(-90));
    Pose2d CYCLE_DEPOSIT = new Pose2d(-30, -56, toRadians(-115));
    Pose2d DUCK = new Pose2d(-60, -52, toRadians(180));
    Pose2d PARK = new Pose2d(-60, -35, toRadians(180));

    Pose2d PASS_1_START = new Pose2d(-30, -54, toRadians(-130));

    Pose2d PASS_2_END = new Pose2d(-62, -57, toRadians(-130));
    Pose2d PASS_2_START = new Pose2d(-30, -57, toRadians(-130));

    Pose2d PASS_3_END = new Pose2d(-60, -61, toRadians(-122.5));
    Pose2d PASS_3_START = new Pose2d(-30, -61, toRadians(-122.5));

    Pose2d DUCK_DROP = new Pose2d(-42, -42, toRadians(-147.5));

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        autonomousDrivetrain = new AutonomousDrivetrain(hardwareMap, Math.toRadians(-90));
        autonomousDrivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FtcDashboard dashboard = FtcDashboard.getInstance();

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
                .setVelConstraint(new TranslationalVelocityConstraint(60))
                .setReversed(true)
                .lineToLinearHeading(CYCLE_DEPOSIT)
                .waitSeconds(4)
                .build();

        duck = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_DEPOSIT)
                .lineToLinearHeading(DUCK)
                .strafeLeft(3)
                .build();

        pick = autonomousDrivetrain.trajectorySequenceBuilder(DUCK)
                .strafeRight(6)
                .lineToLinearHeading(PASS_1_START)
                .lineToSplineHeading(PASS_2_START)
                .setVelConstraint(new TranslationalVelocityConstraint(15))
                .lineToLinearHeading(PASS_2_END)
                .setVelConstraint(new TranslationalVelocityConstraint(60))
                .lineToLinearHeading(PASS_2_START)
                .lineToSplineHeading(PASS_3_START)
                .setVelConstraint(new TranslationalVelocityConstraint(15))
                .lineToLinearHeading(PASS_3_END)
                .setVelConstraint(new TranslationalVelocityConstraint(60))
                .build();

        drop = autonomousDrivetrain.trajectorySequenceBuilder(DUCK)
                .lineToLinearHeading(DUCK_DROP)
                .build();


        park = autonomousDrivetrain.trajectorySequenceBuilder(DUCK)
                .lineToLinearHeading(PARK)
                .build();
    }

    @Override
    public void start() {
        BarcodePipeline.BarcodePosition position = pipeline.getAnalysis();
        telemetry.addLine(position.toString());
        telemetry.update();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        //preload
                        new FollowTrajectoryCommand(autonomousDrivetrain, preload).alongWith(preload(position)),

                        new InstantCommand(robot.intake::stop),

                        new FollowTrajectoryCommand(autonomousDrivetrain, duck),
                        new DuckRedCommand(robot.ducc),
                        new WaitCommand(3000),
                        new DuckOffCommand(robot.ducc),

                        //pick
                        new InstantCommand(robot.intake::start),
                        new FollowTrajectoryCommand(autonomousDrivetrain, pick),

                        //drop
                        new FollowTrajectoryCommand(autonomousDrivetrain, drop)
                                .alongWith(new IntakeAndExtendCommand(robot.dump, robot.lift, robot.arm, robot.intake)),

                        //park
                        new OuttakeAndResetCommand(robot.dump, robot.lift, robot.arm, robot.intake)
                                .alongWith(new WaitCommand(1000)
                                        .andThen(new FollowTrajectoryCommand(autonomousDrivetrain, park))))
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


    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        autonomousDrivetrain.update();
    }
}
