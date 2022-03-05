package org.firstinspires.ftc.teamcode.auto.mti;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.commandbase.command.drivecommand.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.auto.IntakeAndExtendHighCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.auto.OuttakeAndResetAutoCommand;
import org.firstinspires.ftc.teamcode.common.ff.vision.BarcodePipeline;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.rr.AutonomousDrivetrain;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class red_cycle_auto_3 extends OpMode {
    private Robot robot;
    private AutonomousDrivetrain autonomousDrivetrain;
    private BarcodePipeline pipeline;
    private FtcDashboard dashboard;

    private TrajectorySequence preload, pickup1, drop1, pickup2, drop2, pickup3, drop3, pickup4, drop4, pickup5, drop5, park;

    public static Pose2d CYCLE_START = new Pose2d(12, -62, toRadians(-90));
    public static Pose2d[] CYCLE_DEPOSIT = new Pose2d[]{
            new Pose2d(6, -52, toRadians(-60)),
            new Pose2d(5, -52, toRadians(-60)),
            new Pose2d(4, -52, toRadians(-60)),
            new Pose2d(3, -52, toRadians(-60)),
            new Pose2d(2, -52, toRadians(-60)),
            new Pose2d(1, -52, toRadians(-60)),
    };

    public static Pose2d[] GAP = new Pose2d[]{
            new Pose2d(20, -65, toRadians(0)),
            new Pose2d(20, -65, toRadians(0)),
            new Pose2d(20, -65, toRadians(0)),
            new Pose2d(20, -65, toRadians(0)),
            new Pose2d(20, -65, toRadians(0)),
            new Pose2d(20, -65, toRadians(0))
    };
    public static Pose2d[] CYCLE_COLLECT = new Pose2d[]{
            new Pose2d(36, -65, toRadians(0)),
            new Pose2d(36.5, -65, toRadians(0)),
            new Pose2d(37, -65, toRadians(0)),
            new Pose2d(37.5, -65, toRadians(0)),
            new Pose2d(38, -65, toRadians(0)),
            new Pose2d(38.5, -65, toRadians(0))
    };

    Pose2d PARK = new Pose2d(36, -66, toRadians(0));

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        autonomousDrivetrain = new AutonomousDrivetrain(hardwareMap, CYCLE_START.getHeading());
        autonomousDrivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        autonomousDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dashboard = FtcDashboard.getInstance();

        robot.lift.intake();
        robot.arm.intake();
        robot.dump.intake();
        robot.turret.intake();
        robot.initLiftUp();

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
                .lineToSplineHeading(CYCLE_DEPOSIT[0])
                .build();

        pickup1 = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_DEPOSIT[0])
                .splineTo(GAP[0].vec(), toRadians(-10))
                .splineTo(CYCLE_COLLECT[0].vec(), toRadians(0))
                .build();
        pickup2 = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_DEPOSIT[0])
                .splineTo(GAP[1].vec(), toRadians(-10))
                .splineTo(CYCLE_COLLECT[1].vec(), toRadians(0))
                .build();
        pickup3 = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_DEPOSIT[1])
                .splineTo(GAP[2].vec(), toRadians(-10))
                .splineTo(CYCLE_COLLECT[2].vec(), toRadians(0))
                .build();
        pickup4 = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_DEPOSIT[2])
                .splineTo(GAP[3].vec(), toRadians(-10))
                .splineTo(CYCLE_COLLECT[3].vec(), toRadians(0))
                .build();
        pickup5 = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_DEPOSIT[3])
                .splineTo(GAP[4].vec(), toRadians(-10))
                .splineTo(CYCLE_COLLECT[4].vec(), toRadians(0))
                .build();

        drop1 = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_COLLECT[0])
                .setReversed(true)
                .splineTo(GAP[0].vec(), toRadians(-190))
                .splineTo(CYCLE_DEPOSIT[0].vec(), toRadians(120))
                .build();
        drop2 = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_COLLECT[1])
                .setReversed(true)
                .splineTo(GAP[1].vec(), toRadians(-190))
                .splineTo(CYCLE_DEPOSIT[1].vec(), toRadians(120))
                .build();
        drop3 = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_COLLECT[2])
                .setReversed(true)
                .splineTo(GAP[2].vec(), toRadians(-190))
                .splineTo(CYCLE_DEPOSIT[2].vec(), toRadians(120))
                .build();
        drop4 = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_COLLECT[3])
                .setReversed(true)
                .splineTo(GAP[3].vec(), toRadians(-190))
                .splineTo(CYCLE_DEPOSIT[3].vec(), toRadians(120))
                .build();
        drop5 = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_COLLECT[4])
                .setReversed(true)
                .splineTo(GAP[4].vec(), toRadians(-190))
                .splineTo(CYCLE_DEPOSIT[4].vec(), toRadians(120))
                .build();

        park = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_DEPOSIT[4])
                .splineTo(GAP[5].vec(), toRadians(-10))
                .splineTo(PARK.vec(), toRadians(0))
                .build();
    }

    @Override
    public void start() {
        BarcodePipeline.BarcodePosition position = pipeline.getAnalysis();
        telemetry.addLine(position.toString());
        telemetry.update();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowTrajectoryCommand(autonomousDrivetrain, preload),

                        //pickup, delay start intake
                        new FollowTrajectoryCommand(autonomousDrivetrain, pickup1)
                                .alongWith(new WaitCommand(1250).andThen(new InstantCommand(robot.intake::start))),

                        //extend and delay deposit
                        new FollowTrajectoryCommand(autonomousDrivetrain, drop1).alongWith(
                                new IntakeAndExtendHighCommand(robot.dump, robot.lift, robot.arm, robot.intake).andThen(
                                        new WaitCommand(1000).andThen(
                                                new OuttakeAndResetAutoCommand(robot.dump, robot.lift, robot.arm, robot.intake)
                                        )
                                )
                        ),

                        //pickup, delay start intake
                        new FollowTrajectoryCommand(autonomousDrivetrain, pickup2)
                                .alongWith(new WaitCommand(1250).andThen(new InstantCommand(robot.intake::start))),

                        //extend and delay deposit
                        new FollowTrajectoryCommand(autonomousDrivetrain, drop2).alongWith(
                                new IntakeAndExtendHighCommand(robot.dump, robot.lift, robot.arm, robot.intake).andThen(
                                        new WaitCommand(1000).andThen(
                                                new OuttakeAndResetAutoCommand(robot.dump, robot.lift, robot.arm, robot.intake)
                                        )
                                )
                        ),

                        //pickup, delay start intake
                        new FollowTrajectoryCommand(autonomousDrivetrain, pickup3)
                                .alongWith(new WaitCommand(1250).andThen(new InstantCommand(robot.intake::start))),

                        //extend and delay deposit
                        new FollowTrajectoryCommand(autonomousDrivetrain, drop3).alongWith(
                                new IntakeAndExtendHighCommand(robot.dump, robot.lift, robot.arm, robot.intake).andThen(
                                        new WaitCommand(1000).andThen(
                                                new OuttakeAndResetAutoCommand(robot.dump, robot.lift, robot.arm, robot.intake)
                                        )
                                )
                        ),

                        //pickup, delay start intake
                        new FollowTrajectoryCommand(autonomousDrivetrain, pickup4)
                                .alongWith(new WaitCommand(1250).andThen(new InstantCommand(robot.intake::start))),

                        //extend and delay deposit
                        new FollowTrajectoryCommand(autonomousDrivetrain, drop4).alongWith(
                                new IntakeAndExtendHighCommand(robot.dump, robot.lift, robot.arm, robot.intake).andThen(
                                        new WaitCommand(1000).andThen(
                                                new OuttakeAndResetAutoCommand(robot.dump, robot.lift, robot.arm, robot.intake)
                                        )
                                )
                        ),

                        //pickup, delay start intake
                        new FollowTrajectoryCommand(autonomousDrivetrain, pickup5)
                                .alongWith(new WaitCommand(1250).andThen(new InstantCommand(robot.intake::start))),

                        //extend and delay deposit
                        new FollowTrajectoryCommand(autonomousDrivetrain, drop5).alongWith(
                                new IntakeAndExtendHighCommand(robot.dump, robot.lift, robot.arm, robot.intake).andThen(
                                        new WaitCommand(1000).andThen(
                                                new OuttakeAndResetAutoCommand(robot.dump, robot.lift, robot.arm, robot.intake)
                                        )
                                )
                        ),

                        new FollowTrajectoryCommand(autonomousDrivetrain, park)
                )
        );

    }


    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        autonomousDrivetrain.update();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
        robot.webcam.closeCameraDevice();
    }
}