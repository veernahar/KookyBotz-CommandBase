package org.firstinspires.ftc.teamcode.auto.cycle;

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
public class red_cycle_auto_2 extends OpMode {
    private Robot robot;
    private AutonomousDrivetrain autonomousDrivetrain;
    private BarcodePipeline pipeline;
    private FtcDashboard dashboard;

    private TrajectorySequence preload, pickup1, drop1, pickup2, drop2, pickup3, drop3, park;

    public static Pose2d CYCLE_START = new Pose2d(12, -62, toRadians(-90));
    public static Pose2d CYCLE_DEPOSIT = new Pose2d(10, -56, toRadians(-52.5));
    public static Pose2d PRELOAD_DEPOSIT = new Pose2d(10, -56, toRadians(-54));

    public static Pose2d[] GAP = new Pose2d[]{
            new Pose2d(12, -64, toRadians(0)),
            new Pose2d(12, -64.25, toRadians(0)),
            new Pose2d(12, -64.5, toRadians(0)),
            new Pose2d(12, -64.75, toRadians(0)),
    };
    public static Pose2d[] CYCLE_COLLECT = new Pose2d[]{
            new Pose2d(39, -64, toRadians(0)),
            new Pose2d(41.5, -64, toRadians(0)),
            new Pose2d(44, -64, toRadians(0)),
            new Pose2d(38, -64, toRadians(0)),
    };

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
                .lineToSplineHeading(PRELOAD_DEPOSIT)
                .build();

        pickup1 = autonomousDrivetrain.trajectorySequenceBuilder(PRELOAD_DEPOSIT)
                .waitSeconds(0.5)
                .lineToSplineHeading(GAP[0])
                .lineTo(CYCLE_COLLECT[0].vec())
                .build();

        drop1 = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_COLLECT[0])
                .lineTo(GAP[0].vec())
                .lineToSplineHeading(CYCLE_DEPOSIT)
                .build();

        pickup2 = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_DEPOSIT)
                .waitSeconds(0.5)
                .lineToSplineHeading(GAP[1])
                .lineTo(CYCLE_COLLECT[1].vec())
                .build();

        drop2 = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_COLLECT[0])
                .lineTo(GAP[1].vec())
                .lineToSplineHeading(CYCLE_DEPOSIT)
                .build();

        pickup3 = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_DEPOSIT)
                .waitSeconds(0.5)
                .lineToSplineHeading(GAP[2])
                .lineTo(CYCLE_COLLECT[2].vec())
                .build();

        drop3 = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_COLLECT[0])
                .lineTo(GAP[2].vec())
                .lineToSplineHeading(CYCLE_DEPOSIT)
                .build();

        park = autonomousDrivetrain.trajectorySequenceBuilder(CYCLE_DEPOSIT)
                .waitSeconds(0.5)
                .lineToSplineHeading(GAP[3])
                .lineTo(CYCLE_COLLECT[3].vec())
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
                        new FollowTrajectoryCommand(autonomousDrivetrain, preload)
                                .alongWith(preloadExtend(position))
                                .andThen(new WaitCommand(500).andThen(preloadRetract(position))),

                        //3 cycles
                        new FollowTrajectoryCommand(autonomousDrivetrain, pickup1),
                        new FollowTrajectoryCommand(autonomousDrivetrain, drop1).alongWith(extend()),
                        new FollowTrajectoryCommand(autonomousDrivetrain, pickup2).alongWith(outtake()),
                        new FollowTrajectoryCommand(autonomousDrivetrain, drop2).alongWith(extend()),
                        new FollowTrajectoryCommand(autonomousDrivetrain, pickup3).alongWith(outtake()),
                        new FollowTrajectoryCommand(autonomousDrivetrain, drop3).alongWith(extend()),

                        //park
                        new FollowTrajectoryCommand(autonomousDrivetrain, park).alongWith(outtake())
                )
        );

    }

    public Command extend() {
        return new IntakeAndExtendCommand(robot.dump, robot.lift, robot.arm, robot.intake);
    }

    public Command outtake() {
        return new OuttakeAndResetCommand(robot.dump, robot.lift, robot.arm, robot.intake);
    }

    public Command preloadExtend(BarcodePipeline.BarcodePosition position) {
        switch (position) {
            case LEFT:
                return new SequentialCommandGroup(
                        new IntakeAndExtendLowCommand(robot.dump, robot.lift, robot.arm, robot.intake),
                        new WaitCommand(250)
                );
            case CENTER:
                return new SequentialCommandGroup(
                        new IntakeAndExtendMidCommand(robot.dump, robot.lift, robot.arm, robot.intake),
                        new WaitCommand(250)
                );
            default:
                return new SequentialCommandGroup(
                        new IntakeAndExtendCommand(robot.dump, robot.lift, robot.arm, robot.intake),
                        new WaitCommand(250)
                );

        }
    }

    public Command preloadRetract(BarcodePipeline.BarcodePosition position) {
        switch (position) {
            case LEFT:
            case CENTER:
                return new SequentialCommandGroup(
                        new OuttakeAndResetMidLowCommand(robot.dump, robot.lift, robot.arm, robot.intake)
                );
            default:
                return new SequentialCommandGroup(
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

    @Override
    public void stop(){
        CommandScheduler.getInstance().reset();
        robot.webcam.stopStreaming();
        robot.webcam.closeCameraDevice();
    }
}