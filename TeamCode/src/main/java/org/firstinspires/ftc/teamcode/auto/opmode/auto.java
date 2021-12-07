package org.firstinspires.ftc.teamcode.auto.opmode;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.command.drivecommand.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.IntakeAndExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommand.OuttakeAndResetCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.rr.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp
public class auto extends CommandOpMode {
    private Robot robot;
    private RRMecanumDrive rrMecanumDrive;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        rrMecanumDrive = new RRMecanumDrive(hardwareMap);

        robot.lift.intake();
        robot.arm.intake();
        robot.dump.intake();
        robot.turret.intake();

        Pose2d CYCLE_START = new Pose2d(12, -65, toRadians(0));
        Pose2d CYCLE_DEPOSIT = new Pose2d(6, -54, toRadians(-57.5));
        Pose2d[] GAP = new Pose2d[]{
                new Pose2d(12, -64, toRadians(0)),
                new Pose2d(12, -63, toRadians(0)),
                new Pose2d(12, -62, toRadians(0)),
                new Pose2d(12, -61, toRadians(0)),
                new Pose2d(12, -60, toRadians(0))
        };
        Pose2d[] CYCLE_COLLECT = new Pose2d[]{
                new Pose2d(40, -64, toRadians(0)),
                new Pose2d(42, -63, toRadians(0)),
                new Pose2d(44, -62, toRadians(0)),
                new Pose2d(46, -61, toRadians(0)),
                new Pose2d(40, -60, toRadians(0)),
        };

        rrMecanumDrive.getLocalizer().setPoseEstimate(CYCLE_START);

        TrajectorySequence preload = rrMecanumDrive.trajectorySequenceBuilder(CYCLE_START)
                .setReversed(false)
                .lineToSplineHeading(CYCLE_DEPOSIT)
                .waitSeconds(0.75)
                .lineToSplineHeading(GAP[0])
                .lineTo(CYCLE_COLLECT[0].vec())
                .build();

        TrajectorySequence cycle1 = rrMecanumDrive.trajectorySequenceBuilder(CYCLE_COLLECT[0])
                .lineTo(GAP[1].vec())
                .lineToSplineHeading(CYCLE_DEPOSIT)
                .waitSeconds(0.75)
                .lineToSplineHeading(GAP[1])
                .lineTo(CYCLE_COLLECT[1].vec())
                .build();

        TrajectorySequence cycle2 = rrMecanumDrive.trajectorySequenceBuilder(CYCLE_COLLECT[1])
                .lineTo(GAP[2].vec())
                .lineToSplineHeading(CYCLE_DEPOSIT)
                .waitSeconds(0.75)
                .lineToSplineHeading(GAP[2])
                .lineTo(CYCLE_COLLECT[2].vec())
                .build();

        TrajectorySequence cycle3 = rrMecanumDrive.trajectorySequenceBuilder(CYCLE_COLLECT[2])
                .lineTo(GAP[3].vec())
                .lineToSplineHeading(CYCLE_DEPOSIT)
                .waitSeconds(0.75)
                .lineToSplineHeading(GAP[3])
                .lineTo(CYCLE_COLLECT[3].vec())
                .build();

        TrajectorySequence cycle4 = rrMecanumDrive.trajectorySequenceBuilder(CYCLE_COLLECT[3])
                .lineTo(GAP[4].vec())
                .lineToSplineHeading(CYCLE_DEPOSIT)
                .waitSeconds(0.75)
                .lineToSplineHeading(GAP[4])
                .lineTo(CYCLE_COLLECT[4].vec())
                .build();

        schedule(
                new SequentialCommandGroup(
                        new InstantCommand(), //  run on init I think idfk
                        new ParallelCommandGroup( // preload
                                new FollowTrajectoryCommand(rrMecanumDrive, preload),
                                new IntakeAndExtendCommand(robot.dump, robot.lift, robot.arm, robot.intake)
                                        .andThen(new WaitCommand(750))
                                        .andThen(new OuttakeAndResetCommand(robot.dump, robot.lift, robot.arm, robot.intake))
                        ),
                        new ParallelCommandGroup( // cycles
                                new FollowTrajectoryCommand(rrMecanumDrive, cycle1),
                                new IntakeAndExtendCommand(robot.dump, robot.lift, robot.arm, robot.intake)
                                        .andThen(new WaitCommand(1700))
                                        .andThen(new OuttakeAndResetCommand(robot.dump, robot.lift, robot.arm, robot.intake))
                        ),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(rrMecanumDrive, cycle2),
                                new IntakeAndExtendCommand(robot.dump, robot.lift, robot.arm, robot.intake)
                                        .andThen(new WaitCommand(1950))
                                        .andThen(new OuttakeAndResetCommand(robot.dump, robot.lift, robot.arm, robot.intake))
                        ),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(rrMecanumDrive, cycle3),
                                new IntakeAndExtendCommand(robot.dump, robot.lift, robot.arm, robot.intake)
                                        .andThen(new WaitCommand(2200))
                                        .andThen(new OuttakeAndResetCommand(robot.dump, robot.lift, robot.arm, robot.intake))
                        ),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(rrMecanumDrive, cycle4),
                                new IntakeAndExtendCommand(robot.dump, robot.lift, robot.arm, robot.intake)
                                        .andThen(new WaitCommand(2450))
                                        .andThen(new OuttakeAndResetCommand(robot.dump, robot.lift, robot.arm, robot.intake))
                        ),
                        new InstantCommand(robot.intake::stop)
                )
        );
    }

    @Override
    public void run() {
        super.run();

        if (robot.intake.hasFreight()) {
            robot.intake.stop();
        }
    }
}
