package org.firstinspires.ftc.teamcode.rr.opmode;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rr.RRMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RRMecanumDrive drive = new RRMecanumDrive(hardwareMap);
        Pose2d CYCLE_START = new Pose2d(12, -64, toRadians(0));
        Pose2d CYCLE_DEPOSIT = new Pose2d(6, -54, toRadians(-55));
        Pose2d[] GAP = new Pose2d[]{
                new Pose2d(12, -64, toRadians(0)),
                new Pose2d(12, -64, toRadians(0)),
                new Pose2d(12, -63, toRadians(0)),
                new Pose2d(12, -63, toRadians(0)),
                new Pose2d(12, -63, toRadians(0))
        };
        Pose2d[] CYCLE_COLLECT = new Pose2d[]{
                new Pose2d(42, -64, toRadians(0)),
                new Pose2d(44, -64, toRadians(0)),
                new Pose2d(46, -63, toRadians(0)),
                new Pose2d(48, -63, toRadians(0)),
                new Pose2d(40, -63, toRadians(0)),
        };

        drive.getLocalizer().setPoseEstimate(CYCLE_START);

        waitForStart();

        if (isStopRequested()) return;


        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(CYCLE_START)
                //preload
                .setReversed(false)
                .lineToSplineHeading(new Pose2d(CYCLE_DEPOSIT.vec(), CYCLE_DEPOSIT.getHeading()))
                .waitSeconds(0.25)
                .lineToSplineHeading(GAP[0])
                .lineTo(CYCLE_COLLECT[0].vec())

                //cycle
                .lineTo(GAP[1].vec())
                .lineToSplineHeading(new Pose2d(CYCLE_DEPOSIT.vec(), CYCLE_DEPOSIT.getHeading()))
                .waitSeconds(0.25)
                .lineToSplineHeading(GAP[1])
                .lineTo(CYCLE_COLLECT[1].vec())

                .lineTo(GAP[2].vec())
                .lineToSplineHeading(new Pose2d(CYCLE_DEPOSIT.vec(), CYCLE_DEPOSIT.getHeading()))
                .waitSeconds(0.25)
                .lineToSplineHeading(GAP[2])
                .lineTo(CYCLE_COLLECT[2].vec())

                .lineTo(GAP[3].vec())
                .lineToSplineHeading(new Pose2d(CYCLE_DEPOSIT.vec(), CYCLE_DEPOSIT.getHeading()))
                .waitSeconds(0.25)
                .lineToSplineHeading(GAP[3])
                .lineTo(CYCLE_COLLECT[3].vec())

                .lineTo(GAP[4].vec())
                .lineToSplineHeading(new Pose2d(CYCLE_DEPOSIT.vec(), CYCLE_DEPOSIT.getHeading()))
                .waitSeconds(0.25)
                .lineToSplineHeading(GAP[4])
                .lineTo(CYCLE_COLLECT[4].vec())


                .build()
        );
    }
}
