package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.hardware.sensor.encoder.MotorEncoder;
import com.technototes.library.logger.Log;
import com.technototes.library.logger.Loggable;
import com.technototes.library.structure.CommandOpMode;
import com.technototes.library.util.Color;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.OdometrySubsystem;

@TeleOp
public class DriveTest extends CommandOpMode implements Loggable {
    private OdometrySubsystem robotOdometry;

    // anything you are logging must be public
    @Log(name="Drivebase pose", color = Color.RED)
    public DrivebaseSubsystem robotDrive;
    private EncodedMotor<DcMotorEx> fL, fR, bL, bR;
    private MotorEncoder leftEncoder, rightEncoder, centerEncoder;

    static final double TRACKWIDTH = 9.38101;
    static final double WHEEL_DIAMETER = 1.37795276;
    static double TICKS_TO_INCHES;
    static final double CENTER_WHEEL_OFFSET = -9;


    @Override
    public void uponInit() {

        telemetry  = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //TODO invert necessary motors
// can do on same line with chaining
        fL = new EncodedMotor<>("lf");//.invert();
        fR = new EncodedMotor<>("rf");
        bL = new EncodedMotor<>("lb");
        bR = new EncodedMotor<>("rb");


        leftEncoder = new MotorEncoder("rb");
        rightEncoder = new MotorEncoder("rf");
        centerEncoder = new MotorEncoder("lf");

        leftEncoder.invert();

        // create our drive object
        //TODO have imu
        robotDrive = new DrivebaseSubsystem(fL, fR, bL, bR, null, new OdometrySubsystem(leftEncoder, rightEncoder, centerEncoder));



    }

    @Override
    public void runLoop() {
        robotDrive.setWeightedDrivePower(new Pose2d(
                driverGamepad.leftStick.getXAxis() / driverGamepad.rightTrigger.getAsDouble() > 0.5 ? 1 : 2,
                driverGamepad.leftStick.getYAxis() / driverGamepad.rightTrigger.getAsDouble() > 0.5 ? 1 : 2,
                driverGamepad.rightStick.getXAxis() / driverGamepad.rightTrigger.getAsDouble() > 0.5 ? 1 : 2
        ));
        robotDrive.update();
    }
}
