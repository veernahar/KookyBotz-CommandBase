package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class DriveTest extends OpMode {
    private HolonomicOdometry m_robotOdometry;
    private MecanumDrive m_robotDrive;
    private Motor fL, fR, bL, bR;
    private MotorEx leftEncoder, rightEncoder, centerEncoder;

    static final double TRACKWIDTH = 9.38101;
    static final double WHEEL_DIAMETER = 1.37795276;
    static double TICKS_TO_INCHES;
    static final double CENTER_WHEEL_OFFSET = -9;


    @Override
    public void init() {

        telemetry  = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        fL = new Motor(hardwareMap, "lf");
        fR = new Motor(hardwareMap, "rf");
        bL = new Motor(hardwareMap, "lb");
        bR = new Motor(hardwareMap, "rb");

        fL.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        fR.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        fL.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // create our drive object
        m_robotDrive = new MecanumDrive(fL, fR, bL, bR);

        leftEncoder = new MotorEx(hardwareMap, "rb");
        rightEncoder = new MotorEx(hardwareMap, "rf");
        centerEncoder = new MotorEx(hardwareMap, "lf");

        leftEncoder.encoder.setDirection(Motor.Direction.REVERSE);

        // calculate multiplier
        TICKS_TO_INCHES = 1 / 1743.02601133;

        // create our odometry object and subsystem
        m_robotOdometry = new HolonomicOdometry(
                () -> leftEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                () -> rightEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                () -> centerEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );


    }

    @Override
    public void loop() {
        m_robotDrive.driveRobotCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        m_robotOdometry.updatePose();

        telemetry.addLine(m_robotOdometry.getPose().toString());
    }
}
