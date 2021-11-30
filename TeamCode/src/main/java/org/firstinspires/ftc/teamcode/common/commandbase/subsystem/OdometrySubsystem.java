package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.technototes.library.hardware.sensor.encoder.MotorEncoder;
import com.technototes.library.subsystem.Subsystem;
import com.technototes.path.subsystem.DeadWheelConstants;
import com.technototes.path.subsystem.ThreeDeadWheelLocalizer;



public class OdometrySubsystem extends ThreeDeadWheelLocalizer implements Subsystem {
    @Config
    public abstract static class OdometryConstants implements DeadWheelConstants {

        @LateralDistance
        public static double LATERAL_DISTANCE = 0;

        @ForwardOffset
        public static double FORWARD_OFFSET = 0;

        @EncoderOverflow
        public static boolean ENCODER_OVERFLOW = true;

        @GearRatio
        public static double GEAR_RATIO = 1;

        @TicksPerRev
        public static double TICKS_PER_REV = 8192;

        @WheelRadius
        public static double WHEEL_RADIUS = 0;

    }

    public OdometrySubsystem(MotorEncoder l, MotorEncoder r, MotorEncoder f) {
        super(l, r, f, ()->OdometryConstants.class);
    }
}
