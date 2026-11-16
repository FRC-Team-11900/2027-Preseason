package first.subsystems.drive;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.LinearVelocity;

public class DriveConstant {
    private static final double kOffset = Units.inchesToMeters(21.75 / 2.0);

    public static final Translation2d[] kModuleLocations = {
            new Translation2d(kOffset, kOffset),
            new Translation2d(kOffset, -kOffset),
            new Translation2d(-kOffset, kOffset),
            new Translation2d(-kOffset, -kOffset)
    };

    public static final LinearVelocity MAX_MODULE_SPEED = MetersPerSecond.of(4);

    public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(4);
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(1);

    public static final LinearAcceleration MAX_VELOCITY_RATE = MetersPerSecondPerSecond.of(3);
    public static final AngularAcceleration MAX_ANGULAR_VELOCITY_RATE = RotationsPerSecondPerSecond.of(1);

}
