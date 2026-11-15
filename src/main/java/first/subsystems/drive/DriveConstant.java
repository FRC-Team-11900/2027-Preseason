package first.subsystems.drive;

import static org.wpilib.units.Units.MetersPerSecond;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;
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
}
