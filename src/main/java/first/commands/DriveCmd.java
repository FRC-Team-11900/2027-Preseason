package first.commands;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;


import org.wpilib.command2.Command;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.kinematics.ChassisVelocities;

import first.subsystems.drive.Drive;
import first.subsystems.drive.DriveConstant;

public class DriveCmd extends Command {

    private final Drive drive;

    private final Gamepad gamepad;

    private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

    public DriveCmd(Drive drive, Gamepad gamepad) {
        this.drive = drive;
        this.gamepad = gamepad;

        this.xLimiter = new SlewRateLimiter(DriveConstant.MAX_VELOCITY_RATE.in(MetersPerSecondPerSecond));
        this.yLimiter = new SlewRateLimiter(DriveConstant.MAX_VELOCITY_RATE.in(MetersPerSecondPerSecond));
        this.rotLimiter = new SlewRateLimiter(DriveConstant.MAX_ANGULAR_VELOCITY_RATE.in(RadiansPerSecondPerSecond));

        this.addRequirements(drive);
    }

    @Override
    public void execute() {

        var xSpeed = -gamepad.getLeftY() * DriveConstant.MAX_VELOCITY.in(MetersPerSecond);
        var ySpeed = -gamepad.getLeftX() * DriveConstant.MAX_VELOCITY.in(MetersPerSecond);
        var rot = -gamepad.getRightX() * DriveConstant.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);

        var velocities = new ChassisVelocities(
            xLimiter.calculate(xSpeed),
            yLimiter.calculate(ySpeed),
            rotLimiter.calculate(rot)
        );

        drive.setFieldOrientedVelocities(velocities);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
