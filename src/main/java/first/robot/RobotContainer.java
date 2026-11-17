// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command2.Command;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;

import first.commands.DriveCmd;
import first.subsystems.drive.Drive;
import first.subsystems.drive.GyroIO;
import first.subsystems.drive.GyroIOPigeon;
import first.subsystems.drive.ModuleConfigs;
import first.subsystems.drive.ModuleIO;
import first.subsystems.drive.ModuleIOTalon;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {

  private final ModuleIO[] modules = new ModuleIO[4];
  private final GyroIO gyro = new GyroIOPigeon();

  public final Drive drive;

  public final Gamepad gamepad;

  private final Selectable<Command> autoSelector;


  public RobotContainer() {

    for (int i = 0; i < 4; i++) {
      this.modules[i] = new ModuleIOTalon(ModuleConfigs.drivetrainIDs[i], ModuleConfigs.constants);
    }

    this.drive = new Drive(this.modules, this.gyro);

    this.gamepad = new Gamepad(0);

    this.autoSelector = AutoBuilder.buildAutoChooser();

    Tunables.publish("Autonomous Selector", this.autoSelector);

    configureDefaultCommands();
    configureBindings();
  }

  private void configureDefaultCommands() {
    this.drive.setDefaultCommand(new DriveCmd(this.drive, this.gamepad));
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return autoSelector.getSelected();
  }
}
