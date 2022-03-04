// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;


import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Drive_Subsystem;


public class TeleDriveCommand extends CommandBase {
  /** Creates a new TeleDrive. */
  
  private final PS4Controller driver_Controller;
  private final Drive_Subsystem drive_Subsystem;

  public TeleDriveCommand(PS4Controller driver_Controller, Drive_Subsystem drive_Subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driver_Controller = driver_Controller;
    this.drive_Subsystem = drive_Subsystem;
    addRequirements(drive_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    drive_Subsystem.arcadeDrive(getSpeed(), getRotation(),true);
  }

  private double getSpeed() {
    double speed =-driver_Controller.getRawAxis(1);
    return speed;
  }

  private double getRotation() {
    double rotation = driver_Controller.getRawAxis(2);
    return rotation;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    
  }

}
