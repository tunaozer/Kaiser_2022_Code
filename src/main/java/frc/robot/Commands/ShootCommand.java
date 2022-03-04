// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Shooter_Subsystem;

public class ShootCommand extends CommandBase {

  private final Shooter_Subsystem shooter_Subsystem;

  public ShootCommand(Shooter_Subsystem shooter_Subsystem) {
    this.shooter_Subsystem = shooter_Subsystem;
    addRequirements(shooter_Subsystem);
  }
  @Override
  public void initialize() {
    shooter_Subsystem.shoot();    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
  
    }
}


