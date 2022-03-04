// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Climber_Elevator_Subsystem;

public class ClimbDownCommand extends CommandBase { 

  private final Climber_Elevator_Subsystem climber_Elevator_Subsystem;

  public ClimbDownCommand(Climber_Elevator_Subsystem climber_Elevator_Subsystem) {
    this.climber_Elevator_Subsystem = climber_Elevator_Subsystem;

    addRequirements(climber_Elevator_Subsystem);
  }
  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    climber_Elevator_Subsystem.elevator_Up(); 

  }
// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
    climber_Elevator_Subsystem.elevator_Down();
  
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}








