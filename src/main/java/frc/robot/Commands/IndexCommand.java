// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;
import frc.robot.Subsystems.Indexer_Subsystem;
import frc.robot.Subsystems.Intake_Subsystem;
import frc.robot.Subsystems.Shooter_Subsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexCommand extends CommandBase {

  private final Indexer_Subsystem indexer_Subsystem;
  private final Shooter_Subsystem shooter_Subsystem;
  private final Intake_Subsystem intake_Subsystem;
// Use addRequirements() here to declare subsystem dependencies.
  public IndexCommand(Indexer_Subsystem indexer_Subsystem, Shooter_Subsystem shooter_Subsystem, Intake_Subsystem intake_Subsystem) {
    this.indexer_Subsystem = indexer_Subsystem;
    this.shooter_Subsystem = shooter_Subsystem;
    this.intake_Subsystem = intake_Subsystem;
    addRequirements(indexer_Subsystem);
  }   


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer_Subsystem.start_Indexer(shooter_Subsystem.shooter_Is_Ready()); 
    intake_Subsystem.start_Intake();   

  }

  // Called once the command ends or is interrupted.


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
   // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer_Subsystem.stop_Indexer();
    intake_Subsystem.stop_Intake();
    
  }
}



 






