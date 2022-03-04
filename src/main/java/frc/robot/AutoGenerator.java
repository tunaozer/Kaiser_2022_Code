package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;//A class that enforces constraints on differential drive voltage expenditure based on the motor dynamics and the drive kinematics. Ensures that the acceleration of any wheel of the robot while following the trajectory is never higher than what can be achieved with the given maximum voltage.
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Commands.*;
import frc.robot.Subsystems.*;

public class AutoGenerator {
    private final Drive_Subsystem m_drive;
    private final Shooter_Subsystem m_shooter;
    private final Indexer_Subsystem m_indexer;
    private final Intake_Subsystem m_intake;
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public AutoGenerator(Drive_Subsystem m_drive, Shooter_Subsystem m_shooter, Indexer_Subsystem m_indexer, Intake_Subsystem m_intake) {
      this.m_drive = m_drive;
      this.m_shooter = m_shooter;
      this.m_intake = m_intake;
      this.m_indexer = m_indexer;
    }

}
