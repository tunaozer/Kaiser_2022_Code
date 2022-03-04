// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;   


import com.ctre.phoenix.motorcontrol.FeedbackDevice; //Choose the feedback device for a selected sensor. Consult product-specific documentation to determine what is available/supported for your device
import com.ctre.phoenix.motorcontrol.NeutralMode; //Choose the neutral mode for a motor controller
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection; //Collection of simulation commands available to a TalonSRX motor controller.Use the getSimCollection() routine inside your motor controller to create the respective sim collection.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; //CTRE Talon SRX Motor Controller when used on CAN Bus.
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration; //Configurables available to TalonSRX
import com.ctre.phoenix.motorcontrol.ControlMode; //Choose the control mode for a motor controller. Consult product-specific documentation to determine what is available/supported for your device.
import com.ctre.phoenix.motorcontrol.DemandType; //Choose the demand type for the 4 param set

import edu.wpi.first.hal.SimDouble; //A wrapper around a simulator double value handle.
import edu.wpi.first.hal.simulation.SimDeviceDataJNI; //Converts simulation data into Java language.
import edu.wpi.first.wpilibj.SPI; //Represents a SPI bus port.
import edu.wpi.first.wpilibj.drive.DifferentialDrive; //A class for driving differential drive/skid-steer drive platforms such as the Kit of Parts drive base, "tank drive", or West Coast Drive.
import edu.wpi.first.math.geometry.Pose2d; //Represents a 2d pose containing translational and rotational elements.
import edu.wpi.first.math.geometry.Rotation2d; //A rotation in a 2d coordinate frame represented a point on the unit circle (cosine and sine).
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry; //Class for differential drive odometry. Odometry allows you to track the robot's position on the field over the course of a match using readings from 2 encoders and a gyroscope.Teams can use odometry during the autonomous period for complex tasks like path following. Furthermore, odometry can be used for latency compensation when using computer-vision systems.It is important that you reset your encoders to zero before using this class. Any subsequent pose resets also require the encoders to be reset to zero.
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds; //Represents the wheel speeds for a differential drive drivetrain.
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim; //This class simulates the state of the drivetrain. In simulationPeriodic, users should first set inputs from motors with setInputs(double, double), call update(double) to update the simulation, and set estimated encoder and gyro positions, as well as estimated odometry pose. Teams can use Field2d to visualize their robot on the Sim GUI's field.
import edu.wpi.first.wpilibj.smartdashboard.Field2d; //2D representation of game field for dashboards.
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //The SmartDashboard class is the bridge between robot programs and the SmartDashboard on the laptop.
import edu.wpi.first.math.system.plant.DCMotor; //Holds the constants for a DC motor.
import edu.wpi.first.math.trajectory.Trajectory; //Represents a time-parameterized trajectory. The trajectory contains of various States that represent the pose, curvature, time elapsed, velocity, and acceleration at that point.
import edu.wpi.first.math.util.Units; //Utility class that converts between commonly used units in FRC.
import edu.wpi.first.wpilibj2.command.Command; //The Command class is at the very core of the entire command framework. Every command can be started with a call to start(). Once a command is started it will call initialize(), and then will repeatedly call execute() until the isFinished() returns true. Once it does, end() will be called.
import edu.wpi.first.wpilibj2.command.RamseteCommand; //A command that uses a RAMSETE controller (RamseteController) to follow a trajectory Trajectory with a differential drive.
import edu.wpi.first.math.controller.RamseteController; //Ramsete is a nonlinear time-varying feedback controller for unicycle models that drives the model to a desired pose along a two-dimensional trajectory. Why would we need a nonlinear control law in addition to the linear ones we have used so far like PID? If we use the original approach with PID controllers for left and right position and velocity states, the controllers only deal with the local pose. If the robot deviates from the path, there is no way for the controllers to correct and the robot may not reach the desired global pose. This is due to multiple endpoints existing for the robot which have the same encoder path arc lengths.
import edu.wpi.first.wpilibj2.command.SubsystemBase; //A base for subsystems that handles registration in the constructor, and provides a more intuitive method for setting the default command.This class is provided by the NewCommands VendorDep

import frc.robot.Constants.DriveConstants; //Pull the constant values from constant.java

import com.kauailabs.navx.frc.AHRS; // Importing from ROborio lib. The AHRS class provides an interface to AHRS capabilities of the KauaiLabs navX Robotics Navigation Sensor via SPI, I2C and Serial (TTL UART and USB) communications interfaces on the RoboRIO.

public class Drive_Subsystem extends SubsystemBase { // defines motor ports
  private final WPI_VictorSPX left_Master_Motor = new WPI_VictorSPX(DriveConstants.kLeft_Master_Port);
  private final WPI_VictorSPX left_Slave_Motor = new WPI_VictorSpx(DriveConstants.kLeft_Slave_Port);

  private final WPI_TalonSRX right_Master_Motor = new WPI_TalonSRX(DriveConstants.kRight_Master_Port);
  private final WPI_TalonSRX right_Slave_Motor = new WPI_TalonSRX(DriveConstants.kRight_Slave_Port);

  private DifferentialDrive m_drive = new DifferentialDrive(left_Master_Motor, right_Master_Motor);
  public DifferentialDriveOdometry m_odometry; //To track the change of locations with numbers

  public final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private double target;
  private boolean isSlowMode = false;

  TalonSRXSimCollection left_Master_Motor_sim = left_Master_Motor.getSimCollection();
  TalonSRXSimCollection right_Master_Motor_sim = right_Master_Motor.getSimCollection();

  private DifferentialDrivetrainSim m_drive_sim = new DifferentialDrivetrainSim( 
          DCMotor.getCIM(2),
          10.71,
          9.1,
          70,
          Units.inchesToMeters(3),
          0.65,
          null
  );

  private Field2d m_field = new Field2d();

  
  
  public Drive_Subsystem() {

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    left_Slave_Motor.follow(left_Master_Motor);
    right_Slave_Motor.follow(right_Master_Motor);

    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    talonConfig.slot0.kP = DriveConstants.kP;
    talonConfig.slot0.kI = DriveConstants.kI;
    talonConfig.slot0.kD = DriveConstants.kD;
    talonConfig.slot0.integralZone = 400;
    talonConfig.slot0.closedLoopPeakOutput = 1.0;
    talonConfig.openloopRamp = DriveConstants.kOpen_Loop_Ramp;

    left_Master_Motor.configAllSettings(talonConfig);
    right_Master_Motor.configAllSettings(talonConfig);

    resetEncoders();

    right_Master_Motor.setInverted(true);
    left_Master_Motor.setInverted(false);

    left_Slave_Motor.setInverted(false);
    right_Slave_Motor.setInverted(true);

    right_Master_Motor.setSensorPhase(true);
    left_Master_Motor.setSensorPhase(true);

    right_Master_Motor.setSafetyEnabled(false);
    right_Slave_Motor.setSafetyEnabled(false);
    
    left_Master_Motor.setSafetyEnabled(false);
    left_Slave_Motor.setSafetyEnabled(false);
    m_drive.setSafetyEnabled(false);

    left_Master_Motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    right_Master_Motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    SmartDashboard.putData("Field", m_field);



  }


  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), 
    getLeftEncoderDistance(), 
    getRightEncoderDistance());

    System.out.println(getPose());
    m_field.setRobotPose(m_odometry.getPoseMeters());

  }


  @Override
  public void simulationPeriodic() {
    // Set the inputs to the system. Note that we need to use
    // the output voltage, NOT the percent output.
    m_drive_sim.setInputs(left_Master_Motor.getMotorOutputVoltage(),
                          right_Master_Motor.getMotorOutputVoltage()); //Right side is inverted, so forward is negative voltage

    System.out.print(left_Master_Motor.getMotorOutputVoltage());
    System.out.print(right_Master_Motor.getMotorOutputVoltage());

    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    m_drive_sim.update(0.02);  //the time difference

    // Update all of our sensors.
    left_Master_Motor_sim.setQuadratureRawPosition(
                    distanceToNativeUnits(
                      m_drive_sim.getLeftPositionMeters())); //Get the left encoder position in meters.
    left_Master_Motor_sim.setQuadratureVelocity(
                    velocityToNativeUnits(
                      m_drive_sim.getLeftVelocityMetersPerSecond())); //Get the left encoder velocity in meters per second.
    right_Master_Motor_sim.setQuadratureRawPosition(
                    distanceToNativeUnits(
                      m_drive_sim.getRightPositionMeters())); //Get the right encoder position in meters.
    right_Master_Motor_sim.setQuadratureVelocity(
                    velocityToNativeUnits(
                      m_drive_sim.getRightVelocityMetersPerSecond())); //Get the right encoder velocity in meters per second.

                      
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"); //pulling from simulation device datas
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")); //Get rotate degrees.
    angle.set(-m_drive_sim.getHeading().getDegrees());

  }


  private int distanceToNativeUnits(double positionMeters){ // Gidilen mesafeyi veriyor(metre olarak)
    double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(3));
    double motorRotations = wheelRotations * 1;
    int sensorCounts = (int)(motorRotations * 4096);
    return sensorCounts;
  }

  private int velocityToNativeUnits(double velocityMetersPerSecond){ //Gidilen mesafeyi veriyor(saniye olarak)
    double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(3));
    double motorRotationsPerSecond = wheelRotationsPerSecond * 1; 
    double motorRotationsPer100ms = motorRotationsPerSecond / 10;                                    ////?????????
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * 4096);
    return sensorCountsPer100ms;
  }


  public Pose2d getPose() { //The pose of the robot (x and y are in meters).
    return m_odometry.getPoseMeters();
  }


  public DifferentialDriveWheelSpeeds getWheelSpeeds() {  //
    return new DifferentialDriveWheelSpeeds(
        10.0 * left_Master_Motor.getSelectedSensorVelocity() * DriveConstants.kWheel_Circumference_Meters/ DriveConstants.kEncoder_CPR,
        10.0 * right_Master_Motor.getSelectedSensorVelocity() * DriveConstants.kWheel_Circumference_Meters/ DriveConstants.kEncoder_CPR);
  }

  public void arcadeDrive(double speed, double rotation, boolean useSquares) {

    var xSpeed = speed;
    var zRotation = rotation;

    if (xSpeed == 1) { //hız 1 olduğunda alınan rotate değeri
      zRotation *= 0.5;
    }
    else zRotation *=0.7;


    if (useSquares) {   //hızın ve rotasyonun karesini verir
      xSpeed *= Math.abs(xSpeed);
      zRotation *= Math.abs(zRotation);
    }

    if (isSlowMode) { //slowMode'da robotun ne kadar speed ve rotation alcağını gösterir
      xSpeed *= 0.3;
      zRotation *= 0.4;

    }

    xSpeed = Deadband(xSpeed);
    zRotation = Deadband(zRotation);
    
		left_Master_Motor.set(ControlMode.PercentOutput, xSpeed, DemandType.ArbitraryFeedForward, +zRotation); 
	  right_Master_Motor.set(ControlMode.PercentOutput, xSpeed, DemandType.ArbitraryFeedForward, -zRotation);

  }

  public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
    var leftFeedForwardVolts = DriveConstants.FEED_FORWARD.calculate(leftVelocity)/10;
    var rightFeedForwardVolts = DriveConstants.FEED_FORWARD.calculate(rightVelocity)/10;
    

    left_Master_Motor.set( //bu structure yardımıyla motorlara gerekli velocity volts veriliyor
        ControlMode.Velocity, 
        metersPerSecToEdgesPerDecisec(leftVelocity),    
        DemandType.ArbitraryFeedForward,
        leftFeedForwardVolts);
    right_Master_Motor.set(
        ControlMode.Velocity,
        metersPerSecToEdgesPerDecisec(rightVelocity),
        DemandType.ArbitraryFeedForward,
        rightFeedForwardVolts);
  }

  public void resetOdometry(double start_x, double start_y) {
    resetEncoders();
    zeroHeading();
    m_odometry.resetPosition(new Pose2d(start_x, start_y, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0));  
    System.out.print("resseeet");

  }

  public void resetOdometry() {
    resetEncoders();
    zeroHeading();
    m_odometry.resetPosition(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0));  
    System.out.print("resseeet");

  }





  public void tankDriveVolts(double leftVolts, double rightVolts) {
    left_Master_Motor.setVoltage(leftVolts);
    right_Master_Motor.setVoltage(rightVolts);                   //todo
    m_drive.feed(); //Set the expiration time for the corresponding motor safety object.
    System.out.print("Left: ");
    System.out.println(leftVolts);
    System.out.print("Right: ");
    System.out.println(rightVolts);
  
  }

  public Command createCommandForTrajectory(Trajectory trajectory) { //setting ramsete controllers
    return new RamseteCommand(
            trajectory,
            this::getPose,
            new RamseteController(DriveConstants.kRamsete_B, DriveConstants.kRamsete_Zeta),
            DriveConstants.kDrive_Kinematics,
            this::tankDriveVelocity,
            this);
  }


  //Start des Nebenprogramms

  public double getRightEncoderDistance() { //sağ encoder mesafesi alır
    return right_Master_Motor.getSelectedSensorPosition() * DriveConstants.kWheel_Circumference_Meters/ DriveConstants.kEncoder_CPR;
  }

  public double getLeftEncoderDistance() { //sol encoder mesafesi alır
    return left_Master_Motor.getSelectedSensorPosition(0) * DriveConstants.kWheel_Circumference_Meters/ DriveConstants.kEncoder_CPR;
  }

  public double getAverageEncoderDistance() { // ortalama encoder mesafesi
    return (getRightEncoderDistance() + getLeftEncoderDistance()) / (2.0);
  }

  public void setMaxOutput(double maxOutput) { //max output koyar
    m_drive.setMaxOutput(maxOutput);
  }

  public double getTarget() { //* Return the rate of rotation of the yaw (Z-axis) gyro, in degrees per second.
    return getHeading() + target;
  }

  public void setTarget(double val) {
    target = val;
  }

  public void arcadeDrive(double fwd, double rot)   {
    m_drive.arcadeDrive(fwd, rot, true);
  }

  public void zeroHeading() { //gyro resetlemek için kullanılır
    m_gyro.reset();
  }

  public double getHeading() { //hiç anlamadım
    return Math.IEEEremainder(m_gyro.getAngle(), 360.0d) * -1.0d;                         
  }

  public double getTurnRate() { //eğer sonuç doğruysa -1, yanlışsa 1 konulur
    return m_gyro.getRate() * (DriveConstants.kGyro_Reversed ? -1.0 : 1.0);              
  }

  public double getHeadingCW() {
    // Not negating
    return Math.IEEEremainder(m_gyro.getAngle(), 360);
  }

  public double getTurnRateCW() {
    // Not negating
    return m_gyro.getRate();
  }

  public void resetEncoders() { //encoder resetting
    right_Master_Motor.setSelectedSensorPosition(0);
    left_Master_Motor.setSelectedSensorPosition(0);
  }

  public static double metersPerSecToEdgesPerDecisec(double metersPerSec) {
    return metersToEdges(metersPerSec) * .1d;
  }

  public static double metersToEdges(double meters) {  //anlamadım
    return (meters / DriveConstants.kWheel_Circumference_Meters) * DriveConstants.kEncoder_CPR;
  }

  double Deadband(double value) { //Deadband: Aralık değerinin etki etmediği bölüm
		/* Upper deadband */
		if (value >= +0.09) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.09)
			return value;
		
		/* Outside deadband */
		return 0;
  }
  
  public void setBrake() { //sanırım yavaşlama
    left_Master_Motor.setNeutralMode(NeutralMode.Brake);
    right_Master_Motor.setNeutralMode(NeutralMode.Brake);

  }

  public void setCoast() { //hiç anlamadım
    left_Master_Motor.setNeutralMode(NeutralMode.Coast);
    right_Master_Motor.setNeutralMode(NeutralMode.Coast);

  }

  public void stop() { //motorları durdurma
    left_Master_Motor.set(ControlMode.PercentOutput, 0);
    right_Master_Motor.set(ControlMode.PercentOutput, 0);


  }

  public void setVoltageComp () {//anlamadım
    left_Master_Motor.enableVoltageCompensation(true);
    left_Master_Motor.configVoltageCompSaturation(10);

    right_Master_Motor.enableVoltageCompensation(true);
    right_Master_Motor.configVoltageCompSaturation(10);

  }

  public void disableVoltageComp () {//voltajı keser
    left_Master_Motor.enableVoltageCompensation(false);
    right_Master_Motor.enableVoltageCompensation(false);


  }


  public void changeSlowMode() { //slowmode'dan değişim
    if(isSlowMode) isSlowMode=false;
    else isSlowMode = true;

  }
 
  
}
