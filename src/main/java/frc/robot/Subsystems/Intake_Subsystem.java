package frc.robot.Subsystems;

//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;



public class Intake_Subsystem extends SubsystemBase {  

    private final WPI_VictorSPX m_intake_Motor = new WPI_VictorSPX(IntakeConstants.kIntake_Motor_Port);
    

    

    private boolean intake_pneumatic_state = false;
    private boolean intake_motor_state = false;




  public void change_Intake() { // Intake'i döndürür/durdurur
    if(!intake_motor_state && intake_pneumatic_state){
    start_Intake();
    }
    else {
      stop_Intake();
    }
  }



  public void start_Intake() { //Intake dönmeye başlar
    m_intake_Motor.set(IntakeConstants.kIntake_Motor_speed);
    intake_motor_state = true;
  }

  public void stop_Intake() {  //Intake dönmeyi bırakır
    m_intake_Motor.set(0);
    intake_motor_state = false;
  }


}