package frc.robot.Subsystems;


import  com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import  edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;




public class Climber_Elevator_Subsystem extends SubsystemBase {

    private final CANSparkMax m_climber_elevator = new CANSparkMax(ClimberConstants.kClimber_Elevator_Motor_Port,MotorType.kBrushless);
    private final DoubleSolenoid m_climber_solenoid=  new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7,8);
    

    private boolean climber_pneumatic_state = false;
    private boolean climber_motor_state = false;

    public Climber_Elevator_Subsystem() {
        open_Climber();

    }
    @Override
    public void periodic() {

    }

    
  public void change_Climber() { // Intake'i döndürür/durdurur
  if(!climber_motor_state && climber_pneumatic_state){
  close_Climber();
  }
  else {
    open_Climber();
  }
}

    public void elevator_Up(double speed) {
        m_climber_elevator.set(1);
    }

    public void elevator_Stop() {
        m_climber_elevator.set(0);
    }

    public void elevator_Down(){
        m_climber_elevator.set(-1);
    }
    public void open_Climber() {   //INtake açılır
        m_climber_solenoid.set(Value.kForward);
        climber_pneumatic_state = true;
      }

      public void close_Climber() {   //INtake açılır
        m_climber_solenoid.set(Value.kReverse);
        climber_pneumatic_state = false;
      }  


}




    

    








