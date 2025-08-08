package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PID_Controller {
  // Sensors
  final DigitalInput s_coralSensor = new DigitalInput(0);

  // Motor Declaration
  public static final SparkMax m_Joint1Follower = new SparkMax(5, MotorType.kBrushless);
  public static final SparkMax m_Joint1 = new SparkMax(6, MotorType.kBrushless);
  public static final SparkMax m_Joint2 = new SparkMax(7, MotorType.kBrushless);
  final static SparkMax m_coralIntake = new SparkMax(8, MotorType.kBrushless);
  public static final SparkMax m_ClimbFollower = new SparkMax(9, MotorType.kBrushless);
  public static final SparkMax m_Climb = new SparkMax(10, MotorType.kBrushless);

  // Configs
  SparkMaxConfig c_BREAK = new SparkMaxConfig();
  SparkMaxConfig c_COAST = new SparkMaxConfig();

  // PID config
  PIDController p_Joint1PID = new PIDController(7, 0.005, 0);
  PIDController p_Joint2PID = new PIDController(4.2, 0.005, 0);
  PIDController p_ClimberPID = new PIDController(12, 0.005, 0);

  // Climb presets
  final double kClimbHome = 0.6858;
  final double kClimbExtend = 0.9297;
  final double kClimbRetract = 0.39;

  /**
   * Sets up the PID Class and confirms SparkMak configs. Also sets up the
   * controls class
   */
  public void PIDsetup() {

    // Break setup
    c_BREAK.idleMode(IdleMode.kBrake);
    c_COAST.idleMode(IdleMode.kCoast);

    // Apply configs
    MotorBreakOn();

    // PID setting configs
    p_Joint1PID.disableContinuousInput();
    p_Joint1PID.reset();
    p_Joint1PID.setTolerance(0.0);

    p_Joint2PID.disableContinuousInput();
    p_Joint2PID.reset();
    p_Joint2PID.setTolerance(0.0);

    p_ClimberPID.disableContinuousInput();
    p_ClimberPID.reset();
    p_ClimberPID.setTolerance(0.0);
  }

  /**
   * Apply break config
   */
  public void MotorBreakOn() {
    m_Joint1.configure(c_BREAK, null, null);
    m_Joint1Follower.configure(c_BREAK, null, null);
    m_Joint2.configure(c_BREAK, null, null);
    m_coralIntake.configure(c_BREAK, null, null);
    m_ClimbFollower.configure(c_BREAK, null, null);
    m_Climb.configure(c_BREAK, null, null);
  }

  /**
   * Apply coast config
   */
  public void MotorBreakOFF() {
    m_Joint1.configure(c_COAST, null, null);
    m_Joint1Follower.configure(c_COAST, null, null);
    m_Joint2.configure(c_COAST, null, null);
    m_coralIntake.configure(c_COAST, null, null);
    m_ClimbFollower.configure(c_COAST, null, null);
    m_Climb.configure(c_COAST, null, null);
  }

  /**
   * Update values in Smart Dashboard for debugging.
   */
  public void PIDperiodic() {
    SmartDashboard.putNumber("Joint1 Position", m_Joint1.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("Joint2 Position", m_Joint2.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("Climb Position", m_Climb.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("Coral Position", m_coralIntake.getEncoder().getPosition());

    // Sensor info
    SmartDashboard.putBoolean("Coral Sensor", s_coralSensor.get());

    // Motor Temps
    SmartDashboard.putNumber("Temp Coral", m_coralIntake.getMotorTemperature());
    SmartDashboard.putNumber("Joint1 Lead", m_Joint1.getMotorTemperature());
    SmartDashboard.putNumber("Joint1 Follower", m_Joint1Follower.getMotorTemperature());
    SmartDashboard.putNumber("Joint2", m_Joint2.getMotorTemperature());
    SmartDashboard.putNumber("Climb Lead", m_Climb.getMotorTemperature());
    SmartDashboard.putNumber("Climb Follower", m_ClimbFollower.getMotorTemperature());

    SmartDashboard.putNumber("Climb Lead Current", m_Climb.getOutputCurrent());
    SmartDashboard.putNumber("Climb Follower Current", m_ClimbFollower.getOutputCurrent());
  }

  // ****************************************************************
  // ****************************************************************
  // * CONTROLS CODE
  // ****************************************************************
  // ****************************************************************

  /**
   * Sets the arm position to the specified value
   * 
   * @param Joint1_Target_Angle = The target position of joint 1, value must be
   *                            0-1
   * @param Joint2_Target_Angle = The target position of joint 2, value must be
   *                            0-1
   */
  public void ArmPID(double Joint1_Target_Angle, double Joint2_Target_Angle) {
    // Joint 1 PID Calc
    if (m_Climb.getAbsoluteEncoder().getPosition() > 0.66
        || (Joint1_Target_Angle == 0.1754 && Joint2_Target_Angle == 0.7345)) { // Prevent arm collision
      if (m_Joint1.getAbsoluteEncoder().getPosition() < 0.62) {
        if (m_Joint2.getAbsoluteEncoder().getPosition() > 0.52 // Value must be below before J1 moves (T2 & T3)
            && (Joint2_Target_Angle == 0.185 || Joint2_Target_Angle == 0.22)) { // Only T2 & T3
          m_Joint1.set(MathUtil
              .clamp(p_Joint1PID.calculate(m_Joint1.getAbsoluteEncoder().getPosition(), 0.2875), -0.175,
                  0.5)); // Hold J1 for J2 init movement
        } else {
          m_Joint1.set(MathUtil
              .clamp(p_Joint1PID.calculate(m_Joint1.getAbsoluteEncoder().getPosition(), Joint1_Target_Angle), -0.175,
                  0.5));
        }
      } else {
        m_Joint1.set(MathUtil.clamp(
            p_Joint1PID.calculate(m_Joint1.getAbsoluteEncoder().getPosition(), Joint1_Target_Angle), -0.175, 0.175));
      }
      // Joint 2 PID calc
      if (m_Joint1.getAbsoluteEncoder().getPosition() >= 0.366 // Value must be below before J2 returns home (T2 & T3)
          && (Joint1_Target_Angle == 0.309 || Joint1_Target_Angle == 0.230)
          && !(Robot.m_PreviousArmPresets == Robot.kArmPos_T4)) {
        m_Joint2.set(MathUtil.clamp(p_Joint2PID.calculate(m_Joint2.getAbsoluteEncoder().getPosition(), 0.14), -1, 1));
      } else if (m_Joint1.getAbsoluteEncoder().getPosition() > 0.66
          && (Joint1_Target_Angle == 0.309 || Joint1_Target_Angle == 0.230)) { // Don't crash into the reef from T4
        m_Joint2.set(0);
      } else {
        if (Joint2_Target_Angle == 0.367) { // SLOW if T4
          m_Joint2.set(MathUtil.clamp(
              p_Joint2PID.calculate(m_Joint2.getAbsoluteEncoder().getPosition(), Joint2_Target_Angle), -0.4, 0.4));
        } else {
          if (m_Joint1.getAbsoluteEncoder().getPosition() <= 0.4775 // During upward movement
              && (Joint2_Target_Angle == 0.185 || Joint2_Target_Angle == 0.22)) { // Prevent extension violation (T 2&3)
            m_Joint2.set(MathUtil.clamp(
                p_Joint2PID.calculate(m_Joint2.getAbsoluteEncoder().getPosition(), 0.14), -1, 1));
          } else {
            m_Joint2.set(MathUtil.clamp(
                p_Joint2PID.calculate(m_Joint2.getAbsoluteEncoder().getPosition(), Joint2_Target_Angle), -1, 1));
          }
        }
      }
    } else {
      m_Joint1.set(0);
      m_Joint2.set(0);
    }
  }

  /**
   * Sets the climber position to the specified value
   * 
   * @param Climb_Target_Angle = The target position of joint 1, value must be
   *                           0-1
   */
  public void ClimbPID(double Climb_Target_Angle) {
    // Joint 1 PID Calc
    if (m_Climb.getAbsoluteEncoder().getPosition() > 0.425
        || (m_Climb.getAbsoluteEncoder().getPosition() > 0.395 && Climb_Target_Angle == kClimbRetract)) {
      m_Climb.set(MathUtil.clamp(
          p_ClimberPID.calculate(m_Climb.getAbsoluteEncoder().getPosition(), Climb_Target_Angle), -0.3, 0.3));
    } else {
      m_Climb.set(0);
    }
  }

  /**
   * Returns the current position of the climber's absolute encoder
   */
  public double m_ClimbEncoder() {
    return m_Climb.getAbsoluteEncoder().getPosition();
  }

  /**
   * Returns the current position of the climber's absolute encoder
   */
  public double m_Joint1Encoder() {
    return m_Joint1.getAbsoluteEncoder().getPosition();
  }

  private boolean first_press;

  /**
   * This allows for the manual controls of the arm and intakes as well as other
   * PID motor devices.
   * 
   * @param joystick = The flightstick being used to instantiate control commands
   * @param xBox     = The Xbox controller being used to instantiate control
   *                 commands
   * @param ArmCase  = The current ArmCase
   */
  public void Controls(Joystick joystick, XboxController xBox, String ArmCase) {
    // Check for first press of button

    if (ArmCase == Robot.kArmPos_Intake) {
      if (first_press && Math.abs(m_coralIntake.getEncoder().getPosition()) < 0.025) {
        m_coralIntake.set(0.1);
      } else if (!s_coralSensor.get()) {
        m_coralIntake.set(0.2);
      } else if (s_coralSensor.get() && !first_press) {
        m_coralIntake.getEncoder().setPosition(0);
        first_press = true;
      } else {
        Robot.m_ArmPresets = Robot.kArmPos_Idle;
        first_press = false;
        m_coralIntake.set(0);
      }
    } else if (joystick.getRawButton(1)) {
      if (ArmCase == Robot.kArmPos_T4) {
        m_coralIntake.set(0.2);
      } else if (ArmCase == Robot.kArmPos_AT2) {
        m_coralIntake.set(1);
      } else if (ArmCase == Robot.kArmPos_AT3) {
        m_coralIntake.set(-1);
      } else if ((ArmCase == Robot.kArmPos_T2 || ArmCase == Robot.kArmPos_T3)) {
        m_coralIntake.set(-0.2);
      } else if ((ArmCase == Robot.kArmPos_T1)){
        m_coralIntake.set(-0.4);
      } else if (!first_press && !(ArmCase == Robot.kArmPos_Idle)) {
        m_coralIntake.set(-0.175);
      } else if (s_coralSensor.get()) {
        Robot.m_ArmPresets = Robot.kArmPos_Idle;
        m_coralIntake.set(0);
      } else {
        m_coralIntake.set(0);
      }
    } else {
      if (DriveByLime.AutoStepCnt == 5) {
        m_coralIntake.set(0);
      }
    }

    if (joystick.getRawButton(6) && m_Joint2.getAbsoluteEncoder().getPosition() < 0.95) { // Controls the 2nd joint
      m_Joint2.set(0.2);
    } else if (joystick.getRawButton(4) && m_Joint2.getAbsoluteEncoder().getPosition() > 0.21) {
      m_Joint2.set(-0.2);
    } else if (ArmCase == Robot.kArmPos_Manual) {
      m_Joint2.set(0);
    }

    if (joystick.getRawButton(5) && m_Joint1.getAbsoluteEncoder().getPosition() < 0.73) { // Controls the 1st joint
      m_Joint1.set(0.2);
    } else if (joystick.getRawButton(3) && m_Joint1.getAbsoluteEncoder().getPosition() > 0.2) {
      m_Joint1.set(-0.2);
    } else if (ArmCase == Robot.kArmPos_Manual) {
      m_Joint1.set(0);
    }

    if (xBox.getPOV() == 0 && m_Climb.getAbsoluteEncoder().getPosition() > 0.419
        && m_Climb.getAbsoluteEncoder().getPosition() < 0.975) {
      m_Climb.set(0.3);
    } else if (xBox.getPOV() == 180 && m_Climb.getAbsoluteEncoder().getPosition() > 0.41) {
      m_Climb.set(-0.3);
    } else if (ArmCase == Robot.kArmPos_Climb_Manual) {
      m_Climb.set(0);
    }
  }
}