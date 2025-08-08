package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MecanumDrivetrain extends SubsystemBase {
  // Motor controllers for the four wheels
  protected final SparkMax m_frontLeftMotor = new SparkMax(1, MotorType.kBrushless);
  protected final SparkMax m_frontRightMotor = new SparkMax(3, MotorType.kBrushless);
  protected final SparkMax m_backLeftMotor = new SparkMax(2, MotorType.kBrushless);
  protected final SparkMax m_backRightMotor = new SparkMax(4, MotorType.kBrushless);

  // Configuration for motor inversion and idle modes
  SparkMaxConfig c_INVERTED1 = new SparkMaxConfig();
  SparkMaxConfig c_NORMAL1 = new SparkMaxConfig();

  // Pigeon IMU for orientation tracking
  final Pigeon2 pigeon = new Pigeon2(49);

  // MecanumDrive object to control the robot's mecanum drivetrain
  private final MecanumDrive mecanumDrive;

  double xCorrect = -1; // used to smooth out turning
  boolean forwardDriveToggle = false;
  double speedMultiplier;
  protected double j_rotDeadZone = 0.35;

  private final PIDController rotationController = new PIDController(0.02, 0, 0);

  public MecanumDrivetrain() {
    // Configure motor inversion and idle mode settings
    c_INVERTED1.inverted(true);
    c_NORMAL1.inverted(false);
    c_INVERTED1.idleMode(IdleMode.kBrake);
    c_NORMAL1.idleMode(IdleMode.kBrake);

    m_frontLeftMotor.configure(c_NORMAL1, null, null);
    m_frontRightMotor.configure(c_INVERTED1, null, null);
    m_backLeftMotor.configure(c_NORMAL1, null, null);
    m_backRightMotor.configure(c_INVERTED1, null, null);

    // Initialize the MecanumDrive object with motor controllers
    mecanumDrive = new MecanumDrive(m_frontLeftMotor, m_backLeftMotor, m_frontRightMotor, m_backRightMotor);
    mecanumDrive.setSafetyEnabled(false);

    // Configure the PID controller for smooth rotation handling
    rotationController.enableContinuousInput(-180.0, 180.0);
  }

  /**
   * Drives the robot using a joystick.
   * 
   * @param joystick     The joystick to control the robot.
   * @param fieldCentric If true, uses field-centric controls; otherwise,
   *                     robot-centric.
   */
  public void driveWithJoystick(Joystick joystick, boolean fieldCentric) {
    // Get joystick inputs
    double xSpeed = -joystick.getX(); // Strafe
    double ySpeed = joystick.getY(); // Forward/Backward (Y is typically inverted)
    double rotation = joystick.getTwist(); // Rotation

    if (!(Math.abs(rotation) >= j_rotDeadZone + (Math.abs(xSpeed) * 0.3))
        && !(Math.abs(rotation) >= j_rotDeadZone + (Math.abs(ySpeed) * 0.3))) { // Setup controller deadzone
      rotation = 0;
    }
    speedMultiplier = (((joystick.getRawAxis(3) * -1) * 0.2) + .6); // Speed multiplier

    // Flip logic
    if (joystick.getRawButton(2)) {
      if (forwardDriveToggle == true) {
        forwardDriveToggle = false;
        xCorrect = Math.abs(xCorrect) * -1;
      } else {
        forwardDriveToggle = true;
        xCorrect = Math.abs(xCorrect);
      }
      // prevent doubleclick of the flip button
      try {
        Thread.sleep(300);
      } catch (InterruptedException ex) {
        Thread.currentThread().interrupt();
      }
    }

    if (fieldCentric) {
      // Adjust movement based on robot orientation
      Rotation2d gyroAngle = Rotation2d.fromDegrees(pigeon.getYaw().getValue().in(Degrees));
      mecanumDrive.driveCartesian(ySpeed * speedMultiplier * xCorrect, xSpeed * speedMultiplier * xCorrect,
          Math.pow(rotation, 3) * .45,
          gyroAngle);
    } else {
      // Direct robot-centric control
      mecanumDrive.driveCartesian(ySpeed * speedMultiplier * xCorrect, xSpeed * speedMultiplier * xCorrect,
          Math.pow(rotation, 3) * .45);
    }
  }

  /**
   * This is for use with DriveByLime only!
   * 
   * @param Distance = The robot's speed along the X axis [-1.0..1.0]. Forward is
   *                 positive.
   * @param Strafe   = The robot's speed along the Y axis [-1.0..1.0]. Left is
   *                 positive.
   * @param Yaw      = The robot's rotation rate around the Z axis [-1.0..1.0].
   *                 Counterclockwise is positive.
   */
  public void pid_drive(double Distance, double Strafe, double Yaw) {
    mecanumDrive.driveCartesian(Distance, Strafe, Yaw);
  }
}