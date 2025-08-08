// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kAutoT3L = "T4 Left";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SendableChooser<String> m_BreakChooser = new SendableChooser<>();
  private final SendableChooser<String> m_PlaceChooser = new SendableChooser<>();
  private boolean SendOnce = false; // Ensures that the arm command is only sent once

  // Joysticks
  private final Joystick flight = new Joystick(0);
  private final XboxController xBox = new XboxController(1);
  // Flight Buttons
  // private static final int flight1 = 1; // trigger
  // private static final int flight2 = 2; // thumb
  private static final int flight3 = 3;
  private static final int flight4 = 4;
  private static final int flight5 = 5;
  private static final int flight6 = 6;
  private static final int flight7 = 7;
  private static final int flight8 = 8;
  private static final int flight9 = 9;
  private static final int flight10 = 10;
  private static final int flight11 = 11;
  private static final int flight12 = 12;
  // Flight Other
  // private static final int flightPaddle = 3;

  // Drive Control
  protected static final String kArmPos_Intake = "Pos2";
  protected static final String kArmPos_T2 = "Pos3";
  protected static final String kArmPos_T3 = "Pos4";
  protected static final String kArmPos_T4 = "Pos6";
  protected static final String kArmPos_Manual = "Pos7";
  protected static final String kArmPos_Climb_Extend = "Pos8";
  protected static final String kArmPos_Climb_Retract = "Pos9";
  protected static final String kArmPos_Climb_Home = "Pos10";
  protected static final String kArmPos_Climb_Manual = "Pos11";
  protected static final String kArmPos_AT3 = "Pos12";
  protected static final String kArmPos_AT2 = "Pos13";
  protected static final String kArmPos_Idle = "Pos14";
  protected static final String kArmPos_T1 = "Pos15";
  protected static String m_ArmPresets = kArmPos_Idle;
  protected static String m_PreviousArmPresets = kArmPos_Idle;

  protected static final String kReefSide_R = "Right";
  protected static final String kReefSide_L = "Left";
  protected static String m_ReefSide = kReefSide_L;

  protected static final String kBREAK = "Break";
  protected static final String kCOAST = "Coast";
  protected static String m_BreakChoice = kBREAK;

  protected static final String kManualPlace = "Manual";
  protected static final String kAutoPlace = "Auto";
  protected static String m_PlaceChoice = kAutoPlace;

  // USB Camera
  UsbCamera USBCamera;
  UsbCamera USBCamera1;
  VideoSink server;
  private boolean CamToggle;

  // Make PIDTesting Callable
  private PID_Controller pid_Controller;
  public static MecanumDrivetrain m_Drivetrain;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("No Auto", kDefaultAuto);
    m_chooser.addOption("L4 Left", kAutoT3L);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_BreakChooser.setDefaultOption("Break", kBREAK);
    m_BreakChooser.addOption("Coast", kCOAST);
    SmartDashboard.putData("Break Control", m_BreakChooser);

    m_PlaceChooser.setDefaultOption("Automatic", kAutoPlace);
    m_PlaceChooser.addOption("Manual", kManualPlace);
    SmartDashboard.putData("Place Method", m_PlaceChooser);

    // Camera Server enable
    USBCamera = CameraServer.startAutomaticCapture(0);
    USBCamera1 = CameraServer.startAutomaticCapture(1);
    server = CameraServer.getServer();
    USBCamera.setVideoMode(PixelFormat.kMJPEG, 304, 228, 29);
    USBCamera1.setVideoMode(PixelFormat.kMJPEG, 260, 195, 30);
    pid_Controller = new PID_Controller();
    m_Drivetrain = new MecanumDrivetrain();
    pid_Controller.PIDsetup();
    DriveByLime.DriveByLime_Setup();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Temperature
    SmartDashboard.putNumber("Temp Front Right", m_Drivetrain.m_frontRightMotor.getMotorTemperature());
    SmartDashboard.putNumber("Temp Front Left", m_Drivetrain.m_frontLeftMotor.getMotorTemperature());
    SmartDashboard.putNumber("Temp Back Right", m_Drivetrain.m_backRightMotor.getMotorTemperature());
    SmartDashboard.putNumber("Temp Back Left", m_Drivetrain.m_backLeftMotor.getMotorTemperature());

    // Drive Helpers
    SmartDashboard.putNumber("speedMultiplier", m_Drivetrain.speedMultiplier);
    SmartDashboard.putBoolean("Forward Drive", m_Drivetrain.forwardDriveToggle);
    SmartDashboard.putNumber("Pigeon Yaw", m_Drivetrain.pigeon.getYaw().getValue().in(Degrees));
    SmartDashboard.putNumber("Pigeon Roll", m_Drivetrain.pigeon.getRoll().getValue().in(Degrees));
    SmartDashboard.putNumber("Pigeon Pitch", m_Drivetrain.pigeon.getPitch().getValue().in(Degrees));
    SmartDashboard.putNumber("Y", flight.getY());
    SmartDashboard.putNumber("X", flight.getX());
    SmartDashboard.putNumber("Z", flight.getZ());

    // Limelight info
    SmartDashboard.putNumber("Lower TX", LimelightHelpers.getTX(DriveByLime.kBack));
    SmartDashboard.putNumber("Lower X", LimelightHelpers.getTargetPose3d_RobotSpace(DriveByLime.kBack).getX());
    SmartDashboard.putNumber("Lower Z", LimelightHelpers.getTargetPose3d_RobotSpace(DriveByLime.kBack).getZ());
    SmartDashboard.putNumber("Lower Yaw",
        LimelightHelpers.getTargetPose3d_RobotSpace(DriveByLime.kBack).getRotation().getY());

    SmartDashboard.putNumber("Upper TX", LimelightHelpers.getTX(DriveByLime.kFront));
    SmartDashboard.putNumber("Upper X", LimelightHelpers.getTargetPose3d_RobotSpace(DriveByLime.kFront).getX());
    SmartDashboard.putNumber("Upper Z", LimelightHelpers.getTargetPose3d_RobotSpace(DriveByLime.kFront).getZ());
    SmartDashboard.putNumber("Upper Yaw",
        LimelightHelpers.getTargetPose3d_RobotSpace(DriveByLime.kFront).getRotation().getY());

    SmartDashboard.putString("Reef Side", m_ReefSide);

    pid_Controller.PIDperiodic();

    // Break Toggle
    if (!(m_BreakChoice == m_BreakChooser.getSelected())) {
      m_BreakChoice = m_BreakChooser.getSelected();
      switch (m_BreakChoice) {
        case kBREAK:
          pid_Controller.MotorBreakOn();
          break;
        case kCOAST:
          pid_Controller.MotorBreakOFF();
          break;
      }
    }

    // Camera Toggle
    if (xBox.getXButton()) {
      if (CamToggle == true) {
        CamToggle = false;
        server.setSource(USBCamera);
      } else {
        CamToggle = true;
        server.setSource(USBCamera1);
      }
      // prevent doubleclick of the flip button
      try {
        Thread.sleep(300);
      } catch (InterruptedException ex) {
        Thread.currentThread().interrupt();
      }
    }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    DriveByLime.AutoStepCnt = 0;
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    SendOnce = false;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    pid_Controller.ClimbPID(pid_Controller.kClimbHome);
    if (!SendOnce) {
      switch (m_autoSelected) {
        case kAutoT3L:
          m_ArmPresets = kArmPos_T4;
          m_ReefSide = kReefSide_L;
          break;
        case kDefaultAuto:
        default:
          // Put default auto code here
          break;
      }
      SendOnce = true;
    }
    ArmPosing();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    DriveByLime.AutoStepCnt = 5;
    CamToggle = false;
    server.setSource(USBCamera);
    // m_ArmPresets = kArmPos_Intake;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // ********************************
    // * DRIVE CODE
    // ********************************
    pid_Controller.Controls(flight, xBox, m_ArmPresets);
    if (DriveByLime.AutoStepCnt == 5) {
      m_Drivetrain.driveWithJoystick(flight, false);
    } else {
      pid_Controller.ClimbPID(pid_Controller.kClimbHome);
    }
    ArmPosing();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    DriveByLime.AutoStepCnt = 0;
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    DriveByLime.DriveTest(DriveByLime.kFront, DriveByLime.kRobotPoseT3_L, kArmPos_T3);
    ArmPosing();
    pid_Controller.ClimbPID(pid_Controller.kClimbHome);
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  // ********************************
  // * CUSTOM FUNCTIONS
  // ********************************

  /**
   * This function controls the arm posing, allowing for calls outside of
   * teleoperated periods
   */
  public void ArmPosing() {
    if (!(flight.getPOV() == -1)) {
      if (flight.getPOV() >= 45 && flight.getPOV() <= 135) {
        m_ReefSide = kReefSide_R;
      } else if (flight.getPOV() >= 225 && flight.getPOV() <= 315) {
        m_ReefSide = kReefSide_L;
      }
    }

    if (flight.getRawButton(flight7)) { // This sets the current hold mode
      m_PlaceChoice = m_PlaceChooser.getSelected();
      switch (m_PlaceChoice) { // Allow for manual placement override.
        case kAutoPlace:
          m_ArmPresets = kArmPos_T2;
          DriveByLime.AutoStepCnt = 0;
          break;
        case kManualPlace:
          m_ArmPresets = kArmPos_T2;
          DriveByLime.AutoStepCnt = 5;
          break;
      }
      m_PreviousArmPresets = kArmPos_T2;
    } else if (flight.getRawButton(flight8)) {
      if (pid_Controller.s_coralSensor.get()) {
        m_ArmPresets = kArmPos_Idle;
      } else {
        m_ArmPresets = kArmPos_Intake;
      }
      DriveByLime.AutoStepCnt = 5;
    } else if (flight.getRawButton(flight9)) {
      m_PlaceChoice = m_PlaceChooser.getSelected();
      switch (m_PlaceChoice) { // Allow for manual placement override.
        case kAutoPlace:
          m_ArmPresets = kArmPos_T3;
          DriveByLime.AutoStepCnt = 0;
          break;
        case kManualPlace:
          m_ArmPresets = kArmPos_T3;
          DriveByLime.AutoStepCnt = 5;
          break;
      }
      m_PreviousArmPresets = kArmPos_T3;
    } else if (flight.getRawButton(flight11)) {
      m_PlaceChoice = m_PlaceChooser.getSelected();
      switch (m_PlaceChoice) { // Allow for manual placement override.
        case kAutoPlace:
          m_ArmPresets = kArmPos_T4;
          DriveByLime.AutoStepCnt = 0;
          break;
        case kManualPlace:
          m_ArmPresets = kArmPos_T4;
          DriveByLime.AutoStepCnt = 5;
          break;
      }
      m_PreviousArmPresets = kArmPos_T4;
    } else if (flight.getRawButton(flight10)) {
      m_ArmPresets = kArmPos_AT2;
      DriveByLime.AutoStepCnt = 5;
    } else if (flight.getRawButton(flight12)) {
      m_ArmPresets = kArmPos_AT3;
      DriveByLime.AutoStepCnt = 5;
    } else if (xBox.getLeftBumperButton()) {
      m_ArmPresets = kArmPos_T1;
    } else if (flight.getRawButton(flight3) || flight.getRawButton(flight4) || flight.getRawButton(flight5)
        || flight.getRawButton(flight6)) {
      DriveByLime.AutoStepCnt = 5;
      m_ArmPresets = kArmPos_Manual;
    } else if (xBox.getAButton()) {
      DriveByLime.AutoStepCnt = 5;
      m_ArmPresets = kArmPos_Climb_Retract;
    } else if (xBox.getBButton()) {
      DriveByLime.AutoStepCnt = 5;
      m_ArmPresets = kArmPos_Climb_Extend;
    } else if (xBox.getYButton()) {
      DriveByLime.AutoStepCnt = 5;
      m_ArmPresets = kArmPos_Climb_Home;
    } else if (xBox.getPOV() == 0 || xBox.getPOV() == 180) {
      DriveByLime.AutoStepCnt = 5;
      m_ArmPresets = kArmPos_Climb_Manual;
    } else if (flight.getRawButton(flight11)) { // this is set for clearing level 3 algae from below
      DriveByLime.AutoStepCnt = 5;
      m_ArmPresets = kArmPos_AT3;
    } else if (xBox.getPOV() == 90) {
      DriveByLime.AutoStepCnt = 5;
      m_ArmPresets = kArmPos_AT2;
    }

    switch (m_ArmPresets) { // Hold arm in selected position
      case kArmPos_Intake:
        pid_Controller.ArmPID(0.309, 0.955); // Intake
        pid_Controller.ClimbPID(pid_Controller.kClimbHome);
        break;
      case kArmPos_T2:
        pid_Controller.ArmPID(0.4925, 0.185); // t2
        pid_Controller.ClimbPID(pid_Controller.kClimbHome);
        switch (m_ReefSide) {
          case kReefSide_L:
            DriveByLime.DriveTest(DriveByLime.kFront, DriveByLime.kRobotPoseT2_L, kArmPos_T2);
            break;
          case kReefSide_R:
            DriveByLime.DriveTest(DriveByLime.kFront, DriveByLime.kRobotPoseT2_R, kArmPos_T2);
            break;
        }
        break;
      case kArmPos_T3:
        pid_Controller.ArmPID(0.5252, 0.22); // t3
        pid_Controller.ClimbPID(pid_Controller.kClimbHome);
        switch (m_ReefSide) {
          case kReefSide_L:
            DriveByLime.DriveTest(DriveByLime.kFront, DriveByLime.kRobotPoseT3_L, kArmPos_T3);
            break;
          case kReefSide_R:
            DriveByLime.DriveTest(DriveByLime.kFront, DriveByLime.kRobotPoseT3_R, kArmPos_T3);
            break;
        }
        break;
      case kArmPos_T4:
        pid_Controller.ArmPID(0.725, 0.367); // t4
        pid_Controller.ClimbPID(pid_Controller.kClimbHome);
        switch (m_ReefSide) {
          case kReefSide_L:
            DriveByLime.DriveTest(DriveByLime.kBack, DriveByLime.kRobotPoseT4_L, kArmPos_T4);
            break;
          case kReefSide_R:
            // DriveByLime.DriveTest(DriveByLime.kBack, DriveByLime.kRobotPoseT4_R,
            // kArmPos_T4);
            DriveByLime.AutoStepCnt = 5;
            break;
        }
        break;
      case kArmPos_Manual:
        // This is intentionally left blank
        break;
      case kArmPos_Climb_Extend:
        pid_Controller.ArmPID(0.1744, 0.7833); // Climbing
        pid_Controller.ClimbPID(pid_Controller.kClimbExtend);
        break;
      case kArmPos_Climb_Retract:
        pid_Controller.ArmPID(0.1744, 0.7833); // Climbing
        if (pid_Controller.m_Joint1Encoder() < 0.185) { // Prevent arm collision on retracting
          pid_Controller.ClimbPID(pid_Controller.kClimbRetract);
        } else {
          pid_Controller.ClimbPID(pid_Controller.kClimbHome);
        }
        break;
      case kArmPos_Climb_Home:
        pid_Controller.ArmPID(0.1744, 0.7833); // Climbing
        pid_Controller.ClimbPID(pid_Controller.kClimbHome);
        break;
      case kArmPos_Climb_Manual:
        pid_Controller.ArmPID(0.1744, 0.7833); // Climbing
        break;
      case kArmPos_AT3:
        pid_Controller.ArmPID(0.545, 0.22); // Level 3 Algae
        pid_Controller.ClimbPID(pid_Controller.kClimbHome);
        break;
      case kArmPos_AT2:
        pid_Controller.ArmPID(0.31, 0.8); // Level 2 Algae
        pid_Controller.ClimbPID(pid_Controller.kClimbHome);
        break;
      case kArmPos_Idle:
        pid_Controller.ArmPID(0.230, 0.843); // Idle Pose
        pid_Controller.ClimbPID(pid_Controller.kClimbHome);
        break;
      case kArmPos_T1:
        pid_Controller.ArmPID(0.230, 0.785); // Trough Coral
        pid_Controller.ClimbPID(pid_Controller.kClimbHome);
        break;
    }
  }

  /**
   * This function creates a wait
   * 
   * @param time = Time to wait in milliseconds.
   */
  public static void wait(int time) {
    try {
      Thread.sleep(time);
    } catch (InterruptedException ex) {
      Thread.currentThread().interrupt();
    }
  }
}