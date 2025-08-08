package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;

public class DriveByLime {
  static PIDController p_Distance = new PIDController(0.535, 0.01, 0.1); // Distance
  static PIDController p_Strife = new PIDController(1.94, 0.01, 0.2); // Strife
  static PIDController p_Yaw = new PIDController(0.287, 0.01, 0); // Rotation
  static PIDController p_TX = new PIDController(0.0075, 0.00075, 0); // Rotation
  static final String kFront = "limelight-front";
  static final String kBack = "limelight-back";

  /**
   * These are presets for the auto-posing of the robot.
   * 
   * @see kRobotPose[0] = Distance offset from tag {meters}
   * @see kRobotPose[1] = X offset (Strife) {meters}
   * @see kRobotPose[2] = Y-axis rotation setPoint (0 is recommended) {degrees}
   * @see kRobotPose[3] = Intake speed {-0.2 or 0.2}
   */
  static final double[] kRobotPoseT2_L = { 0.818, 0.35, -0.118, -0.2 }; // T2 Left
  static final double[] kRobotPoseT2_R = { 0.78, 0.01, 0.0281, -0.2 }; // T2 Right
  static final double[] kRobotPoseT3_L = { 0.81, 0.35, 0.02, -0.2 }; // T3 Left
  static final double[] kRobotPoseT3_R = { 0.803, 0.024, 0.015, -0.2 }; // T3 Right
  static final double[] kRobotPoseT4_L = { 0.573, 0.085, -0.297, 0.2 };// T4 Left
  static final double[] kRobotPoseT4_R = { 0.787, 0.0538, -0.052, 0.2 };// TODO T4 Right

  /**
   * Sets up the DriveByLime Class
   */
  public static void DriveByLime_Setup() {
    p_Yaw.setTolerance(0.055);
    p_Strife.setTolerance(0.037);
    p_Distance.setTolerance(0.035);
    p_TX.setTolerance(0.025);

    p_Yaw.disableContinuousInput();
    p_Strife.disableContinuousInput();
    p_Distance.disableContinuousInput();
    p_TX.disableContinuousInput();

    p_Yaw.reset();
    p_Strife.reset();
    p_Distance.reset();
    p_TX.reset();
  }

  public static double AutoStepCnt = 0;
  private static double Time = 0;
  private static boolean DontCrash = false; // This prevents the robot from moving until the arm is out of the way.

  /**
   * Face the nearest apriltag in a given limelight's FOV
   * 
   * @param limelightName   = Which limelight to use, can be "kUpper" or "KLower"
   * @param robotPosePreset = Which robot pose preset to use kRobotPoseX
   * @param ArmPose         = Which arm preset to use
   */
  public static void DriveTest(String limelightName, double[] robotPosePreset, String ArmPose) {
    if (!(LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getZ() == 0) && !(AutoStepCnt == 5)) {
      switch (limelightName) {
        case kFront:
          // Go to specified arm pose when in range.
          if (LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getZ() <= 2.75 && AutoStepCnt == 0) {
            Robot.m_ArmPresets = ArmPose;
          }

          /*
           * Prevent the robot from crashing into the reef during T1 & T2 autos.
           */
          if (AutoStepCnt == 0 && (ArmPose == Robot.kArmPos_T2 || ArmPose == Robot.kArmPos_T3)
              && PID_Controller.m_Joint1.getAbsoluteEncoder().getPosition() <= 0.362) {
            DontCrash = false;
            Robot.m_Drivetrain.pid_drive(0, 0, 0);
          } else {
            DontCrash = true;
          }

          if (AutoStepCnt == 0 && DontCrash) { // Do initial distance approach
            p_Distance.setP(0.56);
            Robot.m_Drivetrain.pid_drive(
                MathUtil.clamp(p_Distance.calculate(LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getZ(),
                    robotPosePreset[0] + 0.2), -0.6, 0.6),
                0,
                MathUtil.clamp(p_TX.calculate(-1 * LimelightHelpers.getTX(limelightName), 0), -0.4, 0.4));
            if (Math.abs(p_Distance.getError()) <= (Math.abs(p_Distance.getErrorTolerance()) * 1.5)) {
              AutoStepCnt = 1;
              System.out.println("S1 COMPLETE");
            }
          } else if (AutoStepCnt == 1) { // Do initial strife and yaw approach
            Robot.m_Drivetrain.pid_drive(
                0,
                MathUtil.clamp(p_Strife.calculate(LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getX(),
                    robotPosePreset[1]), -0.5, 0.5),
                MathUtil.clamp(p_Yaw.calculate(
                    -1 * LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getRotation().getY(),
                    robotPosePreset[2] * -1), -0.5, 0.5));
            if (Math.abs(p_Strife.getError()) <= (Math.abs(p_Strife.getErrorTolerance()) * 1.5)
                && Math.abs(p_Yaw.getError()) <= (Math.abs(p_Yaw.getErrorTolerance()) * 1.5)) {
              AutoStepCnt = 2;
              System.out.println("S2 COMPLETE");
            }
          } else if (AutoStepCnt == 2) { // Do final approach on all
            p_Distance.setP(0.75);
            Robot.m_Drivetrain.pid_drive(
                MathUtil.clamp(p_Distance.calculate(LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getZ(),
                    robotPosePreset[0]), -0.6, 0.6),
                MathUtil.clamp(p_Strife.calculate(LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getX(),
                    robotPosePreset[1]), -0.5, 0.5),
                MathUtil.clamp(p_Yaw.calculate(
                    -1 * LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getRotation().getY(),
                    robotPosePreset[2] * -1), -0.5, 0.5));
            if (Math.abs(p_Distance.getError()) <= (Math.abs(p_Strife.getErrorTolerance()))
                && Math.abs(p_Strife.getError()) <= (Math.abs(p_Strife.getErrorTolerance()))
                && Math.abs(p_Yaw.getError()) <= (Math.abs(p_Yaw.getErrorTolerance()))) {
              AutoStepCnt = 3;
              System.out.println("S3 COMPLETE");
            }
          } else { // Stop robot & Place coral
            Robot.m_Drivetrain.pid_drive(0, 0, 0);
            if (Time <= 70 && AutoStepCnt == 3) {
              Robot.wait(1);
              Time++;
              PID_Controller.m_coralIntake.set(robotPosePreset[3]);
            } else if (AutoStepCnt == 3) {
              Time = 0;
              PID_Controller.m_coralIntake.set(0);
              AutoStepCnt = 4;
              System.out.println("S4 COMPLETE");
            } else if (AutoStepCnt == 4) { // Back away 0.75m before retracting arm
              p_Distance.setP(.65);
              Robot.m_Drivetrain.pid_drive(
                  MathUtil.clamp(p_Distance.calculate(LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getZ(),
                      robotPosePreset[0] + 0.75), -0.6, 0.6),
                  0, 0);
              if (Math.abs(p_Distance.getError()) <= (Math.abs(p_Distance.getErrorTolerance()) * 2)) {
                AutoStepCnt = 5;
                System.out.println("S5 COMPLETE");
                Robot.m_Drivetrain.pid_drive(0, 0, 0);
                Robot.m_ArmPresets = Robot.kArmPos_Idle;
              }
            }
          }

          break;
        case kBack:
          // Go to specified arm pose when in range.
          if (LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getZ() <= 1.25 && AutoStepCnt == 0) {
            Robot.m_ArmPresets = ArmPose;
          }

          if (AutoStepCnt == 0) { // Do initial distance approach
            p_Distance.setP(0.53);
            Robot.m_Drivetrain.pid_drive(
                MathUtil.clamp(-1 * p_Distance
                    .calculate(LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getZ(),
                        robotPosePreset[0] + 0.15),
                    -0.6, 0.6),
                0,
                MathUtil.clamp(p_TX.calculate(-1 * LimelightHelpers.getTX(limelightName), 0), -0.4, 0.4));
            if (Math.abs(p_Distance.getError()) <= (Math.abs(p_Distance.getErrorTolerance()) * 1.6)) {
              AutoStepCnt = 1;
              System.out.println("S1 COMPLETE");
            }
          } else if (AutoStepCnt == 1) { // Do initial strife and yaw approach
            Robot.m_Drivetrain.pid_drive(
                0,
                MathUtil.clamp(p_Strife.calculate(LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getX(),
                    robotPosePreset[1]), -0.5, 0.5),
                MathUtil.clamp(p_Yaw.calculate(
                    -1 * LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getRotation().getY(),
                    -1 * robotPosePreset[2]), -0.5, 0.5));
            if (Math.abs(p_Strife.getError()) <= (Math.abs(p_Strife.getErrorTolerance()) * 1.5)
                && Math.abs(p_Yaw.getError()) <= (Math.abs(p_Yaw.getErrorTolerance()) * 1.5)) {
              AutoStepCnt = 2;
              System.out.println("S2 COMPLETE");
            }
          } else if (AutoStepCnt == 2) { // Do final approach on all
            p_Distance.setP(0.75);
            Robot.m_Drivetrain.pid_drive(
                MathUtil.clamp(-1 * p_Distance
                    .calculate(LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getZ(), robotPosePreset[0]),
                    -0.6, 0.6),
                MathUtil.clamp(p_Strife.calculate(LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getX(),
                    robotPosePreset[1]), -0.5, 0.5),
                MathUtil.clamp(p_Yaw.calculate(
                    -1 * LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getRotation().getY(),
                    -1 * robotPosePreset[2]), -0.5, 0.5));
            if (Math.abs(p_Distance.getError()) <= (Math.abs(p_Strife.getErrorTolerance()))
                && Math.abs(p_Strife.getError()) <= (Math.abs(p_Strife.getErrorTolerance()))
                && Math.abs(p_Yaw.getError()) <= (Math.abs(p_Yaw.getErrorTolerance()))) {
              AutoStepCnt = 3;
              System.out.println("S3 COMPLETE");
              Robot.m_Drivetrain.pid_drive(0, 0, 0);
            }
          } else { // Stop robot & Place coral
            Robot.m_Drivetrain.pid_drive(0, 0, 0);
            if (Time <= 90 && AutoStepCnt == 3) {
              Robot.wait(1);
              Time++;
              PID_Controller.m_coralIntake.set(robotPosePreset[3]);
            } else if (AutoStepCnt == 3) {
              Time = 0;
              PID_Controller.m_coralIntake.set(0);
              AutoStepCnt = 4;
              System.out.println("S4 COMPLETE");
            } else if (AutoStepCnt == 4) { // Back away 0.75m before retracting arm
              p_Distance.setP(.74);
              Robot.m_Drivetrain.pid_drive(
                  MathUtil.clamp(
                      -1 * p_Distance.calculate(LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getZ(),
                          robotPosePreset[0] + 0.75),
                      -0.3, 0.3),
                  0, 0);
              if (Math.abs(p_Distance.getError()) <= (Math.abs(p_Distance.getErrorTolerance()) * 2)) {
                AutoStepCnt = 5;
                System.out.println("S5 COMPLETE");
                Robot.m_Drivetrain.pid_drive(0, 0, 0);
                Robot.m_ArmPresets = Robot.kArmPos_Idle;
              }
            }
          }
          break;
      }
    } else {
      if (!DriverStation.isTeleopEnabled()) {
        Robot.m_Drivetrain.pid_drive(0, 0, 0);
      }
    }
  }
}
