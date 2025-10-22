// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  
  // 2025 SparkMax Java Changes: https://docs.revrobotics.com/revlib/24-to-25#java-1
  // Example code: https://github.com/REVrobotics/REVLib-Examples/blob/main/Java/SPARK/Open%20Loop%20Arcade%20Drive/src/main/java/frc/robot/Robot.java
  private SparkMax leftLeader;
  private SparkMax leftFollower;
  private SparkMax rightLeader;
  private SparkMax rightFollower;
  

  public DrivetrainSubsystem() {

    leftLeader = new SparkMax(Constants.DriveConstants.frontleftDriveMotorPort, MotorType.kBrushed);
    leftFollower = new SparkMax(Constants.DriveConstants.backLeftDriveMotorPort, MotorType.kBrushed);
    rightLeader = new SparkMax(Constants.DriveConstants.frontrightDriveMotorPort, MotorType.kBrushed);
    rightFollower = new SparkMax(Constants.DriveConstants.backrightDriveMotorPort, MotorType.kBrushed);
    
    SparkMaxConfig globalDriveConfig = new SparkMaxConfig();
    SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

    // Create the following global parameters (will apply to all drive motors):
    //    -- Limit current to 50 AMPS
    //    -- Set idle mode to BRAKE */
    globalDriveConfig
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);

    leftLeaderConfig
      .apply(globalDriveConfig)
      .inverted(true);

    leftFollowerConfig
      .apply(globalDriveConfig)
      .follow(leftLeader);

    rightLeaderConfig
      .apply(globalDriveConfig)
      .inverted(false);

    rightFollowerConfig
      .apply(globalDriveConfig)
      .follow(rightLeader);

    // Apply configuration settings to the motors
    leftLeader.configure(leftLeaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(rightLeaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double forward, double rotation){
    leftLeader.set(forward + rotation);
    rightLeader.set(forward - rotation);
  }

  public void stop(){
    leftLeader.stopMotor();
    rightLeader.stopMotor();
  }

}
