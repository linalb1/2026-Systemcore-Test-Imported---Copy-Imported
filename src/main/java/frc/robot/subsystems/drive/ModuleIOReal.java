// // Copyright 2021-2025 FRC 6328
// // http://github.com/Mechanical-Advantage
// //
// // This program is free software; you can redistribute it and/or
// // modify it under the terms of the GNU General Public License
// // version 3 as published by the Free Software Foundation or
// // available in the root directory of this project.
// //
// // This program is distributed in the hope that it will be useful,
// // but WITHOUT ANY WARRANTY; without even the implied warranty of
// // MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// // GNU General Public License for more details.

// package frc.robot.subsystems.drive;

// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.PersistMode;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.SparkBase;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.filter.Debouncer;
// import edu.wpi.first.math.geometry.Rotation2d;
// import java.util.Queue;

// import frc.robot.Configs;
// import frc.robot.util.SparkUtil;
// import static frc.robot.util.SparkUtil.*;
// import static frc.robot.Constants.DriveConstants.*;

// /**
//  * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller, and duty cycle
//  * absolute encoder.
//  */
// public class ModuleIOReal implements ModuleIO {
//     private final Rotation2d zeroRotation;

//     // Hardware objects
//     private final SparkBase driveSpark;
//     private final SparkBase turnSpark;
//     private final RelativeEncoder driveEncoder;
//     private final AbsoluteEncoder turnEncoder;

//     // Closed loop controllers
//     private final SparkClosedLoopController driveController;
//     private final SparkClosedLoopController turnController;

//     // Queue inputs from odometry thread
//     private final Queue<Double> timestampQueue;
//     private final Queue<Double> drivePositionQueue;
//     private final Queue<Double> turnPositionQueue;

//     // Connection debouncers
//     private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
//     private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

//     public ModuleIOReal(int module) {
//         zeroRotation = switch (module) {
//             case 0 -> new Rotation2d();
//             case 1 -> new Rotation2d();
//             case 2 -> new Rotation2d();
//             case 3 -> new Rotation2d();
//             default -> new Rotation2d();};
//         driveSpark = new SparkFlex(
//                 switch (module) {
//                     case 0 -> frontLeftDriveCanId;
//                     case 1 -> frontRightDriveCanId;
//                     case 2 -> backLeftDriveCanId;
//                     case 3 -> backRightDriveCanId;
//                     default -> 0;
//                 },
//                 MotorType.kBrushless);
//         turnSpark = new SparkFlex(
//                 switch (module) {
//                     case 0 -> frontLeftTurnCanId;
//                     case 1 -> frontRightTurnCanId;
//                     case 2 -> backLeftTurnCanId;
//                     case 3 -> backRightTurnCanId;
//                     default -> 0;
//                 },
//                 MotorType.kBrushless);
//         driveEncoder = driveSpark.getEncoder();
//         turnEncoder = turnSpark.getAbsoluteEncoder();
//         driveController = driveSpark.getClosedLoopController();
//         turnController = turnSpark.getClosedLoopController();

//         // Configure drive motor
//         tryUntilOk(
//                 driveSpark,
//                 5,
//                 () -> driveSpark.configure(
//                         Configs.driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
//         tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

//         // Configure turn motor
//         tryUntilOk(
//                 turnSpark,
//                 5,
//                 () -> turnSpark.configure(Configs.turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

//         // Create odometry queues
//         timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
//         drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(() -> driveEncoder.getPosition());
//         turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSpark, () -> turnEncoder.getPosition());
//     }

//     @Override
//     public void updateInputs(ModuleIOInputs inputs) {
//         // Update drive inputs
//         SparkUtil.sparkStickyFault = false;
//         SparkUtil.ifOk(driveSpark, () -> driveEncoder.getPosition(), pos -> inputs.drivePosition = pos);
//         SparkUtil.ifOk(driveSpark, () -> driveEncoder.getVelocity(), vel -> inputs.driveVelocity = vel);
    
//         SparkUtil.ifOk(driveSpark, () -> driveSpark.getAppliedOutput() * driveSpark.getBusVoltage(), volts -> inputs.driveAppliedVolts = volts);
//         SparkUtil.ifOk(driveSpark, () -> driveSpark.getOutputCurrent(), amps -> inputs.driveCurrentAmps = amps);
//         SparkUtil.ifOk(driveSpark, () -> driveSpark.getMotorTemperature(), temp -> inputs.driveTemperature = temp);

//         inputs.driveConnected = driveConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);
//         SparkUtil.sparkStickyFault = false;

//     // Turn Motor Inputs
//         SparkUtil.ifOk(turnSpark, () -> turnEncoder.getPosition(), pos -> inputs.turnPosition = pos);
//         SparkUtil.ifOk(turnSpark, () -> turnEncoder.getVelocity(), vel -> inputs.turnAngularVelocity = vel);
    
//         SparkUtil.ifOk(turnSpark, () -> turnSpark.getAppliedOutput() * turnSpark.getBusVoltage(), volts -> inputs.turnAppliedVolts = volts);
//         SparkUtil.ifOk(turnSpark, () -> turnSpark.getOutputCurrent(), amps -> inputs.turnCurrentAmps = amps);
//         SparkUtil.ifOk(turnSpark, () -> turnSpark.getMotorTemperature(), temp -> inputs.turnTemperature = temp);

//         inputs.turnConnected = turnConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);

//         // Update odometry inputs
//         inputs.odometryTimestamps =
//                 timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
//         inputs.odometryDrivePositionsRad =
//                 drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
//         inputs.odometryTurnPositions = turnPositionQueue.stream()
//                 .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
//                 .toArray(Rotation2d[]::new);
//         timestampQueue.clear();
//         drivePositionQueue.clear();
//         turnPositionQueue.clear();
//     }

//     @Override
//     public void setDriveOpenLoop(double output) {
//         driveSpark.setVoltage(output);
//     }

//     @Override
//     public void setTurnOpenLoop(double output) {
//       turnSpark.setVoltage(output);
//     }

//     @Override
//     public void setDriveVelocity(double velocityRadPerSec) {
//         double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
//         driveController.setSetpoint(
//           velocityRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts, ArbFFUnits.kVoltage);
//     }

//     @Override
//     public void setTurnPosition(Rotation2d rotation) {
//         double setpoint =
//                 MathUtil.inputModulus(rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput);
//         turnController.setSetpoint(setpoint, ControlType.kPosition);
//     }
// }