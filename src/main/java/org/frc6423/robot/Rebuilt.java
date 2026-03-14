// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Ellipse2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import org.frc6423.lib.util.GeometryUtil;

/**
 * Game specific constants for the 2026 FRC Game: Rebuilt
 *
 * <p>The term "robot alliance" or just "alliance" is used to describe the alliance the robot is
 * currently on
 *
 * <p>The term "opposing alliance" is used to describe the alliance the robot isn't currently on
 *
 * <p>All measurements are relative to the origin, which is defined as the far right corner of the
 * robot alliance from the view of the driver
 *
 * <p>The +x/length direction is defined as the forward direction from the view of the driver
 *
 * <p>The +y/width direction is defined as the left direction from the view of the driver
 *
 * <p>Alliance specific {@link Pose2d} constants that have no alliance indicator in name should be
 * assumed to be of the robot alliance TODO cleanup
 */
public class Rebuilt {
  /** {@link Alliance} The alliance robot is on */
  public static final Alliance kAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);

  // * FIELD
  /** {@link AprilTagFieldLayout} The april tag layout */
  public static final AprilTagFieldLayout kAprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  /** {@link Distance} The length of the field */
  public static final Distance kFieldLength = Inches.of(651.22);

  /** {@link Distance} The width of the field */
  public static final Distance kFieldWidth = Inches.of(317.69);

  /** {@link Distance} Length of Alliance Zone */
  public static final Distance kAllianceZoneLength = Inches.of(156.61 + 2.0);

  /** {@link Distance} Length of Trench Zone */
  public static final Distance kTrenchZoneLength = Inches.of(47.00);

  /** {@link Distance} Length of Neutral Zone */
  public static final Distance kNeutralZoneLength = Inches.of(120.00 * 2);

  /** {@link Pose2d} Midpoint of field */
  public static final Pose2d kMidPose =
      new Pose2d(kFieldLength.div(2), kFieldWidth.div(2), new Rotation2d());

  /** {@link Rectangle2d} Field */
  public static final Rectangle2d kField = new Rectangle2d(kMidPose, kFieldLength, kFieldWidth);

  /** {@link Rectangle2d} Robot Alliance Zone */
  public static final Rectangle2d kRobotAllianceZone =
      new Rectangle2d(new Translation2d(), new Translation2d(kAllianceZoneLength, kFieldWidth));

  /** {@link Rectangle2d} Trench Zone */
  public static final Rectangle2d kTrenchZone =
      new Rectangle2d(
          new Translation2d(kAllianceZoneLength, Inches.zero()),
          new Translation2d(kAllianceZoneLength.plus(kTrenchZoneLength), kFieldWidth));

  /** {@link Rectangle2d} Neutral Zone */
  public static final Rectangle2d kNeutralZone =
      new Rectangle2d(
          new Translation2d(kAllianceZoneLength.plus(kTrenchZoneLength), Inches.zero()),
          new Translation2d(
              kAllianceZoneLength.plus(kTrenchZoneLength).plus(kNeutralZoneLength), kFieldWidth));

  /** {@link Rectangle2d} Opposing Trench Zone */
  public static final Rectangle2d kOpposingTrench =
      new Rectangle2d(
          GeometryUtil.allianceFlipPose2d(kMidPose, kTrenchZone.getCenter()),
          kTrenchZoneLength,
          kFieldWidth);

  /** {@link Rectangle2d} Opposing Alliance Zone */
  public static final Rectangle2d kOpposingAlliance =
      new Rectangle2d(
          GeometryUtil.allianceFlipPose2d(kMidPose, kRobotAllianceZone.getCenter()),
          kAllianceZoneLength,
          kFieldWidth);

  // * DEPOT
  /** {@link Distance} representing the length of the depot */
  public static final Distance kDepotLength = Inches.of(27.00);

  /** {@link Distance} representing the width of the depot */
  public static final Distance kDepotWidth = Inches.of(42.00);

  /** {@link Pose2d} representing the location of the robot alliance depot */
  public static final Pose2d kDepotPose2d =
      new Pose2d(kDepotLength.div(2), kFieldWidth.div(2).plus(Inches.of(75.93)), Rotation2d.kZero);

  /** {@link Pose2d} representing the location of the opposing alliance depot */
  public static final Pose2d kOpposingAllianceDepotPose2d = allianceFlipPose2d(kDepotPose2d);

  /** {@link Pose2d} representing the area of the robot alliance depot */
  public static final Rectangle2d kDepot = new Rectangle2d(kDepotPose2d, kDepotLength, kDepotWidth);

  /** {@link Pose2d} representing the area of the opposing alliance depot */
  public static final Rectangle2d kOpposingAllianceDepot =
      new Rectangle2d(kOpposingAllianceDepotPose2d, kDepotLength, kDepotWidth);

  // * HUB
  /** {@link Distance} representing the length of the hub side */
  public static final Distance kHubSideLength = Inches.of(47.00);

  /** {@link Distance} representing the effective diameter of the hub opening */
  public static final Distance kHubEffectiveOpeningDiameter = Inches.of(41.73);

  /** {@link Pose2d} representing the location of the robot alliance hub */
  public static final Pose2d kHubPose2d =
      new Pose2d(Inches.of(182.11), Inches.of(158.84), Rotation2d.kZero);

  /** {@link Pose2d} representing the location of the opposing alliance hub */
  public static final Pose2d kOpposingAllianceHubPose2d = allianceFlipPose2d(kHubPose2d);

  public static final Distance kHubHeight = Inches.of(72.0);

  /** {@link Rectangle2d} representing the area of the robot alliance hub */
  public static final Rectangle2d kHub =
      new Rectangle2d(kHubPose2d, kHubSideLength, kHubSideLength);

  /** {@link Rectangle2d} representing the area of the opposing alliance hub */
  public static final Rectangle2d kOpposingAllianceHub =
      new Rectangle2d(kHubPose2d, kHubSideLength, kHubSideLength);

  /**
   * {@link Ellipse2d} representing the effective area of the robot alliance hub opening
   *
   * <p><strong>WARNING</strong>: this isn't the actual area of the hub opening, it's only the area
   * of a circle tightly containing the area of the opening
   */
  public static final Ellipse2d kHubEffectiveOpening =
      new Ellipse2d(kHubPose2d, kHubEffectiveOpeningDiameter, kHubEffectiveOpeningDiameter);

  /**
   * {@link Ellipse2d} representing the effective area of the opposing alliance hub opening
   *
   * <p><strong>WARNING</strong>: this isn't the actual area of the hub opening, it's only the area
   * of a circle tightly containing the area of the opening
   */
  public static final Ellipse2d kOpposingAllianceHubOpening =
      new Ellipse2d(
          kOpposingAllianceHubPose2d, kHubEffectiveOpeningDiameter, kHubEffectiveOpeningDiameter);

  // * NEUTRAL ZONE
  public static final Distance kNeutralMassLength = Inches.of(71.9);

  public static final Distance kNeutralMassWidth = Inches.of(181.9);

  public static final Rectangle2d kNeutralMass =
      new Rectangle2d(kMidPose, kNeutralMassLength, kNeutralMassWidth);

  public static Pose2d allianceFlipPose2d(Pose2d pose) {
    return GeometryUtil.allianceFlipPose2d(kMidPose, pose);
  }

  public static Pose2d mirrorPose2d(Pose2d pose) {
    return GeometryUtil.mirrorPose2d(kMidPose, pose);
  }

  // * MATCH TIME
  /** {@link Time} representing the total length of a match */
  public static final Time kMatchLength = Seconds.of(2 * 60 + 20);

  /** {@link Time} representing the timestamp when shift zero ends */
  public static final Time kEndOfShiftZero = Seconds.of(2 * 60 + 10);

  /** {@link Time} representing the timestamp when shift one ends */
  public static final Time kEndOfShiftOne = Seconds.of(60 + 45);

  /** {@link Time} representing the timestamp when shift two ends */
  public static final Time kEndOfShiftTwo = Seconds.of(60 + 20);

  /** {@link Time} representing the timestamp when shift three ends */
  public static final Time kEndOfShiftThree = Seconds.of(55);

  /** {@link Time} representing the timestamp when shift four ends */
  public static final Time kEndOfShiftFour = Seconds.of(30);

  /**
   * @return true if robot alliance hub is active
   */
  public static boolean isHubActive() {
    var staringAlliance = getFirstInactiveAlliance();
    var alliance = DriverStation.getAlliance();
    var timestamp = Seconds.of(Timer.getFPGATimestamp());

    if (alliance.isEmpty()) {
      alliance = Optional.of(Alliance.Red);
    }

    if (staringAlliance.isEmpty()
        || timestamp.gt(kEndOfShiftZero)
        || timestamp.lt(kEndOfShiftFour)) {
      return true;
    } else if ((timestamp.lte(kEndOfShiftZero) && timestamp.gt(kEndOfShiftOne))
        || (timestamp.lte(kEndOfShiftTwo) && timestamp.gt(kEndOfShiftThree))) {
      return staringAlliance.get() != alliance.get();
    } else {
      return staringAlliance.get() == alliance.get();
    }
  }

  /**
   * If autonomous scores have not been processed, this method will return an empty {@link Optional}
   *
   * @return {@link Optional} of {@link Alliance} representing the starting inactive alliance
   */
  public static Optional<Alliance> getFirstInactiveAlliance() {
    String gameData = DriverStation.getGameSpecificMessage();

    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'R':
          return Optional.of(Alliance.Red);
        case 'B':
          return Optional.of(Alliance.Blue);
        default:
          return Optional.empty();
      }
    } else return Optional.empty();
  }
}
