// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.game;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Ellipse2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;

/**
 * TODO description
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
 * assumed to be of the robot alliance
 */
public class Rebuilt {

  // * FIELD DIMENSIONS

  /** {@link Distance} representing the length of the field */
  public static final Distance kFieldLength = Inches.of(651.22);

  /** {@link Distance} representing the width of the field */
  public static final Distance kFieldWidth = Inches.of(317.69);

  /**
   * {@link Distance} representing the length of the alliance zones (starts at wall of alliance,
   * ends after the alliance line)
   */
  public static final Distance kAllianceZoneLength = Inches.of(158.25);

  /**
   * {@link Distance} representing the length of the neutral zone (starts and ends at the alliance
   * lines)
   */
  public static final Distance kNeutralZoneLength = Inches.of(334.00);

  /** {@link Pose2d} representing the middle of the field */
  public static final Pose2d kFieldPose2d =
      new Pose2d(kFieldLength, kFieldWidth, Rotation2d.kZero).div(2);

  /** {@link Pose2d} representing the middle of the robot alliance zone */
  public static final Pose2d kAllianceZonePose2d =
      new Pose2d(kAllianceZoneLength, kFieldWidth, Rotation2d.kZero).div(2);

  /** {@link Pose2d} representing the middle of the opposing alliance zone */
  public static final Pose2d kOpposingAllianceZonePose2d = allianceFlipPose2d(kAllianceZonePose2d);

  /** {@link Rectangle2d} representing the area of the field */
  public static final Rectangle2d kField = new Rectangle2d(kFieldPose2d, kFieldLength, kFieldWidth);

  /** {@link Rectangle2d} representing the area of the robot alliance zone */
  public static final Rectangle2d kAlliance =
      new Rectangle2d(kAllianceZonePose2d, kAllianceZoneLength, kFieldWidth);

  /** {@link Rectangle2d} representing the area of the opposing alliance zone */
  public static final Rectangle2d kOpposingAlliance =
      new Rectangle2d(kOpposingAllianceZonePose2d, kAllianceZoneLength, kFieldWidth);

  // * CLIMB JIG DIMENSIONS

  /** {@link Distance} representing the length of the climb jig */
  public static final Distance kClimbJigLength = Inches.of(40.0 + 3.510);

  /** {@link Distance} representing the width of the climb jig */
  public static final Distance kClimbJigWidth = Inches.of(39);

  /** {@link Distance} representing the height of the L1 climb bar */
  public static final Distance kClimbL1Height = Inches.of(27.00);

  /** {@link Distance} representing the height of the L2 climb bar */
  public static final Distance kClimbL2Height = Inches.of(45.00);

  /** {@link Distance} representing the height of the L3 climb bar */
  public static final Distance kClimbL3Height = Inches.of(63.00);

  /** {@link Pose2d} representing the location of the robot alliance climb jig */
  public static final Pose2d kClimbJigPose2d =
      new Pose2d(
          kClimbJigLength.div(2), kFieldWidth.div(2).minus(Inches.of(11.38)), Rotation2d.kZero);

  /** {@link Pose2d} representing the location of the of the opposing alliance climb jig */
  public static final Pose2d kOpposingAllianceClimbJigPose2d = allianceFlipPose2d(kClimbJigPose2d);

  /** {@link Rectangle2d} representing the area of the robot alliance climb jig */
  public static final Rectangle2d kClimbJig =
      new Rectangle2d(kClimbJigPose2d, kClimbJigLength, kClimbJigWidth);

  /** {@link Rectangle2d} representing the area of the opposing alliance climb jig */
  public static final Rectangle2d kOpposingAllianceClimbJig =
      new Rectangle2d(kOpposingAllianceClimbJigPose2d, kClimbJigLength, kClimbJigWidth);

  // * DEPOT DIMENSIONS

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

  // * HUB DIMENSIONS

  /** {@link Distance} representing the length of the hub side */
  public static final Distance kHubSideLength = Inches.of(47.00);

  /** {@link Distance} representing the effective diameter of the hub opening */
  public static final Distance kHubEffectiveOpeningDiameter = Inches.of(41.73);

  /** {@link Pose2d} representing the location of the robot alliance hub */
  public static final Pose2d kHubPose2d =
      new Pose2d(Inches.of(182.11), Inches.of(158.84), Rotation2d.kZero);

  /** {@link Pose2d} representing the location of the opposing alliance hub */
  public static final Pose2d kOpposingAllianceHubPose2d = allianceFlipPose2d(kHubPose2d);

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

  // * BUMP DIMENSIONS

  /** {@link Distance} representing the length of the bump (+x distance) */
  public static final Distance kBumpLength = Inches.of(48.93);

  /** {@link Distance} representing the width of the bump (+y distance) */
  public static final Distance kBumpWidth = Inches.of(73.0);

  /** {@link Pose2d} representing the location of the right bump of the robot alliance */
  public static final Pose2d kRightBumpPose2d =
      new Pose2d(
          Inches.of(182.11),
          kFieldWidth.div(2).minus(kHubSideLength.div(2).plus(kBumpWidth.div(2))),
          Rotation2d.kZero);

  /** {@link Pose2d} representing the location of the left bump of the robot alliance */
  public static final Pose2d kLeftBumpPose2d = mirrorPose2d(kRightBumpPose2d);

  /** {@link Pose2d} representing the location of the right bump of the opposing alliance */
  public static final Pose2d kOpposingAllianceRightBumpPose2d = allianceFlipPose2d(kLeftBumpPose2d);

  /** {@link Pose2d} representing the location of the left bump of the opposing alliance */
  public static final Pose2d kOpposingAllianceLeftBumpPose2d = allianceFlipPose2d(kRightBumpPose2d);

  /** {@link Rectangle2d} representing the area of the robot alliance right side bump */
  public static final Rectangle2d kRightBump =
      new Rectangle2d(kRightBumpPose2d, kBumpLength, kBumpWidth);

  /** {@link Rectangle2d} representing the area of the robot alliance left side bump */
  public static final Rectangle2d kLeftBump =
      new Rectangle2d(kLeftBumpPose2d, kBumpLength, kBumpWidth);

  /** {@link Rectangle2d} representing the area of the opposing alliance right side bump */
  public static final Rectangle2d kOpposingAllianceRightBump =
      new Rectangle2d(kOpposingAllianceRightBumpPose2d, kBumpLength, kBumpWidth);

  /** {@link Rectangle2d} representing the area of the opposing alliance left side bump */
  public static final Rectangle2d kOpposingAllianceLeftBump =
      new Rectangle2d(kOpposingAllianceLeftBumpPose2d, kBumpLength, kBumpWidth);

  // * TRENCH AREA DIMENSIONS
  /** {@link Distance} representing length of trench area */
  public static final Distance kTrenchLength = Inches.of(47.00);

  /** {@link Distance} representing average width of the two trench areas */
  public static final Distance kTrenchWidth = Inches.of(50.47);

  /** {@link Distance} representing height of trench area */
  public static final Distance kTrenchHeight = Inches.of(22.25);

  /** {@link Pose2d} representing the location of robot alliance right trench */
  public static final Pose2d kRightTrenchPose2d =
      new Pose2d(
          kFieldLength.div(2).minus(Inches.of(143.50)), kTrenchWidth.div(2), Rotation2d.kZero);

  /** {@link Pose2d} representing the location of robot alliance left trench */
  public static final Pose2d kLeftTrenchPose2d = mirrorPose2d(kRightTrenchPose2d);

  /** {@link Pose2d} representing the location of opposing alliance right trench */
  public static final Pose2d kOpposingAllianceRightTrenchPose2d =
      allianceFlipPose2d(kLeftTrenchPose2d);

  /** {@link Pose2d} representing the location of opposing alliance left trench */
  public static final Pose2d kOpposingAllianceLeftTrenchPose2d =
      allianceFlipPose2d(kRightTrenchPose2d);

  /** {@link Rectangle2d} representing the robot alliance right trench */
  public static final Rectangle2d kRightTrench =
      new Rectangle2d(kRightTrenchPose2d, kTrenchLength, kTrenchWidth);

  /** {@link Rectangle2d} representing the robot alliance left trench */
  public static final Rectangle2d kLeftTrench =
      new Rectangle2d(kLeftTrenchPose2d, kTrenchLength, kTrenchWidth);

  /** {@link Rectangle2d} representing the opposing alliance right trench */
  public static final Rectangle2d kOpposingAllianceRightTrench =
      new Rectangle2d(kOpposingAllianceRightTrenchPose2d, kTrenchLength, kTrenchWidth);

  /** {@link Rectangle2d} representing the oppposing alliance left trench */
  public static final Rectangle2d kOpposingAllianceLeftTrench =
      new Rectangle2d(kOpposingAllianceLeftTrenchPose2d, kTrenchLength, kTrenchWidth);

  // * SOURCE DIMENSIONS
  /** {@link Distance} representing length of source area */
  public static final Distance kSourceLength = Inches.of(20.00);

  /** {@link Distance} representing width of source area */
  public static final Distance kSourceWidth = Inches.of(49.84);

  /** {@link Pose2d} representing the position of the robot alliance source area */
  public static final Pose2d kSourcePose2d =
      new Pose2d(kSourceWidth.div(2), kSourceLength.div(2), Rotation2d.kZero);

  /** {@link Pose2d} representing the position of the opposing alliance source area */
  public static final Pose2d kOpposingAllianceSourcePose2d = allianceFlipPose2d(kSourcePose2d);

  /** {@link Rectangle2d} representing the robot alliance source area */
  public static final Rectangle2d kSource =
      new Rectangle2d(kSourcePose2d, kSourceLength, kSourceWidth);

  /** {@link Rectangle2d} representing the opposing alliance source area */
  public static final Rectangle2d kOpposingAllianceSource =
      new Rectangle2d(kOpposingAllianceSourcePose2d, kSourceLength, kSourceWidth);

  // TODO initial neutral mass
  /**
   * Convert specified {@link Pose2d} to its opposing alliance equivalent
   *
   * @param pose {@link Pose2d} to convert
   * @return {@link Pose2d}
   */
  public static final Pose2d allianceFlipPose2d(Pose2d pose) {
    return pose.rotateAround(kFieldPose2d.getTranslation(), Rotation2d.k180deg);
  }

  /**
   * Mirror specified {@link Pose2d} over the midline of the y axis
   *
   * @param pose {@link Pose2d} to mirror
   * @return {@link Pose2d}
   */
  public static final Pose2d mirrorPose2d(Pose2d pose) {
    Rotation2d angle = kFieldPose2d.relativeTo(pose).getTranslation().getAngle();

    // I think this should work?
    return pose.rotateAround(kFieldPose2d.getTranslation(), angle.times(2).times(-1));
  }

  // TODO redo everything under

  public static final Time MATCH_TIME = Seconds.of(2 * 60 + 20);
  public static final Time SHIFT_ZERO_END = Seconds.of(2 * 60 + 10);
  public static final Time SHIFT_ONE_END = Seconds.of(60 + 45);
  public static final Time SHIFT_TWO_END = Seconds.of(60 + 20);
  public static final Time SHIFT_THREE_END = Seconds.of(55);
  public static final Time SHIFT_FOUR_END = Seconds.of(30);

  /**
   * @return true if the alliance's hub is active
   */
  public static boolean isAllianceHubActive() {
    var staringAlliance = getStartingInactiveAlliance();
    var alliance = DriverStation.getAlliance();
    var timestamp = Seconds.of(Timer.getFPGATimestamp());

    if (alliance.isEmpty()) {
      alliance = Optional.of(Alliance.Red);
    }

    if (staringAlliance.isEmpty() || timestamp.gt(SHIFT_ZERO_END) || timestamp.lt(SHIFT_FOUR_END)) {
      return true;
    } else if ((timestamp.lte(SHIFT_ZERO_END) && timestamp.gt(SHIFT_ONE_END))
        || (timestamp.lte(SHIFT_TWO_END) && timestamp.gt(SHIFT_THREE_END))) {
      return staringAlliance.get() != alliance.get();
    } else {
      return staringAlliance.get() == alliance.get();
    }
  }

  /**
   * Returns the first inactive alliance in the match
   *
   * <p>If autonomous scores have not been processed, this method will return an empty {@link
   * Optional}
   *
   * @return {@link Optional} of {@link Alliance}
   */
  public static Optional<Alliance> getStartingInactiveAlliance() {
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
