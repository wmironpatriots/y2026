# Superstructure
```mermaid
---
title: FRC 6423 Superstructure Finite State-Machine (2026)
---
stateDiagram-v2
    direction TB
    ***PACKAGED*** --> UNPACKING
    UNPACKING --> NZ: Kicker(Deployed) && In(NZ)
    UNPACKING --> AZ: Kicker Deployed && In(AZ)

    state NZ {
        direction TB
        ***NZ_IDLE*** --> PREPPING_FERRY: Request(Action)
        PREPPING_FERRY --> FERRYING: Drive(AIMED), HOOD(AIMED), Flywheel(CRUSING)
        PREPPING_FERRY --> ***NZ_IDLE***: !Request(Action)
        FERRYING --> ***NZ_IDLE***: !Request(Action)
        FERRYING --> PREPPING_FERRY_INTAKE: Request(Intake) && Request(Action)
        PREPPING_FERRY_INTAKE --> FERRYING_WHILE_INTAKING: Drive(AIMED), HOOD(AIMED), Flywheel(CRUSING), Intake(DEPLOYED)
        PREPPING_FERRY_INTAKE --> PREPPING_FERRY: !Request(Intake) && Request(Action)
        FERRYING_WHILE_INTAKING --> PREPPING_FERRY: !Request(Intake) && Request(Action)
    }
    ***NZ_IDLE*** --> ***AZ_IDLE***: In(AZ)
    ***NZ_IDLE*** --> INTAKING: Request(Intake)
    INTAKING --> ***NZ_IDLE***: !Request(Intake)
    INTAKING --> PREPPING_FERRY_INTAKE: Request(Intake) && Request(Action)
    PREPPING_FERRY_INTAKE --> INTAKING: Request(Intake) && !Request(Action)
    FERRYING_WHILE_INTAKING --> INTAKING: Request(Intake) && !Request(Action)

    state AZ {
        direction TB
        ***AZ_IDLE*** --> PREPPING_FIRE: Request(Action)
        PREPPING_FIRE --> FIRING: Drive(AIMED && TRAJ_ASSIST), HOOD(AIMED), Flywheel(CRUSING)
        PREPPING_FIRE --> ***AZ_IDLE***: !Request(Action)
        FIRING --> ***AZ_IDLE***: Hopper(EMPTY) || !Request(Action)
    }
    ***AZ_IDLE*** --> ***NZ_IDLE***: In(NZ)
    ***AZ_IDLE*** --> INTAKING: Request(Intake)
    INTAKING --> ***AZ_IDLE***: !Request(Intake)
    PREPPING_FIRE --> INTAKING: Request(Intake)
    FIRING --> INTAKING: Request(Intake)
```
## Requests
* Intake ~ Just what it sounds like
* Action
    * When in NZ, this is always just ferrying
    * When in AZ, this is always just shooting

## States
#### PACKAGED
Represents a state where kicker is folded within drivetrain

#### UNPACKING
Represents a state where intake is punching kicker to deploy it

#### INTAKING
Represents a state where intake is attempting to deploy and intake

#### NZ_IDLE
Represents a state where robot is idle in the neutral zone

## Subsystems + Their states
* In (representation of robotstate idk)
    * NZ ~ Neutral Zone
    * AZ ~ Alliance Zone
* Drive
    * AIMED ~ Heading is faced towards virtual target
    * TRAJ_ASSIST ~ Drive trajectory is rubberbanded for consistant shots
* Hood
    * AIMED ~ Pitch is at desired angle
* Flywheel
    * CRUSING ~ At desired velocity
* Hopper
    * UNKNOWN ~ After the hopper starts intaking again, there's no way to know if it has balls
    * EMPTY ~ If shooter beambreak stops detecting, it's assumed that there are no more balls
#### PREPPING_FERRY
Represents a state where robot preparing/aiming to ferry

#### FERRYING
Represents a state where robot is feeding balls to alliance zone

#### PREPPING_FERRY_INTAKE
Represents a state where robot is preparing/aiming to ferry while intaking

#### FERRYING_WHILE_INTAKING
Represents a state where robot is feeding balls to alliance zone while intaking

#### ***AZ_IDLE***
Represents a state where robot is idle in the neutral zone

#### PREPPING_FIRE
Represents a state where robot is preparing/aiming to shoot

#### FIRING
Represents a state where robot is shooting
