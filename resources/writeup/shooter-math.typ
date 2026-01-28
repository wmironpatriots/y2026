#set page(paper: "a5")
#set heading(numbering: "1.")

#show link: set text(fill: blue, weight: 700)
#show link: underline

#import "template.typ": *

#show: kunskap.with(
    title: "Shooter Physics 2026",
    author: "Dasun Abeykoon",
)

A short writeup for our planned shooter algorithm

= Goals
- Accurate/Fast standstill shooting
  - We should be able to have decent accuracy while shooting standstill
  - We should be able to prepare standstill shots quickly
- Flywheel optimization
  - We want our flywheel to reach shooting velocity as quickly as possible while conserving as much power as possible
- Accurate Shoot-On-The-Move(Optional)
  - We want to be able perform SOTM with at least 0.5 max robot speed

#pagebreak()

= Standstill shooting
== Conditions
  We start with the following knowns:
    - Robot Displacement Wrt Hub Center ($arrow(r)$)

    To calculate the following values:
    - Fuel Velocity ($arrow(v)$)

    That have the following constraints:
    - Min and Max Angles ($theta_text("min") < v < theta_text("max")$ where $theta$ is the angle of $arrow(v)$)
    - Min and Max Flywheel Velocities ($v_text("min") < v < v_text("max")$)

    We would also *optionally* want to minimize $v$ to preserve power

== Selecting an Approach

Now we could definitely try our best to create a perfect model using some kinematics, however we could also be lazy and create a #strong(link("https://en.wikipedia.org/wiki/Lookup_table", "Lookup Table")) with many points defined from real world experimenting. 

  Of course this approach does offer more benefits than just "waaaaaaah I don't wanna think"
    - Instead of struggling to model real world imperfections, the system should be able to account for them given a large enough sample size
    - Can have higher accuracy when tuned well
    - Wayyyy more easy computationally  

  The input of our table will be the *distance between the robot and the hub center ($r$)* and the outputs will be the *fuel's velocity ($arrow(v)$)* along with the Time Of Flight ($t$).

== Defining our LUT
1. First define a maximum & minimum $r$ as well as a $Delta r$. Then, place a tape at regular intervals of $Delta r$ between $r_text("min")$ and $r_text("max")$. 

  (I think $r_text("min") = 0.5969 [m]$, $Delta r = 0.27 [m]$, $r_text("max") = 6 [m]$ should lowkey probably be good enough)

2. Since we have 3 outputs, it's going to be slightly cumbersome to figure out a $arrow(v)$ with a minimized magnitude. To mitigate this, I recommend using the Slepinr script in this folder to determine an approximation for $arrow(v)$ & $t$ at each point. You can then attempt to launch with the approximation and tune it from there to get consistency.

Here's an example of how we would define this LUT in code using WPIlib's *InterpolatingTreeMap* & Units class
```Java
/** Represents the parameters of a fuel shot */
public static record ShotParams(
  Angle shotAngle, 
  LinearVelocity shotVelocity, 
  Time timeOfFlight) {}

// Create blank LUT
public static final InterpolatingTreeMap<Distance, ShotParams> kShooterMap = new InterpolatingTreeMap<>();

static {
  // Example of a single point (these are bs values btw)
  kShooterMap.put(
    Meters.of(0.5969), 
    new ShotParams(
      Degrees.of(60.0), 
      MetersPerSecond.of(16.0), 
      Seconds.of(5.9)));
  // ...
}
```

