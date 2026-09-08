# Architecture

A C4 description of motionkit at three zoom levels, followed by the rules that
decide where new code goes. The diagrams are Mermaid, so they render on GitHub;
in the Doxygen reference they appear as source, which is the honest trade for
not adding a diagram toolchain to the build.

The audience is someone deciding whether to depend on this library, or where to
put their next change in it.

## Level 1 — Context

motionkit is a library, not a process. It computes; it never talks to hardware,
never owns a thread, and never decides when it is called. Everything with an
arrow into it is somebody else's process.

```mermaid
flowchart TB
    op["Operator<br/><i>person</i>"]
    integrator["Integrator<br/><i>person</i>"]

    subgraph cell [" "]
        runtime["<b>Motion runtime</b><br/><i>software system</i><br/>Cyclic task, typically 1 kHz.<br/>Owns the clock and the fieldbus."]
        mk["<b>motionkit</b><br/><i>this library</i><br/>Poses, frames, kinematics<br/>and trajectory profiles."]
        safety["<b>Safety supervisor</b><br/><i>software system</i><br/>Independent stop authority."]
    end

    drive["Servo drives<br/><i>hardware</i>"]

    op -->|"jogs, sets feed override"| runtime
    integrator -->|"declares frames, tools,<br/>joint geometry"| runtime
    runtime -->|"calls, synchronously,<br/>inside the cycle"| mk
    runtime -->|"setpoints"| drive
    safety -->|"stop request"| runtime
    mk -.->|"stopping distance<br/>informs the envelope"| safety
```

The dashed edge is the only claim motionkit makes about safety, and it is
deliberately weak. `StopProfile` computes how far an axis travels once a stop
begins; a supervisor may use that to size a safety envelope. motionkit is not
the stop authority and is not built to any functional-safety integrity level.
The supervisor exists precisely because a library linked into the same process
as the motion it supervises cannot be its own independent check.

## Level 2 — Containers

```mermaid
flowchart LR
    subgraph consumer ["Consumer process"]
        app["<b>Application code</b><br/><i>C++20</i>"]
    end

    subgraph dist ["motionkit distribution"]
        core["<b>motionkit_core</b><br/><i>static or shared library</i><br/>All computation."]
        warn["<b>motionkit_warnings</b><br/><i>INTERFACE target</i><br/>The warning set, exported<br/>so consumers may opt in."]
        pkg["<b>motionkitConfig.cmake</b><br/><i>package</i><br/>find_package entry point."]
    end

    app -->|"#include, links"| core
    app -.->|"optional"| warn
    pkg -->|"defines<br/>motionkit::core"| core
    pkg --> warn
```

Two containers rather than one because the warning set is a separate promise
from the code. A consumer who wants motionkit's diagnostics can link
`motionkit::warnings`; one who has their own house style is not
forced into `-Wold-style-cast` by an include. Both are exported, and CI builds
a consumer against the *installed* package rather than the build tree, because
an export that only works in-tree fails in a stranger's project and not in
ours — see [ADR-0004](adr/0004-verify-the-installed-package-in-ci.md).

## Level 3 — Components

The header dependency graph, drawn from the actual `#include` edges rather than
from intent. `types.hpp` is at the bottom and depends on nothing.

```mermaid
flowchart BT
    types["<b>types</b><br/>Scalar, Vec3, Mat3"]
    expected["<b>expected</b><br/>Expected&lt;T, E&gt;"]

    so3["<b>so3</b><br/>SO3"]
    se3["<b>se3</b><br/>SE3"]
    frame["<b>frame_graph</b><br/>FrameGraph, FrameId"]
    kin["<b>kinematics</b><br/>SerialChain, IkOptions"]
    dyn["<b>dynamics</b><br/>DynamicChain, RigidBody"]

    ms["<b>motion_state</b><br/>MotionState, MotionSample"]
    jerk["<b>detail/jerk_segments</b><br/><i>implementation detail</i>"]
    traj["<b>trajectory</b><br/>ScurveProfile, StopProfile,<br/>SynchronizedTrajectory"]

    so3 --> types
    se3 --> so3
    se3 --> types
    frame --> se3
    frame --> expected
    kin --> se3
    kin --> expected
    dyn --> kin
    dyn --> expected
    ms --> types
    jerk --> ms
    jerk --> types
    traj --> jerk
    traj --> ms
    traj --> expected

    classDef pose fill:#dbeafe,stroke:#2563eb,color:#1e3a5f
    classDef motion fill:#dcfce7,stroke:#16a34a,color:#14532d
    classDef base fill:#f1f5f9,stroke:#64748b,color:#1e293b
    class so3,se3,frame,kin,dyn pose
    class ms,jerk,traj motion
    class types,expected base
```

### The gap in the middle is the design

Two subtrees rise from `types` and **never meet**. The pose side — SO3, SE3,
FrameGraph, SerialChain, DynamicChain — answers *where*, and now *what force*.
The motion side — MotionState, ScurveProfile, StopProfile — answers *when*.
Nothing in `trajectory.hpp` includes `se3.hpp`, and nothing in `kinematics.hpp`
or `dynamics.hpp` includes `motion_state.hpp`.

`dynamics` is the interesting test of that boundary, because it is plainly
*about* time — it takes joint velocities and accelerations — and it still sits
on the pose side. It takes them as bare `span<const Scalar>` rather than as a
`MotionState`, so it never learns where those numbers came from. That is not an
oversight to be tidied: a torque calculation has no use for the profile that
produced the acceleration, and coupling it to one would mean a caller with
measured encoder rates could not use it.

That is deliberate, and it is the single most useful thing to know before
adding to this library. Trajectory planning here is over **scalar axes**: a
profile knows a start, a goal and a set of limits, and has no idea whether the
number it is moving is a joint angle, a rail position or a spindle override.
Kinematics maps joint angles to poses and has no notion of time.

The consequence is that motionkit does **not** currently plan a Cartesian
move — a straight line in space, with the joints solved along it and the joint
velocity limits respected. That is the module which joins the two subtrees, and
it is deferred (WP-12) rather than absent by oversight. It is genuinely harder
than either side alone: joint-space limits do not map to a fixed Cartesian
speed, because the Jacobian relating them changes along the path and blows up
near a singularity. Bolting it onto either subtree would leak the other's
concerns across the boundary drawn above.

Until then, a caller wanting Cartesian motion samples a path themselves and
calls `inverse()` per waypoint, which the 2.3 µs solve time makes viable inside
a 1 kHz cycle. What they do not get for free is a guarantee that joint limits
are respected *between* waypoints.

### Rules that decide where code goes

1. **Dependencies point down, never sideways within a level.** `se3` may use
   `so3`; `frame_graph` and `kinematics` are siblings and must not include one
   another. A change needing that edge is a change that wants a new module.
2. **`detail/` is not part of the promise.** It is excluded from the published
   reference. Anything a consumer may reasonably depend on belongs in a public
   header, where the documentation gate applies to it.
3. **Failures that a control loop can expect are values, not exceptions.**
   `Expected<T, E>` on the boundary; exceptions only where the caller has
   already violated a precondition, such as normalising a zero vector.
4. **Anything callable from the cyclic task allocates nothing and does not
   throw.** `lookup`, `sample`, `forward`, `jacobian` and `inverse` are all
   pinned by tests that replace global `operator new` and assert a zero delta.
