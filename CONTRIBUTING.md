Contributing Guidelines

Thank you for considering a contribution to this project!
This repository contains competition code for the 35993B VEX V5 robot, and our goal is to keep it reliable, understandable, and safe to use on real hardware.

These guidelines explain how to contribute code, documentation, or ideas in a clean and consistent way.

🧭 1. General Rules

Be respectful, collaborative, and constructive.

Follow the project’s Code of Conduct.

All changes must be understood by the contributor and reviewed by at least one teammate.

Keep code safe — remember the robot moves in the real world.

🛠 2. Contribution Workflow
1. Fork or branch the project

You can work on:

A new branch in this repo, or

Your own fork (recommended for larger changes).

2. Make your changes

This may include:

Autonomous routines

Drive/control improvements

Localization adjustments

Documentation

Testing utilities

3. Follow the coding expectations

See Section 3 below.

4. Test your changes

All robot-affecting changes must be tested:

First in simulation (if applicable)

Then on the robot in a safe environment

With someone supervising

5. Submit a Pull Request

Make a PR with:

Clear explanation of changes

Any risks or testing notes

Video evidence for significant movement changes (recommended)

6. Review Process

A teammate reviews your PR.
They may:

Request clarification

Ask for cleanup

Suggest improvements

Approve it

Once approved, the PR will be merged.

🧩 3. Coding Expectations
✔ Keep code readable

Prefer meaningful names for functions and variables.

Add comments where logic is not obvious.

Keep auton code clean and linear.

✔ Use consistent structure

Movement code → drive.hpp

Localization code → localization.hpp

Autonomous routines → auton.hpp and autonomus.hpp

✔ Document “why,” not only “how”

If something is tuned, hacked, or adjusted, include why it works.

✔ Mechanisms

Motor naming and port configuration must match vex.h.

✔ Follow the safe-operation guidelines

Never push untested or unsafe movement logic.

🎵 4. About “Vibe Coding”

We allow rapid, intuitive prototyping (“vibe coding”), as long as:

The author understands what they wrote

The code is explainable

The code is reviewed

The final version is cleaned up before merging

Unclear or unexplained code will not be accepted.

🤖 5. Autonomous Code Guidelines

Autonomous logic belongs in:

auton.hpp

autonomus.hpp (optional helper routines)

Auton must be:

Predictable

Readable

Safe

Tuned based on field tests

Follow this general pattern:

setPose_mm(startX, startY, startHeading);

turnToHeading(angle);
driveForward(distance);

intake.spin(fwd);
wait(300, msec);
intake.stop();


Do not restructure low-level movement unless necessary.

🧪 6. Testing Requirements

Before merging:

Ensure compile success in VEXcode

Verify autonomous routines on the field

Check for accidental motor reversals

Confirm safe behavior

If your change modifies:

Turning

Distance driving

Heading correction

Localization

…it must be field-tested.

📄 7. Commit Message Style

Use descriptive commit messages:

Add left-side autonomous routine
Fix turnToHeading overshoot issue
Improve localization accuracy
Clean up driveForward control loop


Avoid vague messages like “fix stuff” or “update code.”

📜 8. Documentation Contributions

Documentation updates are always welcome:

README improvements

Inline comments

Auton descriptions

Safety instructions

Developer notes

If something confused you, improving the documentation helps everyone.

🚨 9. What Not to Do

Don’t push directly to main unless authorized

Don’t submit unsafe or untested movement code

Don’t modify vex.h manually

Don’t remove or break existing functionality without discussion

Don’t push large changes without talking to the team first

🙌 10. Thank You

Every contribution — code, documentation, ideas, testing, or bug reports — helps the robot improve.
We appreciate your effort and teamwork!

If you have questions, open an Issue or ask a team member.
