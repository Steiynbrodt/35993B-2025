Security Policy
Supported Versions

Even though security risk is minimal for VEX V5 robots, the following versions are monitored for stability and unintended behavior:

Version	Supported
5.1.x	:white_check_mark:
5.0.x	:x:
4.0.x	:white_check_mark:
< 4.0	:x:
Security Context

This project runs on a VEX V5 Brain, which:

Has no internet connection

Does not run arbitrary code from external sources

Accepts only compiled binaries from VEXcode

Communicates only with:

Motors

Sensors

VEX controller (radio link)

USB (for programming)

This means the actual attack surface is extremely small, and “security vulnerabilities” usually refer to:

Unsafe robot behavior

Unexpected motor activation

Incorrect autonomous logic

Sensor misconfiguration

File corruption issues in VEXcode

Behavior that could physically harm the robot or operators

Reporting a Vulnerability

If you find unsafe behavior, unintended motion, or any code that could pose a risk to the robot or team members, please report it.

How to Report

You may report potential issues by:

Email:
Send a message describing the unsafe behavior and how to reproduce it.

GitHub Issues:
If the issue is not sensitive or safety-critical, you can open an Issue labeled safety.

GitHub Security Advisory:
Use this only if you believe the issue relates to a deeper structural flaw (rare, but supported).

What to Include

Please provide:

What happened

Steps to reproduce

Hardware involved

Expected vs actual behavior

Whether the issue happened in auton, driver, or both

Response Expectations

You can expect:

Acknowledgement within 48–72 hours

Weekly updates until the issue is resolved

Clear communication on whether the issue:

Was accepted

Requires more info

Was not reproducible

Has been fixed

Responsible Disclosure

Because this is an educational robotics project with very limited external risk:

You may privately report safety issues at any time

Please avoid sharing untested unsafe or hazardous behavior publicly until it’s reviewed

All fixes will be published in normal public commits
