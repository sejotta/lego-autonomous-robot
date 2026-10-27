"""Core package for the LEGO autonomous robot project.

The package is split by responsibility:

- :mod:`robot.kinematics`  pure differential-drive math
- :mod:`robot.mapping`     grid map + NF1 path planning
- :mod:`robot.vision`      HSV ball tracking and ORB marker recognition
- :mod:`robot.odometry`    encoder/gyro odometry thread
- :mod:`robot.robot`       hardware-facing robot API
- :mod:`robot.mission`     four-phase mission orchestration
"""

__version__ = "1.0.0"
