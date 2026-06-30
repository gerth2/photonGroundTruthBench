"""Lightweight Subsystem base — conceptual replacement for commands2.Subsystem.

Subclass this for any hardware abstraction that needs a periodic update
called once per robot loop.  No WPILib dependency.
"""


class Subsystem:
    def periodic(self) -> None:
        """Called every robot loop by the Robot base class."""
        pass

    def getName(self) -> str:
        return type(self).__name__
