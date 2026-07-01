"""Abstract base class that defines the ``periodic()`` contract for all hardware subsystems."""

from abc import ABC, abstractmethod


class Subsystem(ABC):
    """Interface that every hardware driver implements.

    Subclasses provide a ``periodic()`` method called once per robot loop
    cycle (20 ms) to update internal state and push outputs.
    """

    @abstractmethod
    def periodic(self) -> None:
        """Advance the subsystem by one cycle.

        Called every robot loop iteration.  Implementations should read
        sensors, compute state, and write actuators as needed.
        """
        ...
