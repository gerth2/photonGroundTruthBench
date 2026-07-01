from abc import ABC, abstractmethod


class Subsystem(ABC):
    @abstractmethod
    def periodic(self) -> None:
        ...
