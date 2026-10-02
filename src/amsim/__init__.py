"""Aerial Manipulator Simulation Platform."""
__version__ = "0.1.0"

__all__ = ["ScenarioContext"]


def __getattr__(name):
    if name == "ScenarioContext":
        from .api import ScenarioContext
        return ScenarioContext
    raise AttributeError(name)
