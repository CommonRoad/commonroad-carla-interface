"""Utility functions for tests."""""
from dataclasses import dataclass


@dataclass
class EdgeCase:
    """Class for edge cases in traffic light cycle creation tests."""

    def __init__(self, case, expected):
        """Initialize the edge case."""
        self.case = case
        self.expected = expected
