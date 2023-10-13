from abc import ABC
import pygame
import weakref

from typing import List


class VisualizationBase(ABC):
    """Abstract base class for all visualization objects.

    Any class that inherits from VisualizationBase must call super().__init__()
    if it overrides __init__(). This is crucial because it adds the instance
    to a self-managing list of all active VisualizationBase instances.

    Attributes
    ----------
    __instances (List[weakref.ref]): A class level attribute that
        holds weak references to all instances of VisualizationBase or its child classes.
        This allows instances to be automatically removed from the list if there are no
        other references to them, allowing them to be garbage collected.

    """

    __instances: List[weakref.ref] = []
    is_visible = False

    def __init__(self, z_axis: float = 1.0) -> None:
        """
        Initializes a new instance of VisualizationBase and adds it to the list of instances.

        :param z_axis: The z-axis value used for sorting instances in the get_instances() method. Defaults to 1.0.
        :type z_axis: float
        """
        super().__init__()
        self._z_axis = z_axis
        self.__instances.append(weakref.ref(self))

    @classmethod
    def get_instances(cls) -> List['VisualizationBase']:
        """
        Retrieves all instances of VisualizationBase child classes.

        This class method allows access to all active instances of VisualizationBase
        or its child classes for operations such as batch updates or rendering.

        :return: A list containing all active instances, sorted based on their z-axis value.
        :rtype: List[VisualizationBase]
        """
        # Filter out weak references that have been garbage collected
        cls.__instances = [ref for ref in cls.__instances if ref() is not None]
        cls.__instances.sort(key=lambda ref: ref()._z_axis)  # pylint: disable=protected-access
        return [ref() for ref in cls.__instances]

    def tick(self, clock: pygame.time.Clock):
        """
        Update the state of the object for a new game tick.

        This function should be called once per frame to perform actions that
        need to occur over time, such as animation, movement, collision detection, etc.

        :param clock: The game clock to synchronize game events.
        :type clock: pygame.time.Clock
        """
        pass  # pylint: disable=unnecessary-pass

    def render(self, display: pygame.surface.Surface):
        """
        Render the object onto the game display.

        This function should be called once per frame, after all tick functions
        have been called, to draw the updated state of the object.

        :param display: The game display to render the object onto.
        :type display: pygame.display
        """
        pass  # pylint: disable=unnecessary-pass

    def restart(self):
        """
        Reset the object to its initial state.

        This function should be called when the game or level is restarted,
        to clear any accumulated state.
        """
        pass  # pylint: disable=unnecessary-pass

    def destroy(self):
        """
        Free any resources that the object is using.

        This function should be called when the object is no longer needed,
        to allow Python's garbage collector to reclaim its memory.
        """
        VisualizationBase.__instances = [ref for ref in VisualizationBase.__instances if
                                         ref() is not None and ref() != self]

    def __del__(self):
        """
        Destructor method for VisualizationBase objects.

        This method is automatically called by Python's garbage collector
        when there are no more references to the object and it is being
        garbage collected. It performs cleanup actions and frees any resources
        that the object is using.

        Note:
            The `__del__()` method is not guaranteed to be called for every
            instance of the class. Its execution depends on the garbage
            collection mechanism of Python.

        Important:
            Avoid relying on the `__del__()` method for critical cleanup tasks.
            It is recommended to use explicit `destroy()` or `cleanup()` methods
            to ensure proper cleanup of resources.

        """
        print(f"VisualizationBase object {self.__class__.__name__} destroyed")
