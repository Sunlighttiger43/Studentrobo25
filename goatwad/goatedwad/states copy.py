import logging
from abc import ABC, abstractmethod
from typing import Optional
from olaves_robot import OlavesRobot
import math

# Creating a logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

console = logging.StreamHandler()
console.setLevel(logging.DEBUG)
logger.addHandler(console)


class State(ABC):
    def __init__(self, robot: OlavesRobot):
        self.robot = robot

    @abstractmethod
    def run(self) -> Optional["State"]:
        pass


class StateMachine:
    def __init__(self, initial_state: State):
        self.current_state = initial_state

    def run(self):
        while self.current_state is not None:
            logger.info(f"Running state: {type(self.current_state).__name__}")
            self.current_state = self.current_state.run()
        logger.info("State machine execution completed.")


class StartState(State):
    def __init__(self, robot: OlavesRobot):
        super().__init__(robot)
        self.pallets_collected = 0

    def run(self) -> Optional[State]:
        logger.info("[StartState] Starting the match with pallets_collected=0.")
        return MainCycle(self.robot, pallets_collected=0)


class MainCycle(State):
    """
    Picks up 3 pallets (or however many remain up to 6),
    places them on the high-rise, then returns to corner.
    Loops until we've placed 6.
    """

    def __init__(self, robot: OlavesRobot, pallets_collected: int = 0):
        super().__init__(robot)
        self.pallets_collected = pallets_collected

    def run(self) -> Optional[State]:
        logger.info(f"[MainCycle] Pallets placed so far = {self.pallets_collected}")

        if self.pallets_collected >= 6:
            logger.info("[MainCycle] All 6 boxes placed. Transitioning to Done.")
            return DoneState(self.robot)

        # Decide how many we need in this trip:
        needed = min(2, 6 - self.pallets_collected)
        boxes_collected = self._collect_pallets(needed)

        if boxes_collected == 0:
            logger.warning("[MainCycle] Could not collect any pallets this cycle!")

        self._place_pallets()

        self.robot.power((-0.5, -0.5), 0.5)

        # self._return_to_corner()

        self.pallets_collected += boxes_collected
        logger.info(f"[MainCycle] Total placed so far: {self.pallets_collected}")

        if self.pallets_collected < 6:
            return MainCycle(self.robot, self.pallets_collected)
        else:
            return DoneState(self.robot)

    def _collect_pallets(self, needed: int) -> int:
        logger.info(
            f"[MainCycle._collect_pallets] Attempting to pick up {needed} pallets."
        )
        boxes_collected = 0

        while boxes_collected < needed and self.robot.get_comp_time() < 120:
            chosen_marker = self._search_for_pallet()
            if not chosen_marker:
                logger.warning("[_collect_pallets] No pallet found after search.")
                break

            marker_id = chosen_marker.id
            logger.info(
                f"[_collect_pallets] Moving to marker ID={marker_id} for pickup."
            )
            self.robot.move_to_marker(marker_id)

            self.robot._move(0.1)
            self.robot.sleep(0.1)
            self.robot._move(0.1)

            self.robot.pick_hold_box()
            boxes_collected += 1
            logger.info(
                f"Picked 1 pallet. So far collected this batch={boxes_collected}."
            )

        if not self.robot.get_comp_time() < 120:
            self.robot.drop_box()
        return boxes_collected

    def _search_for_pallet(self) -> Optional[object]:
        """
        Repeatedly looks for a pallet marker from our zone.
        - If found immediately, returns the closest marker
        - Otherwise, does small turns, each time waiting, re-checking camera
        - Returns None if none found after a full search

        You can adapt this to do a full 360 search or partial.
        """
        # Try once initially without turning
        markers = self.robot.camera.see()
        pallet_markers = [m for m in markers if m.id in self.robot.zone_pallets()]
        if pallet_markers:
            pallet_markers.sort(key=lambda x: x.position.distance)
            return pallet_markers[0]

        logger.info(
            "[_search_for_pallet] No pallet in initial view. Starting search turns..."
        )
        total_turned = 0.0
        turn_increment = math.pi / 64.0
        max_turn = 8 * math.pi

        while total_turned < max_turn:
            logger.debug("[_search_for_pallet] Still no pallet in view")
            self.robot.power((-0.5, 0.5), 0.15)
            total_turned += turn_increment

            self.robot.sleep(0.5)

            markers = self.robot.camera.see()
            logger.debug(
                f"[_search_for_pallets] Markers seen: {[marker.id for marker in markers]}"
            )
            pallet_markers = [m for m in markers if m.id in self.robot.zone_pallets()]
            if pallet_markers:
                pallet_markers.sort(key=lambda x: x.position.distance)
                return pallet_markers[0]

        logger.warning("[_search_for_pallet] No pallet found after full 360 turn.")
        return None

    def _place_pallets(self):
        logger.info("[_place_pallets] Placing boxes on the high-rise.")
        markers = self.robot.camera.see()
        hi_rise_marker = [m for m in markers if m.id in range(195, 200)]
        if hi_rise_marker:
            hi_rise_marker.sort(key=lambda x: x.position.distance)

        logger.info(
            "[_place_pallets] No pallet in initial view. Starting search turns..."
        )
        total_turned = 0.0
        turn_increment = math.pi / 64.0
        max_turn = 16 * math.pi

        while total_turned < max_turn:
            if total_turned % 5 / 2 * math.pi == 0:
                self.robot._move(0.3)
            logger.debug("[_place_pallets] Still no pallet in view")
            self.robot.power((-0.5, 0.5), 0.15)
            total_turned += turn_increment

            self.robot.sleep(0.5)

            markers = self.robot.camera.see()
            logger.debug(
                f"[_place_palletss] Markers seen: {[marker.id for marker in markers]}"
            )
            hi_rise_marker = [m for m in markers if m.id in self.robot.zone_pallets()]
            if hi_rise_marker:
                hi_rise_marker.sort(key=lambda x: x.position.distance)
                break

        self.robot.move_to_marker(hi_rise_marker[0])
        self.robot._move(0.3)
        self.robot.drop_box()

    def _return_to_corner(self):
        logger.info("[_return_to_corner] Returning to corner base marker.")
        base_marker_id = self.robot.zone_base()
        base_marker = self.robot.get_marker_by_id(base_marker_id)
        if base_marker:
            self.robot.move_to_marker(base_marker_id)
        else:
            logger.warning(
                "[_return_to_corner] Couldn't see base marker on first try, scanning."
            )
            found = self.robot.get_marker_by_idrange([base_marker_id])
            if found:
                self.robot.move_to_marker(base_marker_id)
            else:
                logger.error(
                    "[_return_to_corner] Base marker not found after scanning. Proceeding anyway."
                )


class DoneState(State):
    def run(self) -> Optional[State]:
        logger.info("[DoneState] All done, stopping.")
        return None


def main():
    robot = OlavesRobot()
    sm = StateMachine(initial_state=StartState(robot))
    sm.run()
