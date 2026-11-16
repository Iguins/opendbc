"""
Stop assist: with the Fisker's own ACC (the "Hydra") driving, slow for a stop sunnypilot's driving model predicts.

The Hydra's ACC brakes and accelerates through ADAS_0x121 alone. While a stop is predicted, the carcontroller
sends signed 0x121 candidates carrying the acceleration from StopAssist.update, and panda puts one in place of the
Hydra's own frame only when it asks for less (fisker_stop_assist_modify in safety/modes/fisker.h). The request
starts at the Hydra's own, goes down toward the model's at STOP_ASSIST_JERK_DOWN and, once the model no longer
predicts the stop, back up at STOP_ASSIST_JERK_UP until it reaches the Hydra's again, where it stops.

Below STOP_ASSIST_HANDOFF_SPEED with the stop still predicted, the driver is asked to brake to finish the stop:
the Hydra has never stopped and held the car on its own here. From then on the request never goes back up by
itself, so the car doesn't move off on the model's word; braking, the gas pedal or cancelling ACC ends it.
"""
import math

from opendbc.car import DT_CTRL, structs
from opendbc.car.fisker.values import CarControllerParams

StopAssistState = structs.CarStateSP.StopAssistState


class StopAssist:
  def __init__(self):
    self.accel: float | None = None   # what we ask for while active, None when idle
    self.handed_off = False

  def reset(self) -> None:
    self.accel = None
    self.handed_off = False

  @property
  def state(self) -> StopAssistState:
    if self.accel is None:
      return StopAssistState.off
    return StopAssistState.brakeToStop if self.handed_off else StopAssistState.slowing

  def update(self, allowed: bool, stop_predicted: bool, model_accel: float, hydra_accel: float, v_ego: float) -> float | None:
    """The acceleration to ask the VCU for this frame, or None to leave the Hydra's own request alone.
    allowed: sunnypilot is engaged, the Hydra's ACC is Active with a valid request and the driver is off the pedals."""
    P = CarControllerParams
    if not allowed:
      self.reset()
      return None

    if self.accel is None:
      if not (stop_predicted and P.STOP_ASSIST_MIN_SPEED <= v_ego < P.STOP_ASSIST_MAX_SPEED):
        return None
      self.accel = hydra_accel

    # only while the stop is still predicted: once handing back, slowing below it mustn't freeze the request
    if stop_predicted and v_ego < P.STOP_ASSIST_HANDOFF_SPEED:
      self.handed_off = True

    target = min(max(model_accel, P.STOP_ASSIST_ACCEL_MIN), 0.0)
    if self.handed_off:
      goal = min(target, self.accel) if stop_predicted else self.accel
    elif stop_predicted:
      goal = target
    else:
      goal = math.inf
    self.accel = min(max(goal, self.accel - P.STOP_ASSIST_JERK_DOWN * DT_CTRL), self.accel + P.STOP_ASSIST_JERK_UP * DT_CTRL)

    if not stop_predicted and not self.handed_off and self.accel >= hydra_accel:
      self.reset()
      return None
    return self.accel
