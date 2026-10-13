#!/usr/bin/env python3
"""jvePilot courtesy alerts: lead-depart and green-light chimes.

Advisory only: these state machines emit transient onroad events that produce
a single audible chime. They never touch actuation, engagement, or the
WP-Mod / min-steer code paths.
"""
import time

from cereal import log
from openpilot.common.cached_params import CachedParams

EventName = log.OnroadEvent.EventName

STOPPED_VEGO = 0.3        # m/s, "stopped" threshold
ARM_TIME = 3.0            # seconds of steady state before a chime can fire
COOLDOWN = 20.0           # minimum seconds between chimes per alert type

# lead depart
LEAD_ARM_DIST = 15.0      # lead must be within this to arm (m)
LEAD_DEPART_DIST_GAIN = 5.0   # lead moving this far away = departing (m)
LEAD_DEPART_VREL = 2.0        # or pulling away this fast (m/s)

# green light
GREEN_NO_LEAD_DIST = 20.0     # no lead closer than this to arm (m)
GREEN_MODEL_VEGO = 2.0        # model predicts we will be moving at least this fast (m/s)


class JveAlertsHelper:
  def __init__(self):
    self.cached_params = CachedParams()
    self.lead_arm_time: float | None = None
    self.lead_arm_drel: float = 0.0
    self.lead_last_chime = 0.0
    self.green_arm_time: float | None = None
    self.green_last_chime = 0.0
    self.green_fired = False
    self.green_model_prev = False

  def update(self, sm, CS) -> list:
    now = time.monotonic()
    events = []
    lead = sm['radarState'].leadOne
    stopped = CS.vEgo < STOPPED_VEGO

    # Lead depart: stopped behind a close lead for a while, then it pulls away.
    # Departure is evaluated whenever armed, regardless of current distance.
    if self.cached_params.get_bool("jvePilot.settings.alerts.leadDepart", 5000):
      lead_valid = lead.status > 0
      if not stopped:
        self.lead_arm_time = None
      elif self.lead_arm_time is not None:
        departing = lead_valid and (lead.dRel > self.lead_arm_drel + LEAD_DEPART_DIST_GAIN or
                                   lead.vRel > LEAD_DEPART_VREL)
        if (departing and now - self.lead_arm_time > ARM_TIME and
            now - self.lead_last_chime > COOLDOWN):
          events.append(EventName.jveLeadDepart)
          self.lead_last_chime = now
          self.lead_arm_time = None
        elif not lead_valid:
          self.lead_arm_time = None
      elif lead_valid and lead.dRel < LEAD_ARM_DIST:
        self.lead_arm_time = now
        self.lead_arm_drel = lead.dRel
    else:
      self.lead_arm_time = None

    # Green light: stopped with nothing ahead and the model's NEAR-TERM
    # prediction (1.5-3s out) transitions from quiet to expecting movement.
    # Fires at most once per stop (latch resets when the car moves).
    if self.cached_params.get_bool("jvePilot.settings.alerts.greenLight", 5000):
      no_lead = lead.status == 0 or lead.dRel > GREEN_NO_LEAD_DIST
      vx = list(sm['modelV2'].velocity.x)
      near_term = vx[3:6] if len(vx) >= 6 else vx[3:]
      model_moving = len(near_term) > 0 and max(near_term) > GREEN_MODEL_VEGO
      if not stopped:
        self.green_arm_time = None
        self.green_fired = False
        self.green_model_prev = False
      elif not no_lead:
        self.green_arm_time = None
        self.green_model_prev = model_moving
      else:
        if self.green_arm_time is None:
          self.green_arm_time = now
        rising = model_moving and not self.green_model_prev
        self.green_model_prev = model_moving
        if (rising and not self.green_fired and now - self.green_arm_time > ARM_TIME and
            now - self.green_last_chime > COOLDOWN):
          events.append(EventName.jveGreenLight)
          self.green_last_chime = now
          self.green_fired = True
    else:
      self.green_arm_time = None
      self.green_fired = False
      self.green_model_prev = False

    return events
