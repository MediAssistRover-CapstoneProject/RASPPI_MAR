"""Per-selection search time; ordinary travel and suspended time never count."""
import asyncio
import math
import time
from .contracts import StepError


class PathSearchBudget:
    def __init__(self, policy, store=None):
        self.limit = float(policy['budget_sec'])
        self.selection_id = policy['selection_id']
        self.recovery_limit = policy['max_recovery_attempts']
        self.store = store
        self.used, self.recovery_used = store.path_search_state(self.selection_id) if store is not None else (0., 0)
        self.started = None
        self.depth = 0
        self.no_path = False
        self.infrastructure_pending = False

    @property
    def remaining(self):
        elapsed = 0. if self.started is None else max(0., time.monotonic() - self.started)
        return max(0., self.limit - self.used - elapsed)

    def enter(self):
        if self.depth == 0:
            self.started = time.monotonic()
        self.depth += 1

    def leave(self):
        self.depth -= 1
        if self.depth == 0:
            self.used = min(self.limit, self.used + max(0., time.monotonic() - self.started))
            self.started = None
            if self.store is not None:
                self.store.charge_path_search(self.selection_id, self.used, self.recovery_used)

    def recover(self):
        if self.recovery_used >= self.recovery_limit:
            return False
        self.recovery_used += 1
        if self.store is not None:
            self.store.charge_path_search(self.selection_id, self.used, self.recovery_used)
        return True

    async def run(self, action):
        if self.depth:
            return await action()
        remaining = self.remaining
        if remaining <= 0:
            raise StepError('PATH_SEARCH_BUDGET_EXHAUSTED')
        self.enter()
        try:
            return await asyncio.wait_for(action(), remaining)
        except asyncio.TimeoutError:
            raise StepError('PATH_SEARCH_BUDGET_EXHAUSTED') from None
        finally:
            self.leave()


def policy(value):
    if value is None:
        return None
    from uuid import UUID
    if not isinstance(value, dict) or set(value) != {
            'budget_sec', 'max_recovery_attempts', 'selection_id', 'fallback_trigger'}:
        raise StepError('INVALID_PATH_SEARCH_POLICY')
    seconds, attempts = value['budget_sec'], value['max_recovery_attempts']
    if (type(seconds) not in (int, float) or not math.isfinite(seconds) or not 5 <= seconds <= 30
            or type(attempts) is not int or not 0 <= attempts <= 1
            or value['fallback_trigger'] != 'ON_PATH_UNAVAILABLE'):
        raise StepError('INVALID_PATH_SEARCH_POLICY')
    try:
        UUID(value['selection_id'])
    except (ValueError, TypeError, AttributeError):
        raise StepError('INVALID_PATH_SEARCH_POLICY') from None
    return value
