#!/usr/bin/env python3
"""Run the normal SITL trial with an initial no-target observation."""
from validate_segments import Trial, main

new_trial = Trial.new
normal = Trial.normal


def new(self, label, *args, **kwargs):
    if label == 'nominal':
        kwargs['behavior'] = 'search'
        kwargs['search'] = True
    return new_trial(self, label, *args, **kwargs)


def search_trial(self):
    normal(self)
    mission = next(c['mission_id'] for c in self.checks if c['name']=='segmented_approach_completed')
    missing = next(t for t,k,d in self.trace if k=='detect_result' and
                   d['mission_id']==mission and d['reason']=='target_not_found')
    next_capture = next(t for t,k,d in self.trace if t>missing and
                        k=='detect_result' and d['mission_id']==mission)
    issued = next(d for t,k,d in self.trace if missing<t<next_capture and
                  k=='plan_request' and d['mission_id']==mission)
    assert any(missing<t<next_capture and k=='flight' and d['mode']=='TRACK' and
               d['context'] and d['context'][:2]==[mission,issued['goal_id']]
               for t,k,d in self.trace)
    self.check('search_moves_through_real_ego',goal_id=issued['goal_id'])


Trial.new = new
Trial.normal = search_trial
main()
