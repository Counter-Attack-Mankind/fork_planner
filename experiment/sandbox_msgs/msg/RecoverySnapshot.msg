std_msgs/Header header

uint8 PHASE_NONE=0
uint8 PHASE_RETREAT=1
uint8 PHASE_RETREAT_HOLD=2
uint8 PHASE_PASS=3
uint8 PHASE_CLEAR=4
uint8 PHASE_UNRESOLVED=5
uint8 PHASE_ABORT=6
uint8 PHASE_NLP_PREP=7

uint8 recovery_phase
uint64 recovery_id
int32 retreat_vehicle_id
int32 pass_vehicle_id
int32 retreat_path_gen
int32 pass_path_gen
int32 retreat_attempt
# Vehicles in the confirmed directed wait cycle, in cycle order.
int32[] deadlock_cycle_ids
# Vehicles held for this transaction. This may include upstream waiters that
# are not members of deadlock_cycle_ids.
int32[] frozen_vehicle_ids
# Kept for wire/source compatibility with the original recovery prototype.
# It is identical to frozen_vehicle_ids; the recovery optimizer must not treat
# it as the original wait cycle.
int32[] nlp_vehicle_ids
string reason

sandbox_msgs/RecoveryVehicleState[] vehicles
