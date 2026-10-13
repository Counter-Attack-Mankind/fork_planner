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
int32 retreat_vehicle_id
int32 pass_vehicle_id
int32 retreat_path_gen
int32 pass_path_gen
int32 retreat_attempt
int32[] nlp_vehicle_ids
string reason

sandbox_msgs/RecoveryVehicleState[] vehicles
