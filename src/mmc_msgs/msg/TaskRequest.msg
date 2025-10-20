# mmc_msgs/TaskRequest
# High-level task intent for the coordinator_v1. One message can request
# navigation, perception, or both, and indicate the desired sequencing.

# --- Identity & timing -------------------------------------------------------
std_msgs/Header header           # .frame_id free-form (e.g., "map"); .stamp is when LLM created the task
string request_id                # unique id from LLM (UUID or incremental)
builtin_interfaces/Duration timeout  # 0s => no overall timeout at coordinator level

# --- Sequencing --------------------------------------------------------------
# Choose how to order sub-tasks when BOTH are requested.
uint8 SEQ_NAV_THEN_PERCEPTION=0
uint8 SEQ_PERCEPTION_THEN_NAV=1
uint8 SEQ_PARALLEL=2            # coordinator may still serialize internally

uint8 sequence_mode             # one of the constants above (default 0 if unset by sender)

# --- Navigation (optional) ---------------------------------------------------
bool do_navigate                # if true, use nav_target
geometry_msgs/PoseStamped nav_target  # target pose; header.frame_id must be valid (e.g., "map")

# Optional nav tuning (coordinator may ignore if 0)
float32 arrive_distance_tolerance  # meters; 0 => use default
float32 yaw_tolerance              # radians; 0 => use default

# --- Perception (optional) ---------------------------------------------------
bool do_perception              # if true, run DetectObject with below params
string class_id                 # e.g., "red_ball" or "green_cube"
string target_color             # e.g., "red", "green" (free-form; detector may ignore)
string detector                 # e.g., "yolo", "ssd" (free-form; used by perception node)

# Optional perception tuning (coordinator may ignore if 0/empty)
float32 min_confidence          # [0,1]; 0 => use default

# --- Misc --------------------------------------------------------------------
string note                     # free-form comment from LLM (for logging/debug)
