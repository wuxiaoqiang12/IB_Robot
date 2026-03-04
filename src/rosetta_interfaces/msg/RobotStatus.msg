# Robot Status Message
# Unified status reporting from action_dispatch executors
# Published by both TopicExecutor and ActionExecutor for consistent monitoring

# Motion status
bool is_moving                    # Robot is currently executing motion
bool is_healthy                   # No errors or safety issues
string executor_type              # "topic" or "action"

# Goal tracking (for action mode)
string current_goal_id            # ID of current goal (if action mode)
float32 progress                  # Execution progress (0.0 to 1.0)

# Error information
string error_message              # Error description if unhealthy
int32 error_code                  # Error code (0 = no error)

# Timing information
float32 execution_time_sec        # Time since goal started
float32 estimated_remaining_sec   # Estimated time remaining

# Queue status (for topic mode)
int32 queue_size                  # Number of actions in queue
