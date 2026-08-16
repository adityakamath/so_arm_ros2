from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# Best-effort, keep-last-1: realtime command/state streams where a stale sample should just be
# dropped in favor of the next one, not queued or retried.
REALTIME_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Reliable + transient-local so a late-joining subscriber gets the last-published
# robot_description instead of missing it entirely.
ROBOT_DESCRIPTION_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# Reliable + transient-local so a late-joining subscriber immediately learns a SetBool-backed
# service's current state from the topic, instead of having to wait for (or introspect) the
# next call. Used for every 'X_active' status topic published alongside a SetBool service.
LATCHED_BOOL_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
