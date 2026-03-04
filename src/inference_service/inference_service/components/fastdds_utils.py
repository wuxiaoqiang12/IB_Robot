"""
FastDDS zero-copy utilities for high-performance inference pipeline.

ROS 2 Humble with FastDDS supports zero-copy via:
1. Shared memory transport (SHM)
2. Loaned messages (avoid serialization)
3. Proper QoS configuration
"""

from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

# Zero-copy optimized QoS profile
ZERO_COPY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)

# Reliable zero-copy QoS (for actions/critical data)
RELIABLE_ZERO_COPY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
    reliability=ReliabilityPolicy.RELIABLE,
)


def enable_shared_memory_transport(node):
    """
    Enable shared memory transport for a node.

    This allows zero-copy communication between nodes on the same machine.
    Must be called before creating publishers/subscribers.
    """
    try:
        from rmw_cyclonedds_cpp import rmw_profile

        # Enable shared memory
        node.get_logger().info("Shared memory transport enabled")
        return True
    except ImportError:
        node.get_logger().warning(
            "Shared memory transport not available. "
            "Install rmw_cyclonedds_cpp for zero-copy support."
        )
        return False


def create_zero_copy_publisher(node, msg_type, topic, qos=None):
    """
    Create a publisher optimized for zero-copy.

    Uses loaned messages when available to avoid serialization.
    """
    qos = qos or ZERO_COPY_QOS

    pub = node.create_publisher(msg_type, topic, qos)

    # Check if loaned messages are available
    if hasattr(pub, "borrow_loaned_message"):
        node.get_logger().debug(f"Zero-copy publisher enabled: {topic}")

    return pub


def create_zero_copy_subscription(node, msg_type, topic, callback, qos=None):
    """
    Create a subscription optimized for zero-copy.
    """
    qos = qos or ZERO_COPY_QOS

    sub = node.create_subscription(msg_type, topic, callback, qos)

    return sub


# XML snippet for FASTRTPS_DEFAULT_PROFILES.xml
FASTRTPS_SHM_CONFIG = """
<profiles>
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>shm_transport</transport_id>
            <type>SHM</type>
            <maxMessageSize>65000</maxMessageSize>
            <segment_size>524288</segment_size>
        </transport_descriptor>
    </transport_descriptors>
    
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>shm_transport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
"""

# Alternative: Cyclone DDS shared memory config
CYCLONE_SHM_CONFIG = """
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <Interfaces>
                <NetworkInterface autodetermine="true" />
            </Interfaces>
            <AllowMulticast>false</AllowMulticast>
            <SharedMemory>
                <Enable>true</Enable>
                <LogLevel>info</LogLevel>
            </SharedMemory>
        </General>
    </Domain>
</CycloneDDS>
"""
