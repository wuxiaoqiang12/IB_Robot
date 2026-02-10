"""Contract extension loader for robot_config integration.

This module extends the rosetta contract system to support peripheral references
from robot_config, eliminating duplication between hardware configuration and ML I/O.
"""

from pathlib import Path
from typing import Any, Dict, List, Optional, Union

from rosetta.common.contract_utils import (
    Contract,
    ObservationSpec,
    ActionSpec,
    _as_align,
)
import yaml


def load_contract_with_robot_config(
    contract_path: Union[str, Path],
    robot_config: Optional[Any] = None,
) -> Contract:
    """Load contract with robot_config peripheral integration.

    This function loads a rosetta contract and resolves peripheral references
    from the robot_config, eliminating the need to duplicate camera
    configurations in both places.

    Args:
        contract_path: Path to contract YAML file
        robot_config: RobotConfig object from robot_config package

    Returns:
        Contract with resolved peripheral metadata

    Example:
        In contract YAML:
        ```yaml
        observations:
          - key: observation.images.top
            topic: /camera/top
            peripheral: top  # References camera 'top' from robot_config
        ```

        The peripheral metadata (width, height, fps, frame_id) will be
        automatically loaded from robot_config.peripherals.
    """
    # Load base contract
    contract_data = yaml.safe_load(Path(contract_path).read_text(encoding="utf-8")) or {}

    # Resolve peripheral references if robot_config is provided
    if robot_config:
        contract_data = _resolve_peripheral_references(contract_data, robot_config)

    # Build contract dataclasses
    def _obs(it: Dict[str, Any]) -> ObservationSpec:
        obs = ObservationSpec(
            key=it["key"],
            topic=it["topic"],
            type=it["type"],
            selector=it.get("selector"),
            image=it.get("image"),
            align=_as_align(it.get("align")),
            qos=it.get("qos"),
        )
        # Add peripheral metadata if available
        if "_peripheral" in it:
            object.__setattr__(obs, "_peripheral", it["_peripheral"])
        return obs

    def _act(it: Dict[str, Any]) -> ActionSpec:
        pub = it["publish"]
        sb = str(it.get("safety_behavior", "zeros")).lower().strip()
        if sb not in ("zeros", "hold"):
            sb = "zeros"
        return ActionSpec(
            key=it["key"],
            publish_topic=pub["topic"],
            type=pub["type"],
            selector=it.get("selector"),
            from_tensor=it.get("from_tensor"),
            publish_qos=pub.get("qos"),
            publish_strategy=pub.get("strategy"),
            safety_behavior=sb,
        )

    def _task(it: Dict[str, Any]) -> Any:
        from rosetta.common.contract_utils import TaskSpec
        return TaskSpec(
            key=it.get("key", it["topic"]),
            topic=it["topic"],
            type=it["type"],
            qos=it.get("qos"),
        )

    obs = [_obs(it) for it in (contract_data.get("observations") or [])]
    acts = [_act(it) for it in (contract_data.get("actions") or [])]
    tks = [_task(it) for it in (contract_data.get("tasks") or [])]
    rec = contract_data.get("recording") or {}
    proc = contract_data.get("process") or {}

    from rosetta.common.contract_utils import Contract
    return Contract(
        name=contract_data.get("name", "contract"),
        version=int(contract_data.get("version", 1)),
        rate_hz=float(contract_data.get("rate_hz", contract_data.get("fps", 20.0))),
        max_duration_s=float(contract_data.get("max_duration_s", 30.0)),
        observations=obs,
        actions=acts,
        tasks=tks,
        recording=rec,
        robot_type=contract_data.get("robot_type"),
        timestamp_source=str(contract_data.get("timestamp_source", "receive")).lower(),
        process=proc,
    )


def _resolve_peripheral_references(contract_data: Dict, robot_config: Any) -> Dict:
    """Resolve peripheral references in contract using robot_config.

    For observations that reference a peripheral (by name), this function
    adds peripheral metadata from robot_config to the observation.

    Args:
        contract_data: Contract dictionary
        robot_config: RobotConfig object

    Returns:
        Updated contract dictionary with resolved peripheral metadata
    """
    # Build peripheral lookup
    peripherals = {}
    for cam in robot_config.peripherals:
        peripherals[cam.name] = {
            "type": "camera",
            "driver": cam.driver,
            "width": cam.width,
            "height": cam.height,
            "fps": cam.fps,
            "frame_id": cam.frame_id,
            "optical_frame_id": cam.optical_frame_id,
            "pixel_format": cam.pixel_format,
        }

    # Resolve peripheral references in observations
    for obs in contract_data.get("observations", []):
        peripheral_name = obs.get("peripheral")
        if peripheral_name and peripheral_name in peripherals:
            # Add peripheral metadata
            obs["_peripheral"] = peripherals[peripheral_name]

            # Auto-fill image resize if not specified
            if "image" not in obs or obs["image"] is None:
                obs["image"] = {}
            if "resize" not in obs["image"] or obs["image"]["resize"] is None:
                # Default to camera resolution (height, width)
                cam = peripherals[peripheral_name]
                obs["image"]["resize"] = [cam["height"], cam["width"]]

            # Infer encoding from pixel format
            if "encoding" not in obs["image"] or obs["image"]["encoding"] is None:
                pixel_format = peripherals[peripheral_name]["pixel_format"]
                obs["image"]["encoding"] = pixel_format

    return contract_data


def validate_contract_peripheral_consistency(
    contract_data: Dict,
    robot_config: Any,
) -> List[str]:
    """Validate that all peripheral references in contract exist in robot_config.

    Args:
        contract_data: Contract dictionary
        robot_config: RobotConfig object

    Returns:
        List of error messages (empty if valid)
    """
    errors = []
    peripheral_names = {cam.name for cam in robot_config.peripherals}

    for obs in contract_data.get("observations", []):
        peripheral_name = obs.get("peripheral")
        if peripheral_name:
            if peripheral_name not in peripheral_names:
                errors.append(
                    f"Observation '{obs.get('key')}' references undefined peripheral: {peripheral_name}"
                )

    return errors


def generate_contract_from_robot_config(robot_config: Any) -> str:
    """Generate a complete rosetta contract from robot_config.

    This function creates a contract YAML that references peripherals
    defined in robot_config, suitable for use with PolicyBridge and EpisodeRecorder.

    Args:
        robot_config: RobotConfig object

    Returns:
        Contract YAML as string
    """
    contract = {
        "name": robot_config.name,
        "version": 1,
        "robot_type": robot_config.robot_type,
        "rate_hz": robot_config.contract.rate_hz,
        "max_duration_s": robot_config.contract.max_duration_s,
        "observations": [],
        "actions": [],
        "recording": {"storage": "mcap"},
    }

    # Generate observations from contract extension config
    for obs in robot_config.contract.observations:
        obs_dict = {
            "key": obs.key,
            "topic": obs.topic,
            "type": "sensor_msgs/msg/Image" if obs.peripheral else "sensor_msgs/msg/JointState",
        }

        if obs.peripheral:
            obs_dict["peripheral"] = obs.peripheral
            # Image parameters
            if obs.image:
                obs_dict["image"] = obs.image
            else:
                # Use camera defaults from peripheral
                cam = robot_config.get_camera(obs.peripheral)
                if cam:
                    obs_dict["image"] = {"resize": [cam.height, cam.width]}

            if obs.align:
                obs_dict["align"] = obs.align
            if obs.qos:
                obs_dict["qos"] = obs.qos
        else:
            # Non-peripheral observation (e.g., joint states)
            if obs.selector:
                obs_dict["selector"] = obs.selector
            if obs.align:
                obs_dict["align"] = obs.align
            if obs.qos:
                obs_dict["qos"] = obs.qos

        contract["observations"].append(obs_dict)

    # Generate actions from contract extension config
    for action in robot_config.contract.actions:
        action_dict = {
            "key": action.key,
            "publish": action.publish,
            "safety_behavior": action.safety_behavior,
        }

        if action.selector:
            action_dict["selector"] = action.selector
        if action.from_tensor:
            action_dict["from_tensor"] = action.from_tensor

        contract["actions"].append(action_dict)

    return yaml.dump(contract, default_flow_style=False, sort_keys=False)
