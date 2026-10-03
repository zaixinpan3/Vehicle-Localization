"""Configure an uncluttered mapping scene before generating sensor returns.

The profile acts on simulator objects, never on recorded point labels. It
retains structural landmarks and does not destroy pre-existing live actors.
Use a fresh, dedicated, unpopulated world when applying ``mapping-static``;
CARLA does not expose the previous visibility state of environment objects.
"""
from collections import Counter
import re

MOVABLE_LABELS = frozenset({"Car", "Truck", "Bus", "Train", "Motorcycle", "Bicycle",
                            "Pedestrians", "Rider", "Dynamic"})
STRUCTURAL_LABELS = frozenset({"Buildings", "Walls", "Roads", "RoadLines", "Sidewalks",
                               "Ground", "Terrain", "Poles", "TrafficLight", "TrafficSigns",
                               "Vegetation", "Fences", "GuardRail", "Bridge"})
STRUCTURAL_NAME = re.compile(r"street.?light|highway.?light|lamp|pole|curb|sidewalk|traffic.?light|traffic.?sign|road_|building|wall", re.I)
CLUTTER_NAME = re.compile(r"dumpster|trash|garbage|waste.?bin|food.?cart|mail.?box|carla.?cola|vending|bench|picnic|pallet|traffic.?cone|construction.?barrier", re.I)


def removal_reason(label, name):
    """Return a declared removal reason, protecting structural classes first."""
    if label in STRUCTURAL_LABELS or STRUCTURAL_NAME.search(name):
        return None
    if label in MOVABLE_LABELS:
        return "movable semantic class"
    if CLUTTER_NAME.search(name):
        return "named movable street prop"
    return None


def select_environment_objects(objects):
    """Account for CARLA disabling whole actors for ordinary mesh records."""
    rows = [dict(id=int(item.id), name=item.name, label=str(item.type)) for item in objects]
    groups = {}
    for row in rows:
        # Instanced meshes are individually toggled; ordinary SM/SKM records
        # are actor-wide in UObjectRegister::EnableEnvironmentObject.
        owner = re.sub(r"_(?:SM|SKM)_\d+$", "", row["name"])
        groups.setdefault(owner, []).append(row)
    selected, kept = [], Counter()
    for members in groups.values():
        protected = any(row["label"] in STRUCTURAL_LABELS or STRUCTURAL_NAME.search(row["name"])
                        for row in members)
        targeted = any(removal_reason(row["label"], row["name"]) for row in members)
        for row in members:
            if targeted and not protected:
                selected.append(dict(**row, reason=removal_reason(row["label"], row["name"])
                                     or "same actor as selected movable mesh"))
            else:
                kept[row["label"]] += 1
    return selected, kept


def apply_scene_profile(world, profile="mapping-static", exclude_actor_ids=()):
    """Disable selected environment meshes and collisions; return an audit.

    Unknown static geometry is preserved. The generic Props layer is deliberately
    retained because some CARLA versions store lamp parts outside the Poles class.
    Call before spawning the ego in a fresh world. Active traffic causes a clear
    error instead of silently removing another client's actors. Re-enable only
    returned object IDs with ``restore_scene_profile`` when capture finishes.
"""
    if profile not in {"original", "mapping-static"}:
        raise ValueError(f"Unknown CARLA scene profile: {profile}")
    if profile == "original":
        return dict(profile=profile, disabledObjects=[], mutationApplied=False)
    import carla
    excluded = set(exclude_actor_ids)
    traffic = [actor for actor in world.get_actors()
               if actor.id not in excluded and
               (actor.type_id.startswith("vehicle.") or actor.type_id.startswith("walker."))]
    if traffic:
        raise RuntimeError("mapping-static requires a fresh, unpopulated dedicated world; "
                           f"found {len(traffic)} live traffic actors")
    objects = world.get_environment_objects(carla.CityObjectLabel.Any)
    selected, kept = select_environment_objects(objects)
    ids = {item["id"] for item in selected}
    if ids:
        world.enable_environment_objects(ids, False)
    return dict(profile=profile, disabledObjects=selected, mutationApplied=bool(ids),
                disabledByClass=dict(Counter(item["label"] for item in selected)),
                preservedByClass=dict(kept), registeredObjects=len(objects),
                pointLabelFiltering=False, liveActorsDestroyed=False,
                method="Disable environment meshes and collision before ray casting; retain structural classes and unknown static objects")


def restore_scene_profile(world, audit):
    """Restore this capture's environment objects on its dedicated server."""
    ids = {item["id"] for item in audit.get("disabledObjects", [])}
    if ids:
        world.enable_environment_objects(ids, True)
